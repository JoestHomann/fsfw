#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket, struct, itertools, binascii, time

# --------- Transport to FSFW UDP bridge (hosted example) ----------
UDP_HOST = "127.0.0.1"
UDP_PORT = 7301

# --------- PUS / CCSDS basics ----------
APID    = 0x00EF
SRC_ID  = 0x0001

# ---- PUS Service 8 (Action / Function Management) ----
SERVICE_ACT         = 8
# FSFW Service 8 uses subservice 128 for direct commanding (perform action)
SUB_PERFORM_ACTION  = 128
SUB_DATA_REPLY      = 130   # TM[8,130]

# ---- PUS Service 5 (Mode Management) ----
SERVICE_MODE        = 5
SUB_SET_MODE        = 1

# Action IDs (map 1:1 to your DeviceHandler command IDs)
ACT_SET_SPEED = 0x00000001
ACT_STOP      = 0x00000002
ACT_STATUS    = 0x00000003

# DeviceHandler modes (FSFW default enum values)
MODE_OFF    = 0
MODE_ON     = 1
MODE_NORMAL = 2
MODE_RAW    = 3

# Default destination object (objects::RW_CMD_HANDLER, big-endian in AppData)
RW_CMD_HANDLER_OID = 0x00004402

# ---- Receive tuning ----
SOCKET_TIMEOUT_S   = 0.8   # timeout for a single recv() attempt
WAIT_AFTER_CMD_S   = 3.0   # total listen window after a command

# ---------- CRC16-CCITT (FALSE): poly 0x1021, init 0xFFFF ----------
def crc16_ccitt_false(data: bytes, poly=0x1021, init=0xFFFF) -> int:
    crc = init
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ poly) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

# ---------- CCSDS + PUS-C (VER/ACK=0x2F) ----------
def ccsds_tc_primary_header(apid, seq_count, bytes_after_primary):
    # Version=0, Type=1 (TC), SecHdrFlag=1 -> 0x1800
    first  = 0x1800 | (apid & 0x07FF)
    # Sequence flags '11' + sequence count
    second = 0xC000 | (seq_count & 0x3FFF)
    # Packet Length = bytes after PH minus 1
    pkt_len = (bytes_after_primary - 1) & 0xFFFF
    return struct.pack(">HHH", first, second, pkt_len)

VER_ACK = 0x2F  # PUS-C

def pus_tc_sec_hdr(service, subservice, src_id):
    # PUS-C TC secondary header: [Version/Ack][Service][Subservice][SourceID]
    return struct.pack(">BBBH", VER_ACK, service & 0xFF, subservice & 0xFF, src_id & 0xFFFF)

def build_tc(apid, service, subservice, app_data: bytes, seq_count):
    sec = pus_tc_sec_hdr(service, subservice, SRC_ID) + app_data
    after_ph_len  = len(sec) + 2  # add 2 bytes CRC16
    ph            = ccsds_tc_primary_header(apid, seq_count, after_ph_len)
    whole_wo_crc  = ph + sec
    crc           = crc16_ccitt_false(whole_wo_crc)
    sec_crc       = sec + struct.pack(">H", crc)
    pkt           = ph + sec_crc
    # Debug print
    print(f"PUS len(after PH)={len(sec_crc)}  CRC=0x{crc:04X}")
    print("PUS field hex:", binascii.hexlify(sec_crc).decode())
    return pkt

# ---------- Helpers ----------
def be_u32(x): return struct.pack(">I", x)
def be_i16(x): return struct.pack(">h", x)
def be_f32(x): return struct.pack(">f", float(x))

def make_socket():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Connected UDP socket simplifies send/recv
    s.connect((UDP_HOST, UDP_PORT))
    s.settimeout(SOCKET_TIMEOUT_S)
    return s

def send_tc(sock: socket.socket, packet: bytes):
    sock.send(packet)

# ---------- Minimal TM decoding ----------
def print_verification_tm(service: int, subsvc: int):
    # PUS Service 1 (Verification) quick legend:
    # 1 Acceptance Success, 2 Acceptance Failure
    # 3 Start Success,     4 Start Failure
    # 5 Progress Success,  6 Progress Failure
    # 7 Completion Success,8 Completion Failure
    if service != 1:
        return
    meaning = {
        1: "Acceptance Success",
        2: "Acceptance Failure",
        3: "Start Success",
        4: "Start Failure",
        5: "Progress Success",
        6: "Progress Failure",
        7: "Completion Success",
        8: "Completion Failure",
    }.get(subsvc, "Unknown Verification TM")
    print(f"TM[1,{subsvc}]: {meaning}")

def handle_tm(data: bytes, oid: int):
    # Very lightweight PUS TM parsing: PH = 6 bytes, then PUS TM secondary header.
    if len(data) < 10:
        return
    service  = data[7]
    subsvc   = data[8]
    print(f"TM: svc={service} subsvc={subsvc} len={len(data)}")
    print_verification_tm(service, subsvc)

    # ---- Service 8, Data Reply ----
    if service == SERVICE_ACT and subsvc == SUB_DATA_REPLY:
        # FSFW Service 8 packs: [oid:4][action:4][payload...]
        start = 9  # after PUS sec header
        if len(data) < start + 8:
            print("   (TM[8,130] too short)")
            return
        app = data[start:]
        oid_be   = int.from_bytes(app[0:4], "big")
        act_be   = int.from_bytes(app[4:8], "big")
        payload  = app[8:]
        print(f"   (DataReply) oid=0x{oid_be:08X} act=0x{act_be:08X} payload_len={len(payload)}")

        # Device-specific decode: try to find the RW status frame 'AB 10 ..'
        pos = payload.find(b'\xAB\x10')
        if pos != -1 and pos + 8 <= len(payload):
            p = payload[pos:pos+8]
            speed   = int.from_bytes(p[2:4], "big", signed=True)
            torque  = int.from_bytes(p[4:6], "big", signed=True)
            running = p[6]
            print(f"   RW STATUS: speed={speed} rpm  torque={torque} mNm  running={running}")
        else:
            print("   (No embedded 'AB 10' RW frame found)")

def drain_tm(sock: socket.socket, oid: int, total_time_s: float = WAIT_AFTER_CMD_S):
    """Read TM up to total_time_s seconds and decode replies."""
    deadline = time.time() + total_time_s
    got_any = False
    while True:
        remaining = deadline - time.time()
        if remaining <= 0:
            break
        sock.settimeout(min(SOCKET_TIMEOUT_S, max(0.05, remaining)))
        try:
            data = sock.recv(4096)
        except socket.timeout:
            continue
        except Exception as e:
            print(f"(recv error: {e})")
            break
        got_any = True
        handle_tm(data, oid)
    if not got_any:
        print(f"(!) No TM received in {total_time_s:.1f}s window.")

# ---------- Command builders ----------
def rw_action(sock, oid_u32, act_id_u32, params: bytes, seq):
    # AppData: [object_id(4,BE) | action_id(4,BE) | params...]
    app = be_u32(oid_u32) + be_u32(act_id_u32) + params
    pkt = build_tc(APID, SERVICE_ACT, SUB_PERFORM_ACTION, app, seq)
    send_tc(sock, pkt)

def rw_set_speed(sock, oid_u32, rpm, seq):
    # Preferred: send 2-byte signed int (big-endian)
    rw_action(sock, oid_u32, ACT_SET_SPEED, be_i16(int(rpm)), seq)

def rw_set_speed_f(sock, oid_u32, rpm_f, seq):
    # Optional: send 4-byte float if you want to test the float path in your handler
    rw_action(sock, oid_u32, ACT_SET_SPEED, be_f32(rpm_f), seq)

def rw_stop(sock, oid_u32, seq):
    rw_action(sock, oid_u32, ACT_STOP, b"", seq)

def rw_status(sock, oid_u32, seq):
    rw_action(sock, oid_u32, ACT_STATUS, b"", seq)

def rw_set_mode(sock, oid_u32, mode, submode, seq):
    # PUS Service 5: [oid(4)][mode(1)][submode(1)]
    app = be_u32(oid_u32) + bytes([mode & 0xFF, submode & 0xFF])
    pkt = build_tc(APID, SERVICE_MODE, SUB_SET_MODE, app, seq)
    send_tc(sock, pkt)

# ---------- CLI ----------
HELP = """\
Commands:
  set <rpm>            - Send SET_SPEED as int16 (Service 8 / Subservice 128)
  setf <rpm_float>     - Send SET_SPEED as float32
  status               - Send STATUS   (Service 8 / Subservice 128)
  stop                 - Send STOP     (Service 8 / Subservice 128)
  mode <name> [sub]    - Send SET_MODE (Service 5), names: off,on,normal,raw  (default sub=0)
  listen [seconds]     - Only listen for TM and print what arrives (default 5s)
  oid <hex|dec>        - Change target object id (default 0x00004402)
  who                  - Show current settings
  help                 - Show this help
  exit/quit            - Leave
"""

MODE_NAMES = {
    "off": MODE_OFF,
    "on": MODE_ON,
    "normal": MODE_NORMAL,
    "raw": MODE_RAW,
}

def parse_int(s: str) -> int:
    s = s.strip().lower()
    if s.startswith("0x"):
        return int(s, 16)
    return int(s, 10)

def main():
    print("Interactive PUS-8 sender for RW (Subservice 128). Type 'help' for commands.")
    oid = RW_CMD_HANDLER_OID
    seq = itertools.count()
    sock = make_socket()  # one socket for TX+RX

    while True:
        try:
            line = input(">> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not line:
            continue

        parts = line.split()
        cmd = parts[0].lower()

        if cmd in ("exit", "quit"):
            break

        elif cmd == "help":
            print(HELP)

        elif cmd == "who":
            print(f"UDP={UDP_HOST}:{UDP_PORT}  APID=0x{APID:03X}  OID=0x{oid:08X}  wait={WAIT_AFTER_CMD_S}s  timeout={SOCKET_TIMEOUT_S}s")

        elif cmd == "oid":
            if len(parts) < 2:
                print("usage: oid <hex|dec>")
                continue
            try:
                oid = parse_int(parts[1])
                print(f"OID set to 0x{oid:08X}")
            except ValueError:
                print("Invalid OID")

        elif cmd == "set":
            if len(parts) < 2:
                print("usage: set <rpm>")
                continue
            try:
                rpm = int(parts[1])
                print(f"Sending SET_SPEED {rpm} rpm (int16)")
                rw_set_speed(sock, oid, rpm, next(seq))
                print(f"...listening up to {WAIT_AFTER_CMD_S}s for TM")
                drain_tm(sock, oid, WAIT_AFTER_CMD_S)
            except ValueError:
                print("Invalid RPM")

        elif cmd == "setf":
            if len(parts) < 2:
                print("usage: setf <rpm_float>")
                continue
            try:
                rpmf = float(parts[1])
                print(f"Sending SET_SPEED {rpmf} rpm (float32)")
                rw_set_speed_f(sock, oid, rpmf, next(seq))
                print(f"...listening up to {WAIT_AFTER_CMD_S}s for TM")
                drain_tm(sock, oid, WAIT_AFTER_CMD_S)
            except ValueError:
                print("Invalid RPM float")

        elif cmd == "status":
            print("Requesting STATUS")
            rw_status(sock, oid, next(seq))
            print(f"...listening up to {WAIT_AFTER_CMD_S}s for TM")
            drain_tm(sock, oid, WAIT_AFTER_CMD_S)

        elif cmd == "stop":
            print("Sending STOP")
            rw_stop(sock, oid, next(seq))
            print(f"...listening up to {WAIT_AFTER_CMD_S}s for TM")
            drain_tm(sock, oid, WAIT_AFTER_CMD_S)

        elif cmd == "mode":
            if len(parts) < 2:
                print("usage: mode <off|on|normal|raw> [submode]")
                continue
            name = parts[1].lower()
            if name not in MODE_NAMES:
                print("Unknown mode. Use: off, on, normal, raw")
                continue
            try:
                sub = parse_int(parts[2]) if len(parts) >= 3 else 0
            except ValueError:
                print("Invalid submode")
                continue
            mode_val = MODE_NAMES[name]
            print(f"Sending SET_MODE {name.upper()} (mode={mode_val}, sub={sub})")
            rw_set_mode(sock, oid, mode_val, sub, next(seq))
            print(f"...listening up to {WAIT_AFTER_CMD_S}s for TM")
            drain_tm(sock, oid, WAIT_AFTER_CMD_S)

        elif cmd == "listen":
            try:
                secs = float(parts[1]) if len(parts) >= 2 else 5.0
            except ValueError:
                secs = 5.0
            print(f"Listening for TM for {secs:.1f}s...")
            drain_tm(sock, oid, secs)

        else:
            print("Unknown command. Type 'help'.")

if __name__ == "__main__":
    main()
