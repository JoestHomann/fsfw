#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket, struct, itertools, binascii, time

# --------- Transport to FSFW UDP bridge (hosted example) ----------
UDP_HOST = "127.0.0.1"
UDP_PORT = 7301

# --------- PUS / CCSDS basics ----------
APID    = 0x00EF           # must match your RwPusService APID
SRC_ID  = 0x0001
SERVICE = 220              # RwPusService

SUB_SET_SPEED = 1
SUB_STOP      = 2
SUB_STATUS    = 3
SUB_SET_MODE  = 10
SUB_TM_STATUS = 130        # legacy compact TM
SUB_TM_STATUS_TYPED = 131  # NEW: typed TM v1

# DeviceHandler modes (FSFW default enum values)
MODE_OFF    = 0
MODE_ON     = 1
MODE_NORMAL = 2
MODE_RAW    = 3

# Default destination object (objects::RW_HANDLER, big-endian in AppData)
RW_HANDLER_OID = 0x00004402

# ---- Receive tuning ----
SOCKET_TIMEOUT_S   = 9.0   # timeout for a single recv() attempt
WAIT_AFTER_CMD_S   = 10.0  # total listen window after a command

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
    second = 0xC000 | (seq_count & 0x3FFF)      # '11' + seq
    pkt_len = (bytes_after_primary - 1) & 0xFFFF
    return struct.pack(">HHH", first, second, pkt_len)

VER_ACK = 0x2F  # PUS-C

def pus_tc_sec_hdr(service, subservice, src_id):
    return struct.pack(">BBBH", VER_ACK, service & 0xFF, subservice & 0xFF, src_id & 0xFFFF)

def build_tc(apid, service, subservice, app_data: bytes, seq_count):
    sec = pus_tc_sec_hdr(service, subservice, SRC_ID) + app_data
    after_ph_len  = len(sec) + 2  # +CRC
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

def make_socket():
    # one connected UDP socket to send and receive TM from the same source port
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect((UDP_HOST, UDP_PORT))   # fix local port and remote addr
    s.settimeout(SOCKET_TIMEOUT_S)
    return s

def send_tc(sock: socket.socket, packet: bytes):
    sock.send(packet)  # connected socket

def handle_tm(data: bytes, oid: int):
    """Decode minimal PUS TM. Prefer typed 220/131 if present; also decode legacy 220/130."""
    if len(data) < 10:
        return
    service  = data[7]
    subsvc   = data[8]
    print(f"TM: svc={service} subsvc={subsvc} len={len(data)}")

    # --- Typed TM (preferred): svc=220, sub=131, AppData v1 is 28 bytes:
    # ver(1)=1 | objectId(4) | speed(i16) | torque(i16) | running(u8)
    # | flags(u16) | error(u16) | crcErrCnt(u32) | malformedCnt(u32) | timestampMs(u32) | sampleCnt(u16)
    if service == SERVICE and subsvc == SUB_TM_STATUS_TYPED:
        # Search marker: [0x01, objectIdBE] after the PUS sec-hdr (we use a simple scan)
        marker = bytes([1]) + oid.to_bytes(4, "big")
        start = data.find(marker, 9)  # 9 ~ just after our coarse PUS sec-hdr sniff
        if start != -1 and start + 28 <= len(data):
            raw_app = data[start:start+28]
            # Unpack as big-endian
            ver, oid_be, speed, torque, running, flags, err, crc_cnt, mal_cnt, ts_ms, sample = \
                struct.unpack(">B I h h B H H I I I H", raw_app)
            print(
                f"RW TM_STATUS_TYPED v{ver}: oid=0x{oid_be:08X}  "
                f"speed={speed} rpm  torque={torque} mNm  running={running}  "
                f"flags=0x{flags:04X} err=0x{err:04X}  "
                f"crcErrCnt={crc_cnt} malformedCnt={mal_cnt} tsMs={ts_ms} sample={sample}"
            )
        return

    # --- Legacy compact TM: svc=220, sub=130
    if service == SERVICE and subsvc == SUB_TM_STATUS:
        # AppData layout (legacy): oid(4)|speed(i16)|torque(i16)|running(u8)
        marker = oid.to_bytes(4, "big")
        start = data.find(marker, 9)  # search after PUS sec-hdr
        if start != -1 and start + 9 <= len(data):
            speed   = int.from_bytes(data[start+4:start+6], "big", signed=True)
            torque  = int.from_bytes(data[start+6:start+8], "big", signed=True)
            running = data[start+8]
            print(f"RW TM_STATUS: oid=0x{oid:08X}  speed={speed} rpm  torque={torque} mNm  running={running}")

def drain_tm(sock: socket.socket, oid: int, total_time_s: float = WAIT_AFTER_CMD_S):
    """Read TM up to total_time_s seconds and decode TM_STATUS packets."""
    deadline = time.time() + total_time_s
    got_any = False
    while True:
        remaining = deadline - time.time()
        if remaining <= 0:
            break
        # keep per-read timeout bounded
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
def rw_set_speed(sock, oid_u32, rpm, seq):
    app = be_u32(oid_u32) + be_i16(rpm)
    pkt = build_tc(APID, SERVICE, SUB_SET_SPEED, app, seq)
    send_tc(sock, pkt)

def rw_stop(sock, oid_u32, seq):
    app = be_u32(oid_u32)
    pkt = build_tc(APID, SERVICE, SUB_STOP, app, seq)
    send_tc(sock, pkt)

def rw_status(sock, oid_u32, seq):
    app = be_u32(oid_u32)
    pkt = build_tc(APID, SERVICE, SUB_STATUS, app, seq)
    send_tc(sock, pkt)

def rw_set_mode(sock, oid_u32, mode, submode, seq):
    app = be_u32(oid_u32) + bytes([mode & 0xFF, submode & 0xFF])
    pkt = build_tc(APID, SERVICE, SUB_SET_MODE, app, seq)
    send_tc(sock, pkt)

# ---------- CLI ----------
HELP = """\
Commands:
  set <rpm>            - Send SET_SPEED with signed RPM (e.g. set 1000)
  status               - Send STATUS request
  stop                 - Send STOP
  mode <name> [sub]    - Send SET_MODE, names: off,on,normal,raw  (default sub=0)
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
    print("Interactive PUS-220 sender for RW. Type 'help' for commands.")
    oid = RW_HANDLER_OID
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
                print(f"Sending SET_SPEED {rpm} rpm")
                rw_set_speed(sock, oid, rpm, next(seq))
                print(f"...listening up to {WAIT_AFTER_CMD_S}s for TM")
                drain_tm(sock, oid, WAIT_AFTER_CMD_S)
            except ValueError:
                print("Invalid RPM")

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
            secs = float(parts[1]) if len(parts) >= 2 else 5.0
            print(f"Listening for TM for {secs:.1f}s...")
            drain_tm(sock, oid, secs)

        else:
            print("Unknown command. Type 'help'.")

if __name__ == "__main__":
    main()
