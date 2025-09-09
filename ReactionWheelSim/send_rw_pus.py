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
SUB_TM_STATUS = 130        # TM subservice for STATUS reply (128 + 3)

# DeviceHandler modes (FSFW default enum values)
MODE_OFF    = 0
MODE_ON     = 1
MODE_NORMAL = 2
MODE_RAW    = 3

# Default destination object (objects::RW_CMD_HANDLER, big-endian in AppData)
RW_CMD_HANDLER_OID = 0x00004402

# Default TM drain window after sending a TC (seconds)
TM_DRAIN_DEFAULT = 100.0

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
    # One connected UDP socket to send and receive TM from the same source port
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect((UDP_HOST, UDP_PORT))   # fix local port and remote addr
    s.settimeout(0.3)                 # small timeout for recv()
    return s

def send_tc(sock: socket.socket, packet: bytes):
    sock.send(packet)  # connected socket

def drain_tm(sock: socket.socket, oid: int, total_time_s: float = TM_DRAIN_DEFAULT) -> bool:
    """
    Read TM for up to total_time_s seconds and decode TM_STATUS packets.
    Returns True if at least one TM_STATUS was received and decoded.
    """
    saw_tm_status = False
    deadline = time.time() + total_time_s
    while time.time() < deadline:
        try:
            data = sock.recv(4096)
        except socket.timeout:
            break
        if handle_tm(data, oid):
            saw_tm_status = True
    if not saw_tm_status:
        print("(!) No TM_STATUS (svc=220, sub=130) received in window.")
    return saw_tm_status

def handle_tm(data: bytes, oid: int) -> bool:
    """
    Minimal PUS TM parsing. Prints all TMs and specifically decodes TM_STATUS app data.
    Returns True if this packet was a TM_STATUS with valid payload, else False.
    """
    if len(data) < 10:
        return False
    service  = data[7]
    subsvc   = data[8]
    print(f"TM: svc={service} subsvc={subsvc} len={len(data)}")
    if service != SERVICE or subsvc != SUB_TM_STATUS:
        return False

    # AppData from RwPusService: oid(4) | speed(i16) | torque(i16) | running(u8)
    # Find the 4-byte object id as a marker inside the TM after the TM sec-hdr.
    marker = oid.to_bytes(4, "big")
    i = data.find(marker, 9)  # search after PUS TM secondary header
    if i == -1 or i + 9 > len(data):
        print("TM_STATUS: could not locate AppData marker in TM")
        return False

    speed   = int.from_bytes(data[i+4:i+6], "big", signed=True)
    torque  = int.from_bytes(data[i+6:i+8], "big", signed=True)
    running = data[i+8]
    print(f"RW TM_STATUS: oid=0x{oid:08X}  speed={speed} rpm  torque={torque} mNm  running={running}")
    return True

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
  watch <period_s>     - Periodically request STATUS every <period_s> seconds
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
            print(f"UDP={UDP_HOST}:{UDP_PORT}  APID=0x{APID:03X}  OID=0x{oid:08X}")

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
                drain_tm(sock, oid, TM_DRAIN_DEFAULT)  # read TM after command
            except ValueError:
                print("Invalid RPM")

        elif cmd == "status":
            print("Requesting STATUS")
            rw_status(sock, oid, next(seq))
            drain_tm(sock, oid, TM_DRAIN_DEFAULT)

        elif cmd == "stop":
            print("Sending STOP")
            rw_stop(sock, oid, next(seq))
            drain_tm(sock, oid, TM_DRAIN_DEFAULT)

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
            drain_tm(sock, oid, TM_DRAIN_DEFAULT)

        elif cmd == "watch":
            if len(parts) < 2:
                print("usage: watch <period_seconds>")
                continue
            try:
                period = float(parts[1])
                if period <= 0:
                    raise ValueError
            except ValueError:
                print("Invalid period")
                continue
            print(f"Watching STATUS every {period}s. Press Ctrl-C to stop.")
            try:
                while True:
                    rw_status(sock, oid, next(seq))
                    # Drain for most of the period to catch TM; then sleep remainder
                    start = time.time()
                    drain_tm(sock, oid, max(0.2, period * 0.8))
                    elapsed = time.time() - start
                    time.sleep(max(0.0, period - elapsed))
            except KeyboardInterrupt:
                print("\nStopped watching.")
                continue

        else:
            print("Unknown command. Type 'help'.")

if __name__ == "__main__":
    main()
