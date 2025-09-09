#!/usr/bin/env python3
import socket, struct, itertools, binascii

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

# DeviceHandler modes (FSFW default enum values)
MODE_OFF    = 0
MODE_ON     = 1
MODE_NORMAL = 2
MODE_RAW    = 3

# Default destination object (objects::RW_CMD_HANDLER, big-endian in AppData)
RW_CMD_HANDLER_OID = 0x00004402

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

def send_tc(packet: bytes):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.sendto(packet, (UDP_HOST, UDP_PORT))

# ---------- Command builders ----------
def rw_set_speed(oid_u32, rpm, seq):
    app = be_u32(oid_u32) + be_i16(rpm)
    pkt = build_tc(APID, SERVICE, SUB_SET_SPEED, app, seq)
    send_tc(pkt)

def rw_stop(oid_u32, seq):
    app = be_u32(oid_u32)
    pkt = build_tc(APID, SERVICE, SUB_STOP, app, seq)
    send_tc(pkt)

def rw_status(oid_u32, seq):
    app = be_u32(oid_u32)
    pkt = build_tc(APID, SERVICE, SUB_STATUS, app, seq)
    send_tc(pkt)

def rw_set_mode(oid_u32, mode, submode, seq):
    app = be_u32(oid_u32) + bytes([mode & 0xFF, submode & 0xFF])
    pkt = build_tc(APID, SERVICE, SUB_SET_MODE, app, seq)
    send_tc(pkt)

# ---------- CLI ----------
HELP = """\
Commands:
  set <rpm>            - Send SET_SPEED with signed RPM (e.g. set 1000)
  status               - Send STATUS request
  stop                 - Send STOP
  mode <name> [sub]    - Send SET_MODE, names: off,on,normal,raw  (default sub=0)
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
                rw_set_speed(oid, rpm, next(seq))
            except ValueError:
                print("Invalid RPM")

        elif cmd == "status":
            print("Requesting STATUS")
            rw_status(oid, next(seq))

        elif cmd == "stop":
            print("Sending STOP")
            rw_stop(oid, next(seq))

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
            rw_set_mode(oid, mode_val, sub, next(seq))

        else:
            print("Unknown command. Type 'help'.")

if __name__ == "__main__":
    main()
