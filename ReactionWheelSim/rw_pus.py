#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket, struct, itertools, threading, math

# --------- Transport to FSFW UDP bridge (hosted example) ----------
UDP_HOST = "127.0.0.1"
UDP_PORT = 7301

# --------- PUS / CCSDS basics ----------
APID    = 0x00EF           # must match RwPusService APID
SRC_ID  = 0x0001
SERVICE = 220              # RwPusService

# Subservices (TC)
SUB_SET_SPEED        = 1
SUB_STOP             = 2
SUB_STATUS           = 3
SUB_SET_TORQUE       = 4
SUB_SET_MODE         = 10
SUB_ACS_SET_ENABLE   = 140
SUB_ACS_SET_TARGET   = 141

# Subservices (TM)
SUB_TM_STATUS_TYPED  = 131   # typed RW status (v1)
SUB_TM_ACS_TYPED     = 132   # typed ACS HK (v1)
SUB_TM_ATT_YPR       = 133   # typed attitude YPR + error angle (v1)

# Housekeeping Service
SVC_HK                  = 3
SUB_HK_REPORT_PERIODIC  = 25   # periodic HK report (Svc 3 / Sub 25)

# DeviceHandler modes (FSFW default enum values)
MODE_OFF    = 0
MODE_ON     = 1
MODE_NORMAL = 2

# Default destination objects
RW_HANDLER_OID = 0x00004402
ACS_CTRL_OID   = 0x4302AC51   # RW_ACS_CTRL

# ---- Receive tuning ----
SOCKET_TIMEOUT_S   = 0.5   # short timeout for RX loop responsiveness
PRINT_UNKNOWN_TM   = False # set True to print every TM

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
    return pkt

# ---------- Helpers ----------
def be_u32(x): return struct.pack(">I", int(x) & 0xFFFFFFFF)
def be_i16(x): return struct.pack(">h", int(x))
def be_f32(x): return struct.pack(">f", float(x))

def make_socket():
    # one connected UDP socket to send and receive TM from the same source port
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect((UDP_HOST, UDP_PORT))   # fix local port and remote addr
    s.settimeout(SOCKET_TIMEOUT_S)
    return s

def send_tc(sock: socket.socket, packet: bytes):
    sock.send(packet)  # connected socket

# ---------- TM decoding ----------
def parse_service_subservice(data: bytes):
    """Very light PUS sniff: service=data[7], subservice=data[8] (works with our bridge)."""
    if len(data) < 10:
        return None, None
    return data[7], data[8]

def handle_tm_typed_rw_131(data: bytes, rw_oid: int):
    """Decode typed RW TM (220/131, v1). AppData layout per your typed format."""
    marker = bytes([1]) + rw_oid.to_bytes(4, "big")
    start = data.find(marker, 9)
    if start == -1 or start + 28 > len(data):
        return False
    try:
        # Layout: ver(1) | oid(4) | speed(i16) | torque(i16) | running(u8)
        #         | flags(u16) | err(u16) | crc_cnt(u32) | mal_cnt(u32) | ts_ms(u32) | sample(u16)
        ver, oid_be, speed, torque, running, flags, err, crc_cnt, mal_cnt, ts_ms, sample = \
            struct.unpack(">B I h h B H H I I I H", data[start:start+28])
    except struct.error:
        return False
    print(
        f"[TM 220/131] v{ver} oid=0x{oid_be:08X}  "
        f"speed={speed} rpm  torque={torque} mNm  running={running}  "
        f"flags=0x{flags:04X} err=0x{err:04X}  "
        f"crcErrCnt={crc_cnt} malformedCnt={mal_cnt} tsMs={ts_ms} sample={sample}"
    )
    return True

def handle_tm_acs_typed_132(data: bytes, acs_oid: int):
    """Decode typed ACS HK (220/132, v1).
       Layout: [OID(4) | ver(1) | enabled(1) | kd[3]*f32 | tauDes[3]*f32 | tauWheelCmd[4]*f32 | dtMs(u32)] = 50 bytes
    """
    marker = acs_oid.to_bytes(4, "big")
    start = data.find(marker, 9)
    if start == -1 or start + 50 > len(data):
        return False
    payload = data[start:start+50]
    oid_be = int.from_bytes(payload[0:4], "big")
    ver    = payload[4]
    en     = payload[5]
    try:
        kd      = struct.unpack(">fff",   payload[6:18])
        tauDes  = struct.unpack(">fff",   payload[18:30])
        tauWh   = struct.unpack(">ffff",  payload[30:46])
        dt_ms   = int.from_bytes(payload[46:50], "big")
    except struct.error:
        return False
    print(f"[TM 220/132] v{ver} oid=0x{oid_be:08X} enabled={en}  "
          f"Kd=({kd[0]:.3f},{kd[1]:.3f},{kd[2]:.3f})  "
          f"tauDes=({tauDes[0]:.3f},{tauDes[1]:.3f},{tauDes[2]:.3f}) mNm  "
          f"tauWheel=({tauWh[0]:.3f},{tauWh[1]:.3f},{tauWh[2]:.3f},{tauWh[3]:.3f}) mNm  "
          f"dt={dt_ms} ms")
    return True

def handle_tm_att_ypr_133(data: bytes, acs_oid: int):
    """Decode typed attitude YPR TM (220/133, v1).
       Expected AppData (big-endian), 40 bytes total:
         ver(1) | OID(4) | enabled(1) |
         yprRef[3]*f32 (deg) | yprTrue[3]*f32 (deg) |
         errAngleDeg f32 | dtMs u32 | sample u16
    """
    marker = bytes([1]) + acs_oid.to_bytes(4, "big")  # ver=1 + OID
    start = data.find(marker, 9)
    if start == -1 or start + 40 > len(data):
        return False

    p = data[start:start+40]
    ver      = p[0]
    oid_be   = int.from_bytes(p[1:5], "big")
    enabled  = p[5]

    # helper for f32 BE
    def f32(i): return struct.unpack(">f", p[i:i+4])[0]
    off = 6
    try:
        ypr_ref  = (f32(off+0),  f32(off+4),  f32(off+8));   off += 12
        ypr_true = (f32(off+0),  f32(off+4),  f32(off+8));   off += 12
        err_deg  = f32(off);                                  off += 4
        dt_ms    = int.from_bytes(p[off:off+4], "big");       off += 4
        sample   = int.from_bytes(p[off:off+2], "big");       off += 2
    except struct.error:
        return False

    print(f"[TM 220/133] v{ver} oid=0x{oid_be:08X} enabled={enabled} "
          f"ref[YPRdeg]=[{ypr_ref[0]:.2f},{ypr_ref[1]:.2f},{ypr_ref[2]:.2f}] "
          f"true[YPRdeg]=[{ypr_true[0]:.2f},{ypr_true[1]:.2f},{ypr_true[2]:.2f}] "
          f"errAngle={err_deg:.2f} deg  dt={dt_ms} ms sample={sample}")
    return True

def handle_tm(data: bytes, rw_oid: int, acs_oid: int):
    """Decode minimal PUS TM set."""
    svc, sub = parse_service_subservice(data)
    if svc is None:
        if PRINT_UNKNOWN_TM:
            print(f"TM: len={len(data)} (short)")
        return

    # Try typed ACS first when it matches
    if svc == SERVICE and sub == SUB_TM_ACS_TYPED:
        if handle_tm_acs_typed_132(data, acs_oid):
            return

    # Then typed RW (status)
    if svc == SERVICE and sub == SUB_TM_STATUS_TYPED:
        if handle_tm_typed_rw_131(data, rw_oid):
            return

    # Typed ATT YPR
    if svc == SERVICE and sub == SUB_TM_ATT_YPR:
        if handle_tm_att_ypr_133(data, acs_oid):
            return

    # Generic print for other packets
    if PRINT_UNKNOWN_TM or svc in (SERVICE, SVC_HK):
        print(f"TM svc={svc} subsvc={sub} len={len(data)}")

# ---------- Background RX thread ----------
def rx_loop(sock: socket.socket, get_ids_callable, stop_event: threading.Event):
    """Continuously receive TM and print decoded info."""
    while not stop_event.is_set():
        try:
            data = sock.recv(4096)
        except socket.timeout:
            continue
        except OSError:
            break
        except Exception as e:
            print(f"(RX error: {e})")
            break

        rw_oid, acs_oid = get_ids_callable()
        handle_tm(data, rw_oid, acs_oid)

# ---------- Command builders ----------
def rw_set_speed(sock, oid_u32, rpm, seq):
    app = be_u32(oid_u32) + be_i16(rpm)
    pkt = build_tc(APID, SERVICE, SUB_SET_SPEED, app, seq)
    send_tc(sock, pkt)

def rw_set_torque(sock, oid_u32, torque_mNm, seq):
    app = be_u32(oid_u32) + be_i16(torque_mNm)
    pkt = build_tc(APID, SERVICE, SUB_SET_TORQUE, app, seq)
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

def acs_set_enable(sock, acs_oid_u32, enable, seq):
    app = be_u32(acs_oid_u32) + bytes([1 if enable else 0])
    pkt = build_tc(APID, SERVICE, SUB_ACS_SET_ENABLE, app, seq)
    send_tc(sock, pkt)

def acs_set_target(sock, acs_oid_u32, q, seq):
    """Send ACS_SET_TARGET with quaternion q=(q0,q1,q2,q3), big-endian float32."""
    q0, q1, q2, q3 = map(float, q)
    n = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
    if n > 1e-9:
        q0, q1, q2, q3 = (q0/n, q1/n, q2/n, q3/n)
    else:
        # fall back to identity if invalid
        q0, q1, q2, q3 = 1.0, 0.0, 0.0, 0.0
    app = be_u32(acs_oid_u32) + be_f32(q0) + be_f32(q1) + be_f32(q2) + be_f32(q3)
    pkt = build_tc(APID, SERVICE, SUB_ACS_SET_TARGET, app, seq)
    send_tc(sock, pkt)

# ---------- CLI ----------
MODE_NAMES = {
    "off": MODE_OFF,
    "on": MODE_ON,
    "normal": MODE_NORMAL,
}

def parse_int(s: str) -> int:
    s = s.strip().lower()
    if s.startswith("0x"):
        return int(s, 16)
    return int(s, 10)

def main():
    print("Interactive PUS-220 sender for RW/ACS. Commands: speed, torque, status, stop, mode, acs_enable, att. Type 'exit' to quit.")
    oids = {"rw": RW_HANDLER_OID, "acs": ACS_CTRL_OID}
    seq = itertools.count()
    sock = make_socket()  # one socket for TX+RX

    # Start background RX thread
    stop_event = threading.Event()
    rx_thread = threading.Thread(
        target=rx_loop,
        args=(sock, lambda: (oids["rw"], oids["acs"]), stop_event),
        daemon=True,
    )
    rx_thread.start()

    try:
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

            elif cmd == "speed":
                if len(parts) < 2:
                    print("usage: speed <rpm>")
                    continue
                try:
                    rpm = int(parts[1])
                    print(f"Sending SET_SPEED {rpm} rpm to RW 0x{oids['rw']:08X}")
                    rw_set_speed(sock, oids['rw'], rpm, next(seq))
                except ValueError:
                    print("Invalid RPM")

            elif cmd == "torque":
                if len(parts) < 2:
                    print("usage: torque <mNm>")
                    continue
                try:
                    tq = int(parts[1])
                    print(f"Sending SET_TORQUE {tq} mNm to RW 0x{oids['rw']:08X}")
                    rw_set_torque(sock, oids['rw'], tq, next(seq))
                except ValueError:
                    print("Invalid torque")

            elif cmd == "status":
                print(f"Requesting STATUS from RW 0x{oids['rw']:08X}")
                rw_status(sock, oids['rw'], next(seq))

            elif cmd == "stop":
                print(f"Sending STOP to RW 0x{oids['rw']:08X}")
                rw_stop(sock, oids['rw'], next(seq))

            elif cmd == "mode":
                if len(parts) < 2:
                    print("usage: mode <off|on|normal> [submode]")
                    continue
                name = parts[1].lower()
                if name not in MODE_NAMES:
                    print("Unknown mode. Use: off, on, normal")
                    continue
                try:
                    sub = parse_int(parts[2]) if len(parts) >= 3 else 0
                except ValueError:
                    print("Invalid submode")
                    continue
                mode_val = MODE_NAMES[name]
                print(f"Sending SET_MODE {name.upper()} (mode={mode_val}, sub={sub}) to RW 0x{oids['rw']:08X}")
                rw_set_mode(sock, oids['rw'], mode_val, sub, next(seq))

            elif cmd == "acs_enable":
                if len(parts) < 2 or parts[1] not in ("0", "1"):
                    print("usage: acs_enable <0|1>")
                    continue
                en = (parts[1] == "1")
                print(f"Sending ACS_SET_ENABLE {int(en)} to ACS 0x{oids['acs']:08X}")
                acs_set_enable(sock, oids['acs'], en, next(seq))

            elif cmd in ("att", "acs_att", "attitude"):
                # att q0 q1 q2 q3
                if len(parts) != 5:
                    print("usage: att <q0> <q1> <q2> <q3>")
                    continue
                try:
                    q = tuple(float(x) for x in parts[1:5])
                except ValueError:
                    print("Invalid quaternion component(s)")
                    continue
                n = math.sqrt(sum(v*v for v in q))
                qn = tuple((v / n) if n > 1e-9 else (1.0 if i == 0 else 0.0) for i, v in enumerate(q))
                print(f"Sending ACS_SET_TARGET q={qn} to ACS 0x{oids['acs']:08X}")
                acs_set_target(sock, oids['acs'], qn, next(seq))

            else:
                print("Unknown command.")

    finally:
        stop_event.set()
        try:
            sock.close()
        except Exception:
            pass
        rx_thread.join(timeout=1.0)

if __name__ == "__main__":
    main()
