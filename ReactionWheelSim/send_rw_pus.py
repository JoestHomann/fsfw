import socket, struct, itertools, time, binascii

UDP_HOST = "127.0.0.1"     # in WSL senden!
UDP_PORT = 7301

APID = 0x00EF              # MUSS zu deinem Board passen (RW_PUS_APID)
SRC_ID = 0x0001
SERVICE = 220              # RwPusService
SUB_SET_SPEED = 1
SUB_STOP      = 2
SUB_STATUS    = 3

# ---------- CRC16-CCITT (FALSE): poly 0x1021, init 0xFFFF, no final XOR, no reflection ----------
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

# ---------- CCSDS + PUS-A (V1) ----------
def ccsds_tc_primary_header(apid, seq_count, bytes_after_primary):
    # Version=0, Type=1 (TC), SecHdrFlag=1 -> 0x1800
    first  = 0x1800 | (apid & 0x07FF)
    second = 0xC000 | (seq_count & 0x3FFF)      # '11' + seq
    pkt_len = (bytes_after_primary - 1) & 0xFFFF
    return struct.pack(">HHH", first, second, pkt_len)
    
# PUS version/ack:
# PUS-A wäre 0x1F, FSFW erwartet meist PUS-C -> 0x2F
VER_ACK = 0x2F

def pus_tc_sec_hdr(service, subservice, src_id):
    # PUS-C: version/ack in first byte
    return struct.pack(">BBBH", VER_ACK, service & 0xFF, subservice & 0xFF, src_id & 0xFFFF)

def build_tc(apid, service, subservice, app_data: bytes, seq_count):
    sec = pus_tc_sec_hdr(service, subservice, SRC_ID) + app_data
    after_ph_len = len(sec) + 2
    ph = ccsds_tc_primary_header(apid, seq_count, after_ph_len)
    whole_wo_crc = ph + sec
    crc = crc16_ccitt_false(whole_wo_crc)
    sec_crc = sec + struct.pack(">H", crc)
    pkt = ph + sec_crc

    print(f"PUS len(after PH)={len(sec_crc)}  CRC=0x{crc:04X}")
    print("PUS field hex:", binascii.hexlify(sec_crc).decode())
    return pkt


def be_u32(x): return struct.pack(">I", x)
def be_i16(x): return struct.pack(">h", x)

def send_tc(packet: bytes):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.sendto(packet, (UDP_HOST, UDP_PORT))

def rw_set_speed(object_id_u32, rpm, seq):
    app = be_u32(object_id_u32) + be_i16(rpm)
    pkt = build_tc(APID, SERVICE, SUB_SET_SPEED, app, seq)
    send_tc(pkt)

def rw_stop(object_id_u32, seq):
    app = be_u32(object_id_u32)
    pkt = build_tc(APID, SERVICE, SUB_STOP, app, seq)
    send_tc(pkt)

def rw_status(object_id_u32, seq):
    app = be_u32(object_id_u32)
    pkt = build_tc(APID, SERVICE, SUB_STATUS, app, seq)
    send_tc(pkt)

if __name__ == "__main__":
    RW_CMD_HANDLER_OID = 0x00004402  # objects::RW_CMD_HANDLER (Big Endian in AppData)

    seq = itertools.count()
    print("Sending SET_SPEED 1000 rpm")
    rw_set_speed(RW_CMD_HANDLER_OID, 1000, next(seq)); time.sleep(15)

    print("Requesting STATUS")
    rw_status(RW_CMD_HANDLER_OID, next(seq)); time.sleep(15)

    print("Sending STOP")
    rw_stop(RW_CMD_HANDLER_OID, next(seq))
