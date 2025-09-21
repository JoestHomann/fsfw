import serial
import time

START_BYTE_CMD   = 0xAA
START_BYTE_REPLY = 0xAB

CMD_SET_SPEED = 0x01
CMD_STOP      = 0x02
CMD_STATUS    = 0x03

RESP_STATUS   = 0x10

# --- CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, xorout 0x0000) ---
def crc16_ccitt(data: bytes, init: int = 0xFFFF) -> int:
    """Bitwise CRC-16/CCITT-FALSE; good enough for short frames."""
    crc = init
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

def send_command(ser: serial.Serial, cmd_id: int, value: int = 0) -> None:
    """Build and send 6B command: AA <cmd> <valH> <valL> <crcH> <crcL>."""
    # Map negative 16-bit values to unsigned representation (two's complement)
    if value < 0:
        value &= 0xFFFF

    payload = bytes([START_BYTE_CMD, cmd_id, (value >> 8) & 0xFF, value & 0xFF])
    c = crc16_ccitt(payload)
    packet = payload + bytes([(c >> 8) & 0xFF, c & 0xFF])

    print(f"-> Sending: {[hex(b) for b in packet]}")
    ser.write(packet)

def read_reply(ser: serial.Serial, timeout_s: float = 2.0) -> None:
    """
    Read and parse 9B STATUS frame:
      AB 10 <spdH> <spdL> <torH> <torL> <running> <crcH> <crcL>
    Scans the incoming stream for a valid frame (no strict alignment required).
    """
    deadline = time.time() + timeout_s
    buf = bytearray()

    # Keep filling a small buffer and scan for a 9B frame
    while time.time() < deadline:
        chunk = ser.read(64)  # non-blocking up to configured serial timeout
        if chunk:
            buf.extend(chunk)
            # Scan window (only need last ~64 bytes)
            if len(buf) > 256:
                del buf[:-64]

            # Try to find a valid 9B frame
            i = 0
            while i + 9 <= len(buf):
                if buf[i] == START_BYTE_REPLY and buf[i + 1] == RESP_STATUS:
                    frame = bytes(buf[i:i + 9])
                    # CRC over first 7 bytes must match trailing 2 (big-endian)
                    rx = (frame[7] << 8) | frame[8]
                    calc = crc16_ccitt(frame[:7])
                    if rx != calc:
                        # CRC mismatch: skip this start byte and continue scanning
                        i += 1
                        continue

                    # Parse big-endian signed fields
                    speed = (frame[2] << 8) | frame[3]
                    if speed & 0x8000:
                        speed -= 0x10000
                    torque = (frame[4] << 8) | frame[5]
                    if torque & 0x8000:
                        torque -= 0x10000
                    running = frame[6] != 0

                    print(f"<- Received: {[hex(b) for b in frame]}")
                    print(f"Status: speed = {speed} RPM, torque = {torque} mNm, running = {running}")
                    return
                else:
                    i += 1
        else:
            time.sleep(0.01)

    print("No valid STATUS frame received within timeout.")

def main():
    port = input("Enter serial port (e.g., COM4 for Windows or /dev/ttyACM0 for Linux): ").strip()
    baud = 9600
    ser = serial.Serial(port, baud, timeout=0.05)  # small timeout helps the scanner
    time.sleep(2)  # allow Arduino to reset

    print("Connected. Commands: set <value>, stop, status, exit")
    while True:
        try:
            cmd = input(">> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            break

        if cmd.startswith("set"):
            parts = cmd.split()
            if len(parts) != 2:
                print("Usage: set <rpm>")
                continue
            try:
                val = int(parts[1])
            except ValueError:
                print("Invalid SET value")
                continue
            send_command(ser, CMD_SET_SPEED, val)

        elif cmd == "stop":
            send_command(ser, CMD_STOP, 0)

        elif cmd == "status":
            send_command(ser, CMD_STATUS, 0)
            # Give device a brief moment to respond
            time.sleep(0.05)
            read_reply(ser)

        elif cmd == "exit":
            break

        else:
            print("Unknown command. Use: set <rpm> | stop | status | exit")

    ser.close()

if __name__ == "__main__":
    main()
