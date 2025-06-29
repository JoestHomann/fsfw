import serial
import time

START_BYTE_CMD = 0xAA
START_BYTE_REPLY = 0xAB

CMD_SET_SPEED = 0x01
CMD_STOP = 0x02
CMD_STATUS = 0x03

RESP_STATUS = 0x10

def crc8(data):
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x07
            else:
                crc <<= 1
            crc &= 0xFF
    return crc

def send_command(ser, cmd_id, value=0):
    payload = [START_BYTE_CMD, cmd_id, (value >> 8) & 0xFF, value & 0xFF]
    crc = crc8(payload)
    packet = bytearray(payload + [crc])
    print(f"→ Sending: {[hex(b) for b in packet]}")
    ser.write(packet)

def read_reply(ser):
    timeout = time.time() + 2.0
    while ser.in_waiting < 8 and time.time() < timeout:
        time.sleep(0.01)

    while ser.in_waiting >= 8:
        reply = list(ser.read(8))
        print(f"← Received: {[hex(b) for b in reply]}")
        if reply[0] != START_BYTE_REPLY:
            print("Invalid start byte")
            continue
        if crc8(reply[:7]) != reply[7]:
            print("CRC mismatch!")
            continue
        if reply[1] == RESP_STATUS:
            speed = (reply[2] << 8) | reply[3]
            torque = (reply[4] << 8) | reply[5]
            if torque & 0x8000:
                torque -= 0x10000
            running = reply[6]
            print(f"Status: speed = {speed} RPM, torque = {torque} mNm, running = {bool(running)}")
            break
        else:
            print("Unknown response ID")


def main():
    port = input("Enter serial port (e.g., COM4 or /dev/ttyUSB0): ")
    baud = 9600
    ser = serial.Serial(port, baud, timeout=1)
    time.sleep(2)  # Allow Arduino to reset

    print("Connected. Commands: set <value>, stop, status, exit")
    while True:
        cmd = input(">> ").strip().lower()
        if cmd.startswith("set"):
            try:
                val = int(cmd.split()[1])
                send_command(ser, CMD_SET_SPEED, val)
            except:
                print("Invalid SET value")
        elif cmd == "stop":
            send_command(ser, CMD_STOP, 0)
        elif cmd == "status":
            send_command(ser, CMD_STATUS, 0)
            time.sleep(0.1)
            read_reply(ser)
        elif cmd == "exit":
            break
        else:
            print("Unknown command.")

    ser.close()

if __name__ == "__main__":
    main()
