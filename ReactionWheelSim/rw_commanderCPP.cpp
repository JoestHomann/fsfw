// rw_commander.cpp
// Minimal CLI tool to talk to the Arduino RW over a serial port.
// Mirrors the Python version (set/stop/status/exit).
//
// Build (Linux):
//   g++ -std=c++17 -O2 -Wall -Wextra -o rw_commander rw_commander.cpp
//
// Run:
//   ./rw_commander                 -> will ask for port, uses 9600 baud
//   ./rw_commander /dev/ttyACM0    -> 9600 baud
//   ./rw_commander /dev/ttyACM0 115200
//
// Notes:
// - This uses POSIX termios (Linux/macOS). On Windows, use WSL or adapt to Win32 serial.

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cerrno>
#include <cstring>
#include <cstdint>
#include <chrono>
#include <thread>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

// --- Protocol constants (same as Python) ---
static constexpr uint8_t START_BYTE_CMD   = 0xAA;
static constexpr uint8_t START_BYTE_REPLY = 0xAB;

static constexpr uint8_t CMD_SET_SPEED = 0x01;
static constexpr uint8_t CMD_STOP      = 0x02;
static constexpr uint8_t CMD_STATUS    = 0x03;

static constexpr uint8_t RESP_STATUS   = 0x10;

// ---- Utility: CRC-8 (poly 0x07), initial 0x00, like Python ----
uint8_t crc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) {
      if (crc & 0x80) {
        crc = static_cast<uint8_t>((crc << 1) ^ 0x07);
      } else {
        crc = static_cast<uint8_t>(crc << 1);
      }
    }
  }
  return crc;
}

// ---- Map integer baudrate to termios speed_t ----
speed_t baudToFlag(int baud) {
  switch (baud) {
    case 9600:    return B9600;
    case 19200:   return B19200;
    case 38400:   return B38400;
    case 57600:   return B57600;
    case 115200:  return B115200;
#ifdef B230400
    case 230400:  return B230400;
#endif
#ifdef B460800
    case 460800:  return B460800;
#endif
    default:      return B9600; // fallback
  }
}

// ---- Open and configure the serial port (8N1, raw) ----
int openSerial(const std::string& port, int baud) {
  int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    std::cerr << "ERROR: Cannot open " << port << ": " << std::strerror(errno) << "\n";
    return -1;
  }

  // Clear O_NONBLOCK for blocking I/O after configuration
  int flags = fcntl(fd, F_GETFL, 0);
  fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

  termios tio{};
  if (tcgetattr(fd, &tio) != 0) {
    std::cerr << "ERROR: tcgetattr failed: " << std::strerror(errno) << "\n";
    ::close(fd);
    return -1;
  }

  cfmakeraw(&tio);
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~PARENB;     // no parity
  tio.c_cflag &= ~CSTOPB;     // 1 stop bit
  tio.c_cflag &= ~CRTSCTS;    // no HW flow
  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= CS8;         // 8 data bits

  speed_t spd = baudToFlag(baud);
  cfsetispeed(&tio, spd);
  cfsetospeed(&tio, spd);

  // Non-canonical read settings:
  // VMIN=0 (return immediately if data available), VTIME=10 (1.0s read timeout per read)
  tio.c_cc[VMIN]  = 0;
  tio.c_cc[VTIME] = 10;

  if (tcsetattr(fd, TCSANOW, &tio) != 0) {
    std::cerr << "ERROR: tcsetattr failed: " << std::strerror(errno) << "\n";
    ::close(fd);
    return -1;
  }

  // Flush I/O
  tcflush(fd, TCIOFLUSH);
  return fd;
}

void hexdump(const std::vector<uint8_t>& v, const std::string& prefix) {
  std::cout << prefix;
  std::cout << "[";
  for (size_t i = 0; i < v.size(); ++i) {
    if (i) std::cout << ", ";
    std::cout << "0x" << std::hex << std::uppercase << std::setw(2)
              << std::setfill('0') << static_cast<int>(v[i]) << std::dec;
  }
  std::cout << "]\n";
}

// ---- Send a command (with 16-bit signed value) ----
bool send_command(int fd, uint8_t cmd_id, int value = 0) {
  // Convert to unsigned 16-bit like Python (e.g., -10 => 65526)
  uint16_t uval = static_cast<uint16_t>(value & 0xFFFF);

  std::vector<uint8_t> payload{
      START_BYTE_CMD, cmd_id,
      static_cast<uint8_t>((uval >> 8) & 0xFF),
      static_cast<uint8_t>(uval & 0xFF)
  };
  uint8_t crc = crc8(payload.data(), payload.size());
  payload.push_back(crc);

  hexdump(payload, "-> Sending: ");
  ssize_t n = ::write(fd, payload.data(), payload.size());
  if (n < 0) {
    std::cerr << "ERROR: write failed: " << std::strerror(errno) << "\n";
    return false;
  }
  return true;
}

// ---- Read a single 8-byte reply with 2s overall timeout ----
bool read_reply(int fd) {
  using clock = std::chrono::steady_clock;
  auto deadline = clock::now() + std::chrono::seconds(2);

  std::vector<uint8_t> buf;
  buf.reserve(8);

  // Keep reading until we have at least 8 bytes or timeout
  while (clock::now() < deadline) {
    // check how many bytes are available (non-blocking)
    int bytesAvailable = 0;
    if (ioctl(fd, FIONREAD, &bytesAvailable) == -1) {
      std::cerr << "ERROR: ioctl(FIONREAD): " << std::strerror(errno) << "\n";
      return false;
    }

    if (bytesAvailable > 0) {
      uint8_t tmp[64];
      ssize_t n = ::read(fd, tmp, std::min(bytesAvailable, static_cast<int>(sizeof(tmp))));
      if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          // No data right now
        } else {
          std::cerr << "ERROR: read failed: " << std::strerror(errno) << "\n";
          return false;
        }
      } else if (n > 0) {
        buf.insert(buf.end(), tmp, tmp + n);
        if (buf.size() >= 8) break;
      }
    } else {
      // no data yet -> sleep a bit
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  // Process complete 8-byte frames in the buffer
  while (buf.size() >= 8) {
    std::vector<uint8_t> frame(buf.begin(), buf.begin() + 8);
    hexdump(frame, "<- Received: ");

    // Validate frame
    if (frame[0] != START_BYTE_REPLY) {
      std::cout << "Invalid start byte\n";
      // drop first byte and continue searching
      buf.erase(buf.begin());
      continue;
    }
    if (crc8(frame.data(), 7) != frame[7]) {
      std::cout << "CRC mismatch!\n";
      // drop first byte and continue
      buf.erase(buf.begin());
      continue;
    }

    // Parse by response ID
    if (frame[1] == RESP_STATUS) {
      int16_t speed  = static_cast<int16_t>((frame[2] << 8) | frame[3]);
      int16_t torque = static_cast<int16_t>((frame[4] << 8) | frame[5]);
      bool running   = (frame[6] != 0);

      std::cout << "Status: speed = " << speed
                << " RPM, torque = " << torque
                << " mNm, running = " << std::boolalpha << running << "\n";
      return true;
    } else {
      std::cout << "Unknown response ID: 0x" << std::hex
                << static_cast<int>(frame[1]) << std::dec << "\n";
      return true;
    }
  }

  std::cout << "No full 8-byte reply within timeout.\n";
  return false;
}

int main(int argc, char** argv) {
  std::string port;
  int baud = 9600;

  if (argc >= 2) {
    port = argv[1];
  } else {
    std::cout << "Enter serial port (e.g., /dev/ttyACM0): ";
    std::getline(std::cin, port);
  }
  if (argc >= 3) {
    baud = std::stoi(argv[2]);
  }

  int fd = openSerial(port, baud);
  if (fd < 0) {
    return 1;
  }

  // Allow Arduino to reset after opening the port (common behavior)
  std::this_thread::sleep_for(std::chrono::seconds(2));
  std::cout << "Connected. Commands: set <value>, stop, status, exit\n";

  std::string line;
  while (true) {
    std::cout << ">> ";
    if (!std::getline(std::cin, line)) break;

    // to lower & trim
    auto lower = line;
    for (auto& c : lower) c = static_cast<char>(::tolower(static_cast<unsigned char>(c)));

    if (lower.rfind("set", 0) == 0) {
      std::istringstream iss(lower);
      std::string cmd; int val;
      if (iss >> cmd >> val) {
        send_command(fd, CMD_SET_SPEED, val);
      } else {
        std::cout << "Invalid SET value\n";
      }
    } else if (lower == "stop") {
      send_command(fd, CMD_STOP, 0);
    } else if (lower == "status") {
      send_command(fd, CMD_STATUS, 0);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      read_reply(fd);
    } else if (lower == "exit") {
      break;
    } else if (lower.empty()) {
      continue;
    } else {
      std::cout << "Unknown command.\n";
    }
  }

  ::close(fd);
  return 0;
}
