#include "RwPusService.h"

#include <cstring>
#include <iomanip>

#include "commonObjects.h"
#include "fsfw/action/ActionMessage.h"
#include "fsfw/devicehandlers/DeviceHandlerIF.h"
#include "fsfw/devicehandlers/DeviceHandlerMessage.h"
#include "fsfw/ipc/MessageQueueSenderIF.h"
#include "fsfw/modes/ModeMessage.h"
#include "fsfw/objectmanager/ObjectManager.h"
#include "fsfw/returnvalues/returnvalue.h"
#include "fsfw/serviceinterface/ServiceInterfaceStream.h"
#include "fsfw/storagemanager/StorageManagerIF.h"
#include "fsfw/storagemanager/storeAddress.h"

// ---------- helpers (anonymous namespace) ----------
namespace {

inline uint32_t be32(const uint8_t* p) {
  return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | uint32_t(p[3]);
}

// Same CRC8 as the device / DH
uint8_t crc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int j = 0; j < 8; ++j) {
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07) : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

// Search for an 8-byte frame: AB 10 <spdH spdL> <torH torL> <running> <crc>
int findStatusFrame(const uint8_t* buf, size_t len) {
  if (len < 8) return -1;
  for (size_t i = 0; i + 7 < len; ++i) {
    if (buf[i] == 0xAB && buf[i + 1] == 0x10) {
      if (crc8(buf + i, 7) == buf[i + 7]) return static_cast<int>(i);
    }
  }
  return -1;
}

#if defined(RW_VERBOSE)
static void dumpHexWarn(const char* tag, const uint8_t* p, size_t n) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
  sif::warning << tag << " (" << n << "): ";
  for (size_t i = 0; i < n; ++i) {
    sif::warning << std::hex << std::setw(2) << std::setfill('0') << int(p[i]) << " ";
  }
  sif::warning << std::dec << std::endl;
#else
  (void)tag;
  (void)p;
  (void)n;
#endif
}
#else
static inline void dumpHexWarn(const char*, const uint8_t*, size_t) {}
#endif

inline const char* cmdName(Command_t c) {
  if (c == ActionMessage::DATA_REPLY || c == DeviceHandlerMessage::REPLY_DIRECT_COMMAND_DATA) {
    return "DATA_REPLY/DH_DIRECT_DATA";
  }
  switch (c) {
    case ActionMessage::STEP_SUCCESS:
      return "STEP_SUCCESS";
    case ActionMessage::STEP_FAILED:
      return "STEP_FAILED";
    case ActionMessage::COMPLETION_SUCCESS:
      return "COMPLETION_SUCCESS";
    case ActionMessage::COMPLETION_FAILED:
      return "COMPLETION_FAILED";
    case DeviceHandlerMessage::REPLY_RAW_REPLY:
      return "DH_RAW_REPLY";
    default:
      return "UNKNOWN";
  }
}
}  // namespace

// Minimal per-command state
enum class CmdState : uint32_t { NONE = 0, WAIT_DATA = 1 };

// ---------- class implementation ----------

RwPusService::RwPusService(object_id_t objectId, uint16_t apid, uint8_t serviceId,
                           uint8_t numParallelCommands, uint16_t commandTimeoutSeconds)
    : CommandingServiceBase(objectId, apid, "PUS 220 RW CMD", serviceId, numParallelCommands,
                            commandTimeoutSeconds) {}

ReturnValue_t RwPusService::initialize() {
  auto res = CommandingServiceBase::initialize();
  if (res != returnvalue::OK) return res;

  ipcStore = ObjectManager::instance()->get<StorageManagerIF>(objects::IPC_STORE);
  tmStore = ObjectManager::instance()->get<StorageManagerIF>(objects::TM_STORE);
  tcStore = ObjectManager::instance()->get<StorageManagerIF>(objects::TC_STORE);

#if defined(RW_VERBOSE)
  if (ipcStore == nullptr) {
    sif::warning << "RwPusService: IPC_STORE not available!" << std::endl;
    return returnvalue::FAILED;
  }
  sif::warning << "RwPusService: init ok. myQ=0x" << std::hex << this->getCommandQueue()
               << " ipc=" << ipcStore << " tm=" << tmStore << " tc=" << tcStore << std::dec
               << std::endl;
#endif

  return (ipcStore != nullptr) ? returnvalue::OK : returnvalue::FAILED;
}

ReturnValue_t RwPusService::isValidSubservice(uint8_t subservice) {
  switch (static_cast<Subservice>(subservice)) {
    case Subservice::SET_SPEED:
    case Subservice::STOP:
    case Subservice::STATUS:
    case Subservice::SET_MODE:
      return returnvalue::OK;
    default:
      return AcceptsTelecommandsIF::INVALID_SUBSERVICE;
  }
}

ReturnValue_t RwPusService::getMessageQueueAndObject(uint8_t, const uint8_t* tcData, size_t tcLen,
                                                     MessageQueueId_t* id, object_id_t* objectId) {
  if (tcLen < 4) return CommandingServiceBase::INVALID_TC;

  *objectId = static_cast<object_id_t>(be32(tcData));
#if defined(RW_VERBOSE)
  sif::warning << "PUS220 route: objId=0x" << std::hex << *objectId << std::dec << std::endl;
#endif

  auto* dh = ObjectManager::instance()->get<DeviceHandlerIF>(*objectId);
  if (dh == nullptr) {
#if defined(RW_VERBOSE)
    sif::warning << "PUS220 route: INVALID_OBJECT for objId=0x" << std::hex << *objectId << std::dec
                 << std::endl;
#endif
    return CommandingServiceBase::INVALID_OBJECT;
  }

  *id = dh->getCommandQueue();
#if defined(RW_VERBOSE)
  sif::warning << "PUS220 route: commandQueueId=0x" << std::hex << *id << std::dec << std::endl;
#endif

  lastTargetObjectId_ = *objectId;
  return returnvalue::OK;
}

ReturnValue_t RwPusService::prepareCommand(CommandMessage* message, uint8_t subservice,
                                           const uint8_t* tcData, size_t tcLen, uint32_t* state,
                                           object_id_t) {
#if defined(RW_VERBOSE)
  sif::info << "RwPusService: TC subservice=" << int(subservice) << " len=" << tcLen << std::endl;
#endif
  if (ipcStore == nullptr) return returnvalue::FAILED;
  if (tcLen < 4) return CommandingServiceBase::INVALID_TC;

  const uint8_t* app = tcData + 4;
  const size_t appLen = tcLen - 4;

  if (state) *state = static_cast<uint32_t>(CmdState::NONE);

  switch (static_cast<Subservice>(subservice)) {
    case Subservice::SET_SPEED: {
      if (appLen < 2) return CommandingServiceBase::INVALID_TC;
      const int16_t rpm = static_cast<int16_t>((app[0] << 8) | app[1]);

      store_address_t sid{};
      uint8_t* p = nullptr;
      auto rv = ipcStore->getFreeElement(&sid, sizeof(rpm), &p);
      if (rv != returnvalue::OK) return rv;
      std::memcpy(p, &rpm, sizeof(rpm));

#if defined(RW_VERBOSE)
      sif::warning << "PUS220 prepare SET_SPEED: rpm=" << rpm << " store=0x" << std::hex << sid.raw
                   << std::dec << std::endl;
#endif
      ActionMessage::setCommand(message, 0x01 /* CMD_SET_SPEED */, sid);
      return returnvalue::OK;
    }

    case Subservice::STOP: {
      store_address_t sid{};
      uint8_t* p = nullptr;
      auto rv = ipcStore->getFreeElement(&sid, 1, &p);
      if (rv != returnvalue::OK) return rv;
      *p = 0x00;

#if defined(RW_VERBOSE)
      sif::warning << "PUS220 prepare STOP (with dummy store) sid=0x" << std::hex << sid.raw
                   << std::dec << std::endl;
#endif
      ActionMessage::setCommand(message, 0x02 /* CMD_STOP */, sid);
      return returnvalue::OK;
    }

    case Subservice::STATUS: {
      if (state) *state = static_cast<uint32_t>(CmdState::WAIT_DATA);

      store_address_t sid{};
      uint8_t* p = nullptr;
      auto rv = ipcStore->getFreeElement(&sid, 1, &p);
      if (rv != returnvalue::OK) return rv;
      *p = 0x00;

#if defined(RW_VERBOSE)
      sif::warning << "PUS220 prepare STATUS (with dummy store) sid=0x" << std::hex << sid.raw
                   << std::dec << std::endl;
#endif
      ActionMessage::setCommand(message, 0x03 /* CMD_STATUS */, sid);
      return returnvalue::OK;
    }

    case Subservice::SET_MODE: {
      if (appLen < 2) return CommandingServiceBase::INVALID_TC;
      const uint8_t mode = app[0];
      const uint8_t submode = app[1];
#if defined(RW_VERBOSE)
      sif::info << "RwPusService: SET_MODE req -> mode=" << int(mode) << ", sub=" << int(submode)
                << std::endl;
#endif
      ModeMessage::setModeMessage(message, ModeMessage::CMD_MODE_COMMAND, mode, submode);
      return returnvalue::OK;
    }

    case Subservice::TM_STATUS: {
      return CommandingServiceBase::INVALID_TC;
    }
  }

  return CommandingServiceBase::INVALID_TC;
}

ReturnValue_t RwPusService::handleDataReplyAndEmitTm(store_address_t sid, object_id_t objectId) {
  if (sid.raw == StorageManagerIF::INVALID_ADDRESS) {
#if defined(RW_VERBOSE)
    sif::warning << "[PUS220 generic] invalid store address (sid invalid)" << std::endl;
#endif
    return returnvalue::FAILED;
  }

  StorageManagerIF* stores[3] = {ipcStore, tmStore, tcStore};
  const uint8_t* buf = nullptr;
  size_t len = 0;
  StorageManagerIF* usedStore = nullptr;

  for (StorageManagerIF* s : stores) {
    if (s == nullptr) continue;
    const uint8_t* b = nullptr;
    size_t l = 0;
    auto rv = s->getData(sid, &b, &l);
#if defined(RW_VERBOSE)
    sif::warning << "[PUS220 generic] try "
                 << (s == ipcStore ? "IPC" : (s == tmStore ? "TM" : "TC")) << " rv=" << rv
                 << " len=" << l << std::endl;
#endif
    if (rv == returnvalue::OK && b != nullptr && l > 0) {
      buf = b;
      len = l;
      usedStore = s;
      break;
    }
  }

  if (buf == nullptr || len == 0) {
#if defined(RW_VERBOSE)
    sif::warning << "[PUS220 generic] sid=0x" << std::hex << sid.raw << std::dec
                 << " has no usable payload -> keep waiting" << std::endl;
#endif
    return returnvalue::FAILED;
  }

  dumpHexWarn("[PUS220 generic] head", buf, (len < 32 ? len : size_t(32)));
  const int idx = findStatusFrame(buf, len);
#if defined(RW_VERBOSE)
  sif::warning << "[PUS220 generic] findStatusFrame idx=" << idx << " (len=" << len << ")"
               << std::endl;
#endif

  ReturnValue_t rv = returnvalue::OK;
  if (idx >= 0) {
    const uint8_t* p = buf + idx;
    const int16_t speed = static_cast<int16_t>((p[2] << 8) | p[3]);
    const int16_t torque = static_cast<int16_t>((p[4] << 8) | p[5]);
    const uint8_t running = p[6];

#if defined(RW_VERBOSE)
    if (running > 1) {
      sif::warning << "[PUS220 generic] 'running' flag suspicious: " << int(running)
                   << " (expected 0/1)" << std::endl;
    }
#endif
    // AppData: [object_id(4) | speed(2) | torque(2) | running(1)]
    uint8_t app[9];
    app[0] = static_cast<uint8_t>((objectId >> 24) & 0xFF);
    app[1] = static_cast<uint8_t>((objectId >> 16) & 0xFF);
    app[2] = static_cast<uint8_t>((objectId >> 8) & 0xFF);
    app[3] = static_cast<uint8_t>(objectId & 0xFF);
    app[4] = static_cast<uint8_t>((speed >> 8) & 0xFF);
    app[5] = static_cast<uint8_t>(speed & 0xFF);
    app[6] = static_cast<uint8_t>((torque >> 8) & 0xFF);
    app[7] = static_cast<uint8_t>(torque & 0xFF);
    app[8] = running;

    const auto prv =
        tmHelper.prepareTmPacket(static_cast<uint8_t>(Subservice::TM_STATUS), app, sizeof(app));
#if defined(RW_VERBOSE)
    sif::warning << "[PUS220 generic] prepareTmPacket rv=" << prv << std::endl;
#endif
    rv = (prv == returnvalue::OK) ? tmHelper.storeAndSendTmPacket() : prv;
#if defined(RW_VERBOSE)
    sif::warning << "[PUS220 generic] storeAndSend rv=" << rv << std::endl;
#endif
  } else {
#if defined(RW_VERBOSE)
    sif::warning << "[PUS220 generic] no AB 10 .. .. .. .. .. CRC frame in payload" << std::endl;
#endif
    rv = returnvalue::FAILED;
  }

  if (usedStore != nullptr) {
    auto delRv = usedStore->deleteData(sid);
#if defined(RW_VERBOSE)
    if (delRv != returnvalue::OK) {
      sif::warning << "[PUS220 generic] deleteData sid=0x" << std::hex << sid.raw << std::dec
                   << " failed rv=" << delRv << std::endl;
    }
#endif
  }

  return rv;
}

ReturnValue_t RwPusService::handleReply(const CommandMessage* reply, Command_t, uint32_t* state,
                                        CommandMessage*, object_id_t objectId, bool* isStep) {
#if defined(RW_VERBOSE)
  const store_address_t sidActionDbg = ActionMessage::getStoreId(reply);
  const store_address_t sidDhDbg = DeviceHandlerMessage::getStoreAddress(reply);
  sif::warning << "[PUS220] inspect: sidAction=0x" << std::hex << sidActionDbg.raw << " sidDh=0x"
               << sidDhDbg.raw << std::dec << std::endl;
#endif

  const auto cmd = reply->getCommand();

  CmdState st = CmdState::NONE;
  if (state != nullptr) st = static_cast<CmdState>(*state);

#if defined(RW_VERBOSE)
  sif::info << "RwPusService::handleReply cmd=0x" << std::hex << int(cmd) << " (" << cmdName(cmd)
            << ")" << std::dec << std::endl;
  sif::warning << "[PUS220] current state=" << (st == CmdState::WAIT_DATA ? "WAIT_DATA" : "NONE")
               << std::endl;
#endif

  // 1) Data-like replies: resolve store id and parse
  const bool isDataLike = (cmd == ActionMessage::DATA_REPLY) ||
                          (cmd == DeviceHandlerMessage::REPLY_DIRECT_COMMAND_DATA) ||
                          (cmd == DeviceHandlerMessage::REPLY_RAW_REPLY);

  if (isDataLike) {
    const store_address_t sidAction = ActionMessage::getStoreId(reply);
    const store_address_t sidDh = DeviceHandlerMessage::getStoreAddress(reply);
    const store_address_t sid =
        (sidAction.raw != StorageManagerIF::INVALID_ADDRESS) ? sidAction : sidDh;

#if defined(RW_VERBOSE)
    sif::warning << "[PUS220] data-like reply, sid=0x" << std::hex << sid.raw << std::dec
                 << " -> parsing" << std::endl;
#endif

    if (sid.raw == StorageManagerIF::INVALID_ADDRESS) {
      if (st == CmdState::WAIT_DATA) {
        if (isStep) *isStep = true;  // progress without completion
        return returnvalue::OK;
      }
      return returnvalue::OK;
    }

    const auto rv = handleDataReplyAndEmitTm(sid, objectId);
    if (rv == returnvalue::OK) {
      if (state) *state = static_cast<uint32_t>(CmdState::NONE);
      return CommandingServiceBase::EXECUTION_COMPLETE;
    }
    if (st == CmdState::WAIT_DATA) {
      if (isStep) *isStep = true;
      return returnvalue::OK;
    }
    return returnvalue::OK;
  }

  // 2) Control-only replies: keep the command open while waiting for data
  switch (cmd) {
    case ActionMessage::STEP_SUCCESS:
      if (isStep) *isStep = true;
      if (st == CmdState::WAIT_DATA) return returnvalue::OK;
      return returnvalue::OK;

    case ActionMessage::COMPLETION_SUCCESS:
      if (st == CmdState::WAIT_DATA) return returnvalue::OK;
      return CommandingServiceBase::EXECUTION_COMPLETE;

    case ActionMessage::STEP_FAILED:
      if (isStep) *isStep = true;
      return ActionMessage::getReturnCode(reply);

    case ActionMessage::COMPLETION_FAILED:
      return ActionMessage::getReturnCode(reply);

    default:
#if defined(RW_VERBOSE)
      sif::warning << "[PUS220] unhandled control reply; cmd=0x" << std::hex << int(cmd) << std::dec
                   << std::endl;
#endif
      if (st == CmdState::WAIT_DATA) {
        if (isStep) *isStep = true;
        return returnvalue::OK;
      }
      return returnvalue::OK;
  }
}

void RwPusService::handleUnrequestedReply(CommandMessage* reply) {
  const auto cmd = reply->getCommand();
#if defined(RW_VERBOSE)
  sif::warning << "RwPusService::handleUnrequestedReply cmd=0x" << std::hex << int(cmd) << " ("
               << cmdName(cmd) << ")" << std::dec << std::endl;
#endif

  const store_address_t sidAction = ActionMessage::getStoreId(reply);
  const store_address_t sidDh = DeviceHandlerMessage::getStoreAddress(reply);

  if (sidAction.raw != StorageManagerIF::INVALID_ADDRESS ||
      sidDh.raw != StorageManagerIF::INVALID_ADDRESS) {
    const store_address_t sid =
        (sidAction.raw != StorageManagerIF::INVALID_ADDRESS) ? sidAction : sidDh;
#if defined(RW_VERBOSE)
    sif::warning << "[PUS220] UNREQUESTED: treating message as DATA (sid=0x" << std::hex << sid.raw
                 << std::dec << "), object=0x" << std::hex << lastTargetObjectId_ << std::dec
                 << std::endl;
#endif
    (void)handleDataReplyAndEmitTm(sid, lastTargetObjectId_);
  } else {
#if defined(RW_VERBOSE)
    sif::warning << "[PUS220] UNREQUESTED: no store id in message -> ignore" << std::endl;
#endif
  }
  // Signature is void -> nothing to return
}
