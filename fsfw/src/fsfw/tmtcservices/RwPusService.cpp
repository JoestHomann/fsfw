// fsfw/tmtcservices/RwPusService.cpp
#include "RwPusService.h"

#include <cstring>
#include <iomanip>

#include "fsfw/devicehandlers/RwProtocol.h"  // CRC-16, IDs, STATUS_LEN
#include "example_common/mission/acs/AcsController.h"
#include "commonObjects.h"
#include "fsfw/action/ActionMessage.h"
#include "fsfw/devicehandlers/DeviceHandlerIF.h"
#include "fsfw/devicehandlers/DeviceHandlerMessage.h"
#include "fsfw/ipc/MessageQueueIF.h"
#include "fsfw/ipc/MessageQueueSenderIF.h"
#include "fsfw/modes/ModeMessage.h"
#include "fsfw/objectmanager/ObjectManager.h"
#include "fsfw/returnvalues/returnvalue.h"
#include "fsfw/serviceinterface/ServiceInterfaceStream.h"
#include "fsfw/storagemanager/StorageManagerIF.h"
#include "fsfw/storagemanager/storeAddress.h"

namespace {

inline uint32_t be32(const uint8_t* p) {
  return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | uint32_t(p[3]);
}

// Find a STATUS frame with CRC-16/CCITT (RwProtocol::STATUS_LEN bytes).
// Layout: [AB, 10, spdH, spdL, torH, torL, running, crcH, crcL]
int findStatusFrameCrc16(const uint8_t* buf, size_t len) {
  if (len < RwProtocol::STATUS_LEN) return -1;
  for (size_t i = 0; i + (RwProtocol::STATUS_LEN - 1) < len; ++i) {
    if (buf[i] == RwProtocol::START_REPLY &&
        buf[i + 1] == static_cast<uint8_t>(RwProtocol::RespId::STATUS)) {
      if (RwProtocol::verifyCrc16(buf + i, RwProtocol::STATUS_LEN)) {
        return static_cast<int>(i);
      }
    }
  }
  return -1;
}

static inline void be_store_u32(uint8_t* p, uint32_t v) {
  p[0] = static_cast<uint8_t>((v >> 24) & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 16) & 0xFF);
  p[2] = static_cast<uint8_t>((v >> 8) & 0xFF);
  p[3] = static_cast<uint8_t>( v       & 0xFF);
}
static inline void be_store_f32(uint8_t* p, float f) {
  static_assert(sizeof(float) == 4, "float must be 4 bytes");
  uint32_t u;
  std::memcpy(&u, &f, 4);
  be_store_u32(p, u);
}

#if RW_PUS_VERBOSE
static void dumpHexWarn(const char* tag, const uint8_t* p, size_t n) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
  sif::warning << tag << " (" << n << "): ";
  for (size_t i = 0; i < n; ++i) {
    sif::warning << std::hex << std::setw(2) << std::setfill('0') << int(p[i]) << " ";
  }
  sif::warning << std::dec << std::endl;
#else
  (void)tag; (void)p; (void)n;
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
    case ActionMessage::STEP_SUCCESS: return "STEP_SUCCESS";
    case ActionMessage::STEP_FAILED: return "STEP_FAILED";
    case ActionMessage::COMPLETION_SUCCESS: return "COMPLETION_SUCCESS";
    case ActionMessage::COMPLETION_FAILED: return "COMPLETION_FAILED";
    case DeviceHandlerMessage::REPLY_RAW_REPLY: return "DH_RAW_REPLY";
    default: return "UNKNOWN";
  }
}
}  // namespace

// Minimal per-command state
enum class CmdState : uint32_t { NONE = 0, WAIT_DATA = 1 };

RwPusService::RwPusService(object_id_t objectId, uint16_t apid, uint8_t serviceId,
                           uint8_t numParallelCommands, uint16_t commandTimeoutSeconds)
    : CommandingServiceBase(objectId, apid, "PUS 220 RW CMD", serviceId, numParallelCommands,
                            commandTimeoutSeconds) {}

ReturnValue_t RwPusService::initialize() {
  auto res = CommandingServiceBase::initialize();
  if (res != returnvalue::OK) return res;

  ipcStore = ObjectManager::instance()->get<StorageManagerIF>(objects::IPC_STORE);
  tmStore  = ObjectManager::instance()->get<StorageManagerIF>(objects::TM_STORE);
  tcStore  = ObjectManager::instance()->get<StorageManagerIF>(objects::TC_STORE);

#if RW_PUS_VERBOSE
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
    case Subservice::SET_TORQUE:    // NEW
    case Subservice::SET_MODE:
    case Subservice::ACS_SET_ENABLE: // NEW
      return returnvalue::OK;
    default:
      return AcceptsTelecommandsIF::INVALID_SUBSERVICE;
  }
}

ReturnValue_t RwPusService::getMessageQueueAndObject(uint8_t subservice, const uint8_t* tcData,
                                                     size_t tcLen, MessageQueueId_t* id,
                                                     object_id_t* objectId) {
  if (tcLen < 4) return CommandingServiceBase::INVALID_TC;

  // First 4 bytes in AppData are always a target OID (RW handler or ACS controller)
  *objectId = static_cast<object_id_t>(be32(tcData));

  // Default: resolve as DeviceHandler (RW)
  auto* dh = ObjectManager::instance()->get<DeviceHandlerIF>(*objectId);

  if (static_cast<Subservice>(subservice) == Subservice::ACS_SET_ENABLE) {
    // Local operation in service: no device queue involved
    *id = MessageQueueIF::NO_QUEUE;
    lastTargetObjectId_ = *objectId;
    return returnvalue::OK;
  }

  if (dh == nullptr) {
#if RW_PUS_VERBOSE
    sif::warning << "PUS220 route: INVALID_OBJECT for objId=0x" << std::hex << *objectId << std::dec
                 << std::endl;
#endif
    return CommandingServiceBase::INVALID_OBJECT;
  }

  *id = dh->getCommandQueue();

  // Map sender queue -> object id for robust unrequested reply routing
  qidToObj_[*id] = *objectId;

#if RW_PUS_VERBOSE
  sif::warning << "PUS220 route: commandQueueId=0x" << std::hex << *id << std::dec << std::endl;
#endif

  lastTargetObjectId_ = *objectId;  // fallback
  return returnvalue::OK;
}

ReturnValue_t RwPusService::prepareCommand(CommandMessage* message, uint8_t subservice,
                                           const uint8_t* tcData, size_t tcLen, uint32_t* state,
                                           object_id_t objectId) {
#if RW_PUS_VERBOSE
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
      ActionMessage::setCommand(message, 0x01 /* CMD_SET_SPEED */, sid);
      return returnvalue::OK;
    }

    case Subservice::SET_TORQUE: {  // NEW
      if (appLen < 2) return CommandingServiceBase::INVALID_TC;
      const int16_t tq = static_cast<int16_t>((app[0] << 8) | app[1]);
      store_address_t sid{};
      uint8_t* p = nullptr;
      auto rv = ipcStore->getFreeElement(&sid, sizeof(tq), &p);
      if (rv != returnvalue::OK) return rv;
      std::memcpy(p, &tq, sizeof(tq));
      ActionMessage::setCommand(message, 0x04 /* CMD_SET_TORQUE */, sid);
      return returnvalue::OK;
    }

    case Subservice::STOP: {
      store_address_t sid{};
      uint8_t* p = nullptr;
      auto rv = ipcStore->getFreeElement(&sid, 1, &p);
      if (rv != returnvalue::OK) return rv;
      *p = 0x00;
      ActionMessage::setCommand(message, 0x02 /* CMD_STOP */, sid);
      return returnvalue::OK;
    }

    case Subservice::STATUS: {
      // Expect a DATA_REPLY later -> WAIT_DATA
      if (state) *state = static_cast<uint32_t>(CmdState::WAIT_DATA);
      store_address_t sid{};
      uint8_t* p = nullptr;
      auto rv = ipcStore->getFreeElement(&sid, 1, &p);
      if (rv != returnvalue::OK) return rv;
      *p = 0x00;
      ActionMessage::setCommand(message, 0x03 /* CMD_STATUS */, sid);
      return returnvalue::OK;
    }

    case Subservice::SET_MODE: {
      if (appLen < 2) return CommandingServiceBase::INVALID_TC;
      const uint8_t mode = app[0];
      const uint8_t submode = app[1];
      ModeMessage::setModeMessage(message, ModeMessage::CMD_MODE_COMMAND, mode, submode);
      return returnvalue::OK;
    }

    case Subservice::ACS_SET_ENABLE: {  // local handling (no message sent)
      if (appLen < 1) return CommandingServiceBase::INVALID_TC;
      const bool enable = app[0] != 0;
      auto* acs = ObjectManager::instance()->get<AcsController>(objectId);
      if (acs == nullptr) {
#if RW_PUS_VERBOSE
        sif::warning << "RwPusService: ACS OID 0x" << std::hex << objectId << std::dec
                     << " not found" << std::endl;
#endif
        return CommandingServiceBase::INVALID_OBJECT;
      }
      acs->setEnabled(enable);
      // Immediately emit typed ACS HK for operator feedback
      (void)emitAcsTypedHk(objectId);
      // No queue message -> treat as complete
      return CommandingServiceBase::EXECUTION_COMPLETE;
    }

    default:
      return CommandingServiceBase::INVALID_TC;
  }
}

ReturnValue_t RwPusService::handleDataReplyAndEmitTm(store_address_t sid, object_id_t objectId) {
  if (sid.raw == StorageManagerIF::INVALID_ADDRESS) {
#if RW_PUS_VERBOSE
    sif::warning << "[PUS220 generic] invalid store address (sid invalid)" << std::endl;
#endif
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
#if RW_PUS_VERBOSE
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
#if RW_PUS_VERBOSE
    sif::warning << "[PUS220 generic] sid=0x" << std::hex << sid.raw << std::dec
                 << " has no usable payload -> keep waiting" << std::endl;
#endif
    return returnvalue::FAILED;
  }

  dumpHexWarn("[PUS220 generic] head", buf, (len < 32 ? len : size_t(32)));
  const int idx = findStatusFrameCrc16(buf, len);

  ReturnValue_t rv = returnvalue::OK;
  if (idx >= 0) {
    RwProtocol::Status st{};
    const bool ok = RwProtocol::parseStatus(buf + idx, RwProtocol::STATUS_LEN, st);
    if (!ok) {
      rv = returnvalue::FAILED;
    } else {
      // AppData: [object_id(4) | speed(2) | torque(2) | running(1)] => 9 bytes
      uint8_t app[9];
      app[0] = static_cast<uint8_t>((objectId >> 24) & 0xFF);
      app[1] = static_cast<uint8_t>((objectId >> 16) & 0xFF);
      app[2] = static_cast<uint8_t>((objectId >> 8) & 0xFF);
      app[3] = static_cast<uint8_t>(objectId & 0xFF);
      app[4] = static_cast<uint8_t>((st.speedRpm >> 8) & 0xFF);
      app[5] = static_cast<uint8_t>( st.speedRpm       & 0xFF);
      app[6] = static_cast<uint8_t>((st.torqueMnM >> 8) & 0xFF);
      app[7] = static_cast<uint8_t>( st.torqueMnM       & 0xFF);
      app[8] = st.running;

      const auto prv =
          tmHelper.prepareTmPacket(static_cast<uint8_t>(Subservice::TM_STATUS), app, sizeof(app));
      rv = (prv == returnvalue::OK) ? tmHelper.storeAndSendTmPacket() : prv;
    }
  } else {
    rv = returnvalue::FAILED;
  }

  if (usedStore != nullptr) {
    (void)usedStore->deleteData(sid);
  }

  return rv;
}

ReturnValue_t RwPusService::handleReply(const CommandMessage* reply, Command_t, uint32_t* state,
                                        CommandMessage*, object_id_t objectId, bool* isStep) {
#if RW_PUS_VERBOSE
  sif::info << "RwPusService::handleReply cmd=0x" << std::hex << int(reply->getCommand())
            << " (" << cmdName(reply->getCommand()) << ")" << std::dec << std::endl;
#endif

  const auto cmd = reply->getCommand();
  CmdState st = CmdState::NONE;
  if (state != nullptr) st = static_cast<CmdState>(*state);

  // Data-like replies: resolve store id and parse
  const bool isDataLike = (cmd == ActionMessage::DATA_REPLY) ||
                          (cmd == DeviceHandlerMessage::REPLY_DIRECT_COMMAND_DATA) ||
                          (cmd == DeviceHandlerMessage::REPLY_RAW_REPLY);

  if (isDataLike) {
    const store_address_t sidAction = ActionMessage::getStoreId(reply);
    const store_address_t sidDh = DeviceHandlerMessage::getStoreAddress(reply);
    const store_address_t sid =
        (sidAction.raw != StorageManagerIF::INVALID_ADDRESS) ? sidAction : sidDh;

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

  // Control-only replies
  switch (cmd) {
    case ActionMessage::STEP_SUCCESS: {
      if (st != CmdState::WAIT_DATA) {
        return CommandingServiceBase::EXECUTION_COMPLETE;
      }
      if (isStep) *isStep = true;
      return returnvalue::OK;
    }
    case ActionMessage::COMPLETION_SUCCESS: {
      if (st == CmdState::WAIT_DATA) return returnvalue::OK;
      return CommandingServiceBase::EXECUTION_COMPLETE;
    }
    case ActionMessage::STEP_FAILED: {
      if (isStep) *isStep = true;
      return ActionMessage::getReturnCode(reply);
    }
    case ActionMessage::COMPLETION_FAILED: {
      return ActionMessage::getReturnCode(reply);
    }
    default: {
      if (st == CmdState::WAIT_DATA) {
        if (isStep) *isStep = true;
        return returnvalue::OK;
      }
      return returnvalue::OK;
    }
  }
}

void RwPusService::handleUnrequestedReply(CommandMessage* reply) {
  const auto cmd = reply->getCommand();
#if RW_PUS_VERBOSE
  sif::warning << "RwPusService::handleUnrequestedReply cmd=0x" << std::hex << int(cmd) << " ("
               << cmdName(cmd) << ")" << std::dec << std::endl;
#endif

  const store_address_t sidAction = ActionMessage::getStoreId(reply);
  const store_address_t sidDh = DeviceHandlerMessage::getStoreAddress(reply);

  if (sidAction.raw != StorageManagerIF::INVALID_ADDRESS ||
      sidDh.raw != StorageManagerIF::INVALID_ADDRESS) {
    const store_address_t sid =
        (sidAction.raw != StorageManagerIF::INVALID_ADDRESS) ? sidAction : sidDh;

    // Map sender queue -> object id (robust when multiple RWs exist)
    object_id_t obj = lastTargetObjectId_;
    const MessageQueueId_t sender = reply->getSender();
    auto it = qidToObj_.find(sender);
    if (it != qidToObj_.end()) {
      obj = it->second;
    }

#if RW_PUS_VERBOSE
    sif::warning << "[PUS220] UNREQUESTED: treating message as DATA (sid=0x" << std::hex << sid.raw
                 << std::dec << "), object=0x" << std::hex << obj << std::dec
                 << " senderQ=0x" << std::hex << sender << std::dec << std::endl;
#endif
    (void)handleDataReplyAndEmitTm(sid, obj);
  }
}

ReturnValue_t RwPusService::emitAcsTypedHk(object_id_t acsObjectId) {
  auto* acs = ObjectManager::instance()->get<AcsController>(acsObjectId);
  if (acs == nullptr) {
    return CommandingServiceBase::INVALID_OBJECT;
  }

  AcsDiagSnapshot snap{};
  acs->fillDiagSnapshot(snap);

  // AppData layout (big-endian):
  // [OID(4) | ver(1) | enabled(1) | kd[3]*f32 | tauDes[3]*f32 | tauWheel[4]*f32 | dt_ms(u32)]
  uint8_t app[50] = {};
  size_t off = 0;
  be_store_u32(&app[off], static_cast<uint32_t>(acsObjectId)); off += 4;
  app[off++] = snap.version;
  app[off++] = snap.enabled ? 1 : 0;
  for (int i = 0; i < 3; ++i) { be_store_f32(&app[off], snap.kd[i]); off += 4; }
  for (int i = 0; i < 3; ++i) { be_store_f32(&app[off], snap.tauDes[i]); off += 4; }
  for (int i = 0; i < 4; ++i) { be_store_f32(&app[off], snap.tauWheel[i]); off += 4; }
  be_store_u32(&app[off], snap.dt_ms); off += 4;

  const auto prv = tmHelper.prepareTmPacket(static_cast<uint8_t>(Subservice::TM_ACS_HK_TYPED),
                                            app, off);
  if (prv != returnvalue::OK) return prv;
  return tmHelper.storeAndSendTmPacket();
}
