#include "RwPusService.h"

#include <cmath>
#include <cstring>
#include <iomanip>

#include "fsfw/devicehandlers/RwProtocol.h"
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
#include "fsfw/timemanager/Clock.h"

namespace {

// Read big-endian 32-bit unsigned
inline uint32_t be32(const uint8_t* p) {
  return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | uint32_t(p[3]);
}

// Read big-endian float32
inline float be_f32(const uint8_t* p) {
  const uint32_t u = be32(p);
  float f;
  std::memcpy(&f, &u, sizeof(float));
  return f;
}

// Scan buffer for a valid STATUS frame with CRC-16 (length = RwProtocol::STATUS_LEN)
// Returns byte index or -1 if not found
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

// Store big-endian 32-bit unsigned
static inline void be_store_u32(uint8_t* p, uint32_t v) {
  p[0] = static_cast<uint8_t>((v >> 24) & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 16) & 0xFF);
  p[2] = static_cast<uint8_t>((v >> 8) & 0xFF);
  p[3] = static_cast<uint8_t>( v       & 0xFF);
}

// Store big-endian 16-bit unsigned
static inline void be_store_u16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>((v >> 8) & 0xFF);
  p[1] = static_cast<uint8_t>( v       & 0xFF);
}

// Store big-endian float32
static inline void be_store_f32(uint8_t* p, float f) {
  static_assert(sizeof(float) == 4, "float must be 4 bytes");
  uint32_t u;
  std::memcpy(&u, &f, 4);
  be_store_u32(p, u);
}

#if RW_PUS_VERBOSE
// Hex dump helper guarded by RW_PUS_VERBOSE
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

// Human-readable name for command codes in replies (for logs)
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

// Minimal per-command state for multi-step operations
enum class CmdState : uint32_t { NONE = 0, WAIT_DATA = 1 };

// Constructor: basic PUS-220 service with limits for parallel TCs and timeout
RwPusService::RwPusService(object_id_t objectId, uint16_t apid, uint8_t serviceId,
                           uint8_t numParallelCommands, uint16_t commandTimeoutSeconds)
    : CommandingServiceBase(objectId, apid, "PUS 220 RW CMD", serviceId, numParallelCommands,
                            commandTimeoutSeconds) {}

// Resolve storage managers and finish base init
ReturnValue_t RwPusService::initialize() {
  auto res = CommandingServiceBase::initialize();
  if (res != returnvalue::OK) return res;

  ipcStore = ObjectManager::instance()->get<StorageManagerIF>(objects::IPC_STORE);
  tmStore  = ObjectManager::instance()->get<StorageManagerIF>(objects::TM_STORE);
  tcStore  = ObjectManager::instance()->get<StorageManagerIF>(objects::TC_STORE);

#if RW_PUS_VERBOSE
  sif::warning << "RwPusService::initialize: Init ok. myQ=0x" << std::hex << this->getCommandQueue()
               << " ipc=" << ipcStore << " tm=" << tmStore << " tc=" << tcStore << std::dec
               << std::endl;
#endif

  return (ipcStore != nullptr) ? returnvalue::OK : returnvalue::FAILED;
}

// Filter supported subservices (TCs). Unknown subservices are rejected.
ReturnValue_t RwPusService::isValidSubservice(uint8_t subservice) {
  switch (static_cast<Subservice>(subservice)) {
    case Subservice::SET_SPEED:
    case Subservice::STOP:
    case Subservice::STATUS:
    case Subservice::SET_TORQUE:
    case Subservice::SET_MODE:
    case Subservice::ACS_SET_ENABLE:
    case Subservice::ACS_SET_TARGET:  // local only
      return returnvalue::OK;
    default:
      return AcceptsTelecommandsIF::INVALID_SUBSERVICE;
  }
}

// Resolve target queue and object for a TC. The first 4 bytes are always the target OID.
// ACS local operations return NO_QUEUE to keep handling inside the service.
ReturnValue_t RwPusService::getMessageQueueAndObject(uint8_t subservice, const uint8_t* tcData,
                                                     size_t tcLen, MessageQueueId_t* id,
                                                     object_id_t* objectId) {
  if (tcLen < 4) return CommandingServiceBase::INVALID_TC;

  // First 4 bytes in AppData are target object id (RW handler or ACS controller)
  *objectId = static_cast<object_id_t>(be32(tcData));

  // Local-only subservices (no device queue interaction)
  if (static_cast<Subservice>(subservice) == Subservice::ACS_SET_ENABLE ||
      static_cast<Subservice>(subservice) == Subservice::ACS_SET_TARGET) {
    *id = MessageQueueIF::NO_QUEUE;
    lastTargetObjectId_ = *objectId;
    return returnvalue::OK;
  }

  // Default path: forward to a DeviceHandler (a RW instance)
  auto* dh = ObjectManager::instance()->get<DeviceHandlerIF>(*objectId);
  if (dh == nullptr) {
#if RW_PUS_VERBOSE
    sif::warning << "RwPusService::getMessageQueueAndObject: INVALID_OBJECT for objId=0x" << std::hex << *objectId << std::dec
                 << std::endl;
#endif
    return CommandingServiceBase::INVALID_OBJECT;
  }

  *id = dh->getCommandQueue();

  // Map sender queue -> object id (used for unrequested replies)
  qidToObj_[*id] = *objectId;

#if RW_PUS_VERBOSE
  sif::warning << "RwPusService::getMessageQueueAndObject: commandQueueId=0x" << std::hex << *id << std::dec << std::endl;
#endif

  lastTargetObjectId_ = *objectId;  // fallback if mapping is not found later
  return returnvalue::OK;
}

// Convert TC payload to internal command messages and optional state machine setup
ReturnValue_t RwPusService::prepareCommand(CommandMessage* message, uint8_t subservice,
                                           const uint8_t* tcData, size_t tcLen, uint32_t* state,
                                           object_id_t objectId) {
#if RW_PUS_VERBOSE
  sif::info << "RwPusService::prepareCommand: TC subservice=" << int(subservice) << " len=" << tcLen << std::endl;
#endif
  if (ipcStore == nullptr) return returnvalue::FAILED;
  if (tcLen < 4) return CommandingServiceBase::INVALID_TC;

  // AppData after the 4-byte OID
  const uint8_t* app = tcData + 4;
  const size_t appLen = tcLen - 4;

  if (state) *state = static_cast<uint32_t>(CmdState::NONE);

  switch (static_cast<Subservice>(subservice)) {
    case Subservice::SET_SPEED: {
      // Payload: int16 rpm
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

    case Subservice::SET_TORQUE: {
      // Payload: int16 torque_mNm
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
      // No payload needed
      store_address_t sid{};
      uint8_t* p = nullptr;
      auto rv = ipcStore->getFreeElement(&sid, 1, &p);
      if (rv != returnvalue::OK) return rv;
      *p = 0x00;
      ActionMessage::setCommand(message, 0x02 /* CMD_STOP */, sid);
      return returnvalue::OK;
    }

    case Subservice::STATUS: {
      // Expect data reply later, set state to WAIT_DATA
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
      // Payload: mode(1) | submode(1)
      if (appLen < 2) return CommandingServiceBase::INVALID_TC;
      const uint8_t mode = app[0];
      const uint8_t submode = app[1];
      ModeMessage::setModeMessage(message, ModeMessage::CMD_MODE_COMMAND, mode, submode);
      return returnvalue::OK;
    }

    case Subservice::ACS_SET_ENABLE: {
      // Local handling: enable or disable ACS controller
      if (appLen < 1) return CommandingServiceBase::INVALID_TC;
      const bool enable = app[0] != 0;
      auto* acs = ObjectManager::instance()->get<AcsController>(objectId);
      if (acs == nullptr) {
#if RW_PUS_VERBOSE
        sif::warning << "RwPusService::prepareCommand: ACS OID 0x" << std::hex << objectId << std::dec
                     << " not found" << std::endl;
#endif
        return CommandingServiceBase::INVALID_OBJECT;
      }
      acs->enable(enable);
      // Emit typed HK right away so operator sees the change
      (void)emitAcsTypedHk(objectId);
      return CommandingServiceBase::EXECUTION_COMPLETE;
    }

    case Subservice::ACS_SET_TARGET: {
      // Local handling: set new target attitude quaternion (big-endian f32 * 4)
      if (appLen < 16) return CommandingServiceBase::INVALID_TC;
      float q0 = be_f32(app + 0);
      float q1 = be_f32(app + 4);
      float q2 = be_f32(app + 8);
      float q3 = be_f32(app + 12);

      // Normalize defensively (fallback to identity on tiny norm)
      float n = std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
      if (n > 1e-6f) { q0/=n; q1/=n; q2/=n; q3/=n; } else { q0=1.f; q1=q2=q3=0.f; }

      auto* acs = ObjectManager::instance()->get<AcsController>(objectId);
      if (acs == nullptr) {
        return CommandingServiceBase::INVALID_OBJECT;
      }
      acs->setTargetAttitude(std::array<float,4>{q0,q1,q2,q3});

      // Emit typed HK to see the new reference early
      (void)emitAcsTypedHk(objectId);

      return CommandingServiceBase::EXECUTION_COMPLETE;
    }

    default:
      return CommandingServiceBase::INVALID_TC;
  }
}

// Read a stored reply payload, try to parse a STATUS frame and emit typed TM (220/131)
ReturnValue_t RwPusService::handleDataReplyAndEmitTm(store_address_t sid, object_id_t objectId) {
  if (sid.raw == StorageManagerIF::INVALID_ADDRESS) {
#if RW_PUS_VERBOSE
    sif::warning << "RwPusService::handleDataReplyAndEmitTm: Invalid store address (sid invalid)" << std::endl;
#endif
  }

  // Try IPC, then TM, then TC store in this order
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
    sif::warning << "RwPusService::handleDataReplyAndEmitTm: Try "
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
    sif::warning << "RwPusService::handleDataReplyAndEmitTm: sid=0x" << std::hex << sid.raw << std::dec
                 << " has no usable payload -> keep waiting" << std::endl;
#endif
  }

  // Try to locate and validate a STATUS frame
  dumpHexWarn("RwPusService::handleDataReplyAndEmitTm: Head", buf, (len < 32 ? len : size_t(32)));
  const int idx = findStatusFrameCrc16(buf, len);

  ReturnValue_t rv = returnvalue::OK;
  if (idx >= 0) {
    RwProtocol::Status st{};
    const bool ok = RwProtocol::parseStatus(buf + idx, RwProtocol::STATUS_LEN, st);
    if (!ok) {
      rv = returnvalue::FAILED;
    } else {
      // Build typed TM v1 (28 bytes):
      // ver(1)=1 | oid(4) | speed(i16) | torque(i16) | running(u8)
      // | flags(u16)=0 | err(u16)=0 | crcCnt(u32)=0 | malCnt(u32)=0 | tsMs(u32)=uptime | sample(u16)++
      uint8_t app[28] = {};
      size_t off = 0;
      uint32_t tsMs = 0;
      (void)Clock::getUptime(&tsMs);
      static uint16_t sample = 0;

      app[off++] = 1;  // version
      be_store_u32(&app[off], static_cast<uint32_t>(objectId)); off += 4;
      app[off++] = static_cast<uint8_t>((st.speedRpm >> 8) & 0xFF);
      app[off++] = static_cast<uint8_t>( st.speedRpm       & 0xFF);
      app[off++] = static_cast<uint8_t>((st.torqueMnM >> 8) & 0xFF);
      app[off++] = static_cast<uint8_t>( st.torqueMnM       & 0xFF);
      app[off++] = st.running;
      be_store_u16(&app[off], 0); off += 2;      // flags
      be_store_u16(&app[off], 0); off += 2;      // err
      be_store_u32(&app[off], 0); off += 4;      // crcCnt (unknown here)
      be_store_u32(&app[off], 0); off += 4;      // malCnt (unknown here)
      be_store_u32(&app[off], tsMs); off += 4;   // timestamp ms
      be_store_u16(&app[off], sample++); off += 2;

      const auto prv = tmHelper.prepareTmPacket(static_cast<uint8_t>(Subservice::TM_STATUS_TYPED),
                                                app, off);
      rv = (prv == returnvalue::OK) ? tmHelper.storeAndSendTmPacket() : prv;
    }
  } else {
    rv = returnvalue::FAILED;
  }

  // Clean up storage
  if (usedStore != nullptr) {
    (void)usedStore->deleteData(sid);
  }

  return rv;
}

// Handle replies from handlers, progress state machine, and complete TCs when ready
ReturnValue_t RwPusService::handleReply(const CommandMessage* reply, Command_t, uint32_t* state,
                                        CommandMessage*, object_id_t objectId, bool* isStep) {
#if RW_PUS_VERBOSE
  sif::info << "RwPusService::handleReply: cmd=0x" << std::hex << int(reply->getCommand())
            << " (" << cmdName(reply->getCommand()) << ")" << std::dec << std::endl;
#endif

  const auto cmd = reply->getCommand();
  CmdState st = CmdState::NONE;
  if (state != nullptr) st = static_cast<CmdState>(*state);

  // Data-like replies carry a store id; parse and emit TM if possible
  const bool isDataLike = (cmd == ActionMessage::DATA_REPLY) ||
                          (cmd == DeviceHandlerMessage::REPLY_DIRECT_COMMAND_DATA) ||
                          (cmd == DeviceHandlerMessage::REPLY_RAW_REPLY);

  if (isDataLike) {
    const store_address_t sidAction = ActionMessage::getStoreId(reply);
    const store_address_t sidDh = DeviceHandlerMessage::getStoreAddress(reply);
    const store_address_t sid =
        (sidAction.raw != StorageManagerIF::INVALID_ADDRESS) ? sidAction : sidDh;

    if (sid.raw == StorageManagerIF::INVALID_ADDRESS) {
      // No payload yet; if we are waiting for data, step the state machine
      if (st == CmdState::WAIT_DATA) {
        if (isStep) *isStep = true;
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

  // Non-data replies: progress or complete depending on state
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

// Unrequested replies (e.g. spontaneous STATUS) are treated like data and emitted as TM
void RwPusService::handleUnrequestedReply(CommandMessage* reply) {
#if RW_PUS_VERBOSE
  const auto cmd = reply->getCommand();
  sif::warning << "RwPusService::handleUnrequestedReply: cmd=0x" << std::hex << int(cmd) << " ("
               << cmdName(cmd) << ")" << std::dec << std::endl;
#endif

  const store_address_t sidAction = ActionMessage::getStoreId(reply);
  const store_address_t sidDh = DeviceHandlerMessage::getStoreAddress(reply);

  if (sidAction.raw != StorageManagerIF::INVALID_ADDRESS ||
      sidDh.raw != StorageManagerIF::INVALID_ADDRESS) {
    const store_address_t sid =
        (sidAction.raw != StorageManagerIF::INVALID_ADDRESS) ? sidAction : sidDh;

    // Map sender queue -> object id to identify which RW sent the frame
    object_id_t obj = lastTargetObjectId_;
    const MessageQueueId_t sender = reply->getSender();
    auto it = qidToObj_.find(sender);
    if (it != qidToObj_.end()) {
      obj = it->second;
    }

#if RW_PUS_VERBOSE
    sif::warning << "RwPusService::handleUnrequestedReply: Treating message as DATA (sid=0x" << std::hex << sid.raw
                 << std::dec << "), object=0x" << std::hex << obj << std::dec
                 << " senderQ=0x" << std::hex << sender << std::dec << std::endl;
#endif
    (void)handleDataReplyAndEmitTm(sid, obj);
  }
}

// Compose and emit typed ACS HK (PUS 220) using big-endian fields
ReturnValue_t RwPusService::emitAcsTypedHk(object_id_t acsObjectId) {
  auto* acs = ObjectManager::instance()->get<AcsController>(acsObjectId);
  if (acs == nullptr) {
    return CommandingServiceBase::INVALID_OBJECT;
  }

  AcsDiagSnapshot snap = acs->getDiag();

  // AppData layout (big-endian):
  // [OID(4) | ver(1) | enabled(1) | Kd[3]*f32 | tauDes[3]*f32 | tauWheelCmd[4]*f32 | dtMs(u32)]
  uint8_t app[50] = {};
  size_t off = 0;
  be_store_u32(&app[off], static_cast<uint32_t>(acsObjectId)); off += 4;
  app[off++] = snap.version;
  app[off++] = snap.enabled ? 1 : 0;
  for (int i = 0; i < 3; ++i) { be_store_f32(&app[off], snap.Kd[i]); off += 4; }
  for (int i = 0; i < 3; ++i) { be_store_f32(&app[off], snap.tauDes[i]); off += 4; }
  for (int i = 0; i < 4; ++i) { be_store_f32(&app[off], snap.tauWheelCmd[i]); off += 4; }
  be_store_u32(&app[off], snap.dtMs); off += 4;

  const auto prv = tmHelper.prepareTmPacket(static_cast<uint8_t>(Subservice::TM_ACS_HK_TYPED),
                                            app, off);
  if (prv != returnvalue::OK) return prv;
  return tmHelper.storeAndSendTmPacket();
}
