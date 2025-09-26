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
#include "fsfw/modes/ModeMessage.h"
#include "fsfw/objectmanager/ObjectManager.h"
#include "fsfw/returnvalues/returnvalue.h"
#include "fsfw/serviceinterface/ServiceInterfaceStream.h"
#include "fsfw/storagemanager/StorageManagerIF.h"
#include "fsfw/storagemanager/storeAddress.h"
#include "fsfw/timemanager/Clock.h"
#include "fsfw/datapoollocal/LocalPoolVariable.h"
#include "fsfw/devicehandlers/ReactionWheelsHandler.h"

/*
 * RwPusService.cpp - PUS service for Reaction Wheels and ACS
 *
 * RwPusService (Service 220) handles TM/TC to and from RW DeviceHandlers and the ACS controller.
 * Converts TCs to DH actions/mode commands, parse replies, and send typed TM.
 *
 * Features:
 *  - Built on CommandingServiceBase
 *  - Uses RwProtocol.h for parsing/building commands
 *  - Send typed TM: RW status via 220/131 and ACS attitude YPR via 220/133,
 *    as well as ACS HK via 220/132 that includes controller state
 *  - Periodic sending of status gated by HK age and/or ACS enable state
 *  - Handles unrequested replies (STATUS polls -> typed TM)
 *
 * Notes:
 *  - All AppData fields are big-endian
 *  - Quaternion targets are normalized (fallback to identity if degenerate)
 *  - Debug guarded by RW_PUS_VERBOSE
 *
 *  - Joest Homann
 */

namespace {

// -------- Byte order: load helpers -------------------------------------------
// Read u32 (big-endian, network order -> host)
inline uint32_t be32(const uint8_t* p) {
  return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | uint32_t(p[3]);
}

// Read f32 (big-endian) via bitwise copy (avoid aliasing/UB)
inline float be_f32(const uint8_t* p) {
  const uint32_t u = be32(p);
  float f;
  std::memcpy(&f, &u, sizeof(float));
  return f;
}

// -------- STATUS frame finder ------------------------------------------------
// Linear scan for a valid STATUS frame (header+ID+CRC ok). Return offset or -1.
int findStatusFrameCrc16(const uint8_t* buf, size_t len) {
  if (len < RwProtocol::STATUS_LEN) return -1;
  for (size_t i = 0; i + (RwProtocol::STATUS_LEN - 1) < len; ++i) {
    if (buf[i] == RwProtocol::START_REPLY &&
        buf[i + 1] == static_cast<uint8_t>(RwProtocol::RespId::STATUS)) {
      if (RwProtocol::verifyCrc16(buf + i, RwProtocol::STATUS_LEN)) {
        return static_cast<int>(i);  // first valid hit
      }
    }
  }
  return -1;
}

// -------- Byte order: store helpers ------------------------------------------
// Store u32 (big-endian)
static inline void be_store_u32(uint8_t* p, uint32_t v) {
  p[0] = static_cast<uint8_t>((v >> 24) & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 16) & 0xFF);
  p[2] = static_cast<uint8_t>((v >> 8) & 0xFF);
  p[3] = static_cast<uint8_t>( v       & 0xFF);
}

// Store u16 (big-endian)
static inline void be_store_u16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>((v >> 8) & 0xFF);
  p[1] = static_cast<uint8_t>( v       & 0xFF);
}

// Store f32 (big-endian) via bitwise copy
static inline void be_store_f32(uint8_t* p, float f) {
  static_assert(sizeof(float) == 4, "float must be 4 bytes");
  uint32_t u;
  std::memcpy(&u, &f, 4);
  be_store_u32(p, u);
}

#if RW_PUS_VERBOSE
// -------- Hex dump (debug) ---------------------------------------------------
// Write bytes as hex to Service Interface warning stream
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

#if RW_PUS_VERBOSE
// -------- Debug Action Message Output -----------------------------
// Map FSFW command IDs to strings
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
#endif

}  // namespace

// -------- Minimal per-command state -----------------------------------------
// Multi-step TCs (e.g., STATUS) wait for data before completion.
enum class CmdState : uint32_t { NONE = 0, WAIT_DATA = 1 };

// -------- Construction -------------------------------------------------------
// Create PUS service (svc 220) with APID, routing, and timeouts
RwPusService::RwPusService(object_id_t objectId, uint16_t apid, uint8_t serviceId,
                           uint8_t numParallelCommands, uint16_t commandTimeoutSeconds)
    : CommandingServiceBase(objectId, apid, "PUS 220 RW CMD", serviceId, numParallelCommands,
                            commandTimeoutSeconds) {}

// -------- Initialization -----------------------------------------------------
// Resolve store (IPC) and complete base init
ReturnValue_t RwPusService::initialize() {
  auto res = CommandingServiceBase::initialize();
  if (res != returnvalue::OK) return res;

  // Only IPC store is needed here (TC payloads and DH data handover)
  ipcStore = ObjectManager::instance()->get<StorageManagerIF>(objects::IPC_STORE);

#if RW_PUS_VERBOSE
  sif::warning << "RwPusService::initialize: ok. myQ=0x" << std::hex << this->getCommandQueue()
               << " ipc=" << ipcStore << std::dec << std::endl;
#endif

  return (ipcStore != nullptr) ? returnvalue::OK : returnvalue::FAILED;
}

// -------- Subservice filter --------------------------------------------------
// Only allow supported subservices
ReturnValue_t RwPusService::isValidSubservice(uint8_t subservice) {
  switch (static_cast<Subservice>(subservice)) {
    case Subservice::SET_SPEED:
    case Subservice::STOP:
    case Subservice::STATUS:
    case Subservice::SET_TORQUE:
    case Subservice::SET_MODE:
    case Subservice::ACS_SET_ENABLE:
    case Subservice::ACS_SET_TARGET:  // Local-only (no DH queue)
      return returnvalue::OK;
    default:
      return AcceptsTelecommandsIF::INVALID_SUBSERVICE;
  }
}

// -------- Target resolution --------------------------------------------------
// Decode target OID from TC, decide local vs DH routing, and cache mapping
ReturnValue_t RwPusService::getMessageQueueAndObject(uint8_t subservice, const uint8_t* tcData,
                                                     size_t tcLen, MessageQueueId_t* id,
                                                     object_id_t* objectId) {
  if (tcLen < 4) return CommandingServiceBase::INVALID_TC;

  // AppData starts with target OID (RW handler or ACS controller)
  *objectId = static_cast<object_id_t>(be32(tcData));

  // Local-only subservices: no DH queue involved
  if (static_cast<Subservice>(subservice) == Subservice::ACS_SET_ENABLE ||
      static_cast<Subservice>(subservice) == Subservice::ACS_SET_TARGET) {
    *id = MessageQueueIF::NO_QUEUE;
    lastTargetObjectId_ = *objectId;   // remember target for TM context
    acsPollOid_ = *objectId;           // used for periodic 220/133
    return returnvalue::OK;
  }

  // Default path: forward to DeviceHandler (RW instance)
  auto* dh = ObjectManager::instance()->get<DeviceHandlerIF>(*objectId);
  if (dh == nullptr) {
#if RW_PUS_VERBOSE
    sif::warning << "RwPusService::getMessageQueueAndObject: INVALID_OBJECT 0x"
                 << std::hex << *objectId << std::dec << std::endl;
#endif
    return CommandingServiceBase::INVALID_OBJECT;
  }

  *id = dh->getCommandQueue();         // DH command queue

  // Cache senderQ -> objectId to attribute unrequested replies (which RW?)
  qidToObj_[*id] = *objectId;

  // Remember preferred RW for periodic typed TM
  rwPollOid_ = *objectId;

#if RW_PUS_VERBOSE
  sif::warning << "RwPusService::getMessageQueueAndObject: commandQueue=0x" << std::hex << *id
               << std::dec << std::endl;
#endif

  lastTargetObjectId_ = *objectId;     // Fallback if mapping is missing later
  return returnvalue::OK;
}

// -------- TC -> internal command --------------------------------------------
// Translate AppData to internal messages (+ optional state for multi-step)
ReturnValue_t RwPusService::prepareCommand(CommandMessage* message, uint8_t subservice,
                                           const uint8_t* tcData, size_t tcLen, uint32_t* state,
                                           object_id_t objectId) {
#if RW_PUS_VERBOSE
  sif::info << "RwPusService::prepareCommand: subservice=" << int(subservice)
            << " len=" << tcLen << std::endl;
#endif
  if (ipcStore == nullptr) return returnvalue::FAILED;
  if (tcLen < 4) return CommandingServiceBase::INVALID_TC;

  // AppData after the leading OID
  const uint8_t* app   = tcData + 4;
  const size_t   appLen = tcLen - 4;

  if (state) *state = static_cast<uint32_t>(CmdState::NONE);  // Default: single-shot

  switch (static_cast<Subservice>(subservice)) {
    case Subservice::SET_SPEED: {
      // Build ActionMessage with rpm payload in IPC store
      if (appLen < 2) return CommandingServiceBase::INVALID_TC;
      const int16_t rpm = static_cast<int16_t>((app[0] << 8) | app[1]);
      store_address_t sid{}; uint8_t* p = nullptr;
      auto rv = ipcStore->getFreeElement(&sid, sizeof(rpm), &p);
      if (rv != returnvalue::OK) return rv;
      std::memcpy(p, &rpm, sizeof(rpm));
      ActionMessage::setCommand(message, 0x01 /* CMD_SET_SPEED */, sid);
      return returnvalue::OK;
    }
    case Subservice::SET_TORQUE: {
      // Build ActionMessage with torque payload [mNm]
      if (appLen < 2) return CommandingServiceBase::INVALID_TC;
      const int16_t tq = static_cast<int16_t>((app[0] << 8) | app[1]);
      store_address_t sid{}; uint8_t* p = nullptr;
      auto rv = ipcStore->getFreeElement(&sid, sizeof(tq), &p);
      if (rv != returnvalue::OK) return rv;
      std::memcpy(p, &tq, sizeof(tq));
      ActionMessage::setCommand(message, 0x04 /* CMD_SET_TORQUE */, sid);
      return returnvalue::OK;
    }
    case Subservice::STOP: {
      // STOP has no payload - put a dummy byte for uniformity
      store_address_t sid{}; uint8_t* p = nullptr;
      auto rv = ipcStore->getFreeElement(&sid, 1, &p);
      if (rv != returnvalue::OK) return rv;
      *p = 0x00;
      ActionMessage::setCommand(message, 0x02 /* CMD_STOP */, sid);
      return returnvalue::OK;
    }
    case Subservice::STATUS: {
      // Multi-step: expect DATA later -> enter WAIT_DATA
      if (state) *state = static_cast<uint32_t>(CmdState::WAIT_DATA);
      store_address_t sid{}; uint8_t* p = nullptr;
      auto rv = ipcStore->getFreeElement(&sid, 1, &p);
      if (rv != returnvalue::OK) return rv;
      *p = 0x00;
      ActionMessage::setCommand(message, 0x03 /* CMD_STATUS */, sid);
      return returnvalue::OK;
    }
    case Subservice::SET_MODE: {
      // Forward mode/submode to DH
      if (appLen < 2) return CommandingServiceBase::INVALID_TC;
      const uint8_t mode = app[0];
      const uint8_t submode = app[1];
      ModeMessage::setModeMessage(message, ModeMessage::CMD_MODE_COMMAND, mode, submode);
      return returnvalue::OK;
    }
    case Subservice::ACS_SET_ENABLE: {
      // Local: enable/disable ACS and emit typed HK immediately
      if (appLen < 1) return CommandingServiceBase::INVALID_TC;
      const bool enable = app[0] != 0;
      auto* acs = ObjectManager::instance()->get<AcsController>(objectId);
      if (acs == nullptr) {
#if RW_PUS_VERBOSE
        sif::warning << "RwPusService::prepareCommand: ACS OID 0x" << std::hex << objectId
                     << std::dec << " not found" << std::endl;
#endif
        return CommandingServiceBase::INVALID_OBJECT;
      }
      acs->enable(enable);                 // apply now

      // Cache for periodic 220/133 gating
      acsEnabledCached_ = enable;
      acsPollOid_       = objectId;

      // Emit HK so operator sees the change immediately
      (void)emitAcsTypedHk(objectId);
      return CommandingServiceBase::EXECUTION_COMPLETE;
    }
    case Subservice::ACS_SET_TARGET: {
      // Local: set quaternion target (normalize for safety)
      if (appLen < 16) return CommandingServiceBase::INVALID_TC;
      float q0 = be_f32(app + 0);
      float q1 = be_f32(app + 4);
      float q2 = be_f32(app + 8);
      float q3 = be_f32(app + 12);

      // Normalize; if near-zero length, use identity
      float n = std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
      if (n > 1e-6f) { q0/=n; q1/=n; q2/=n; q3/=n; } else { q0=1.f; q1=q2=q3=0.f; }

      auto* acs = ObjectManager::instance()->get<AcsController>(objectId);
      if (acs == nullptr) {
        return CommandingServiceBase::INVALID_OBJECT;
      }
      acs->setTargetAttitude(std::array<float,4>{q0,q1,q2,q3});

      // Emit HK so the new reference is visible right away
      (void)emitAcsTypedHk(objectId);
      return CommandingServiceBase::EXECUTION_COMPLETE;
    }
    default:
      return CommandingServiceBase::INVALID_TC;
  }
}

// -------- Data reply handler -------------------------------------------------
// Read stored reply (IPC), parse STATUS, and emit typed TM 220/131
ReturnValue_t RwPusService::handleDataReplyAndEmitTm(store_address_t sid, object_id_t objectId) {
  if (sid.raw == StorageManagerIF::INVALID_ADDRESS) {
#if RW_PUS_VERBOSE
    sif::warning << "RwPusService::handleDataReplyAndEmitTm: invalid store address" << std::endl;
#endif
  }

  // Single-store path (IPC): look up payload buffer
  StorageManagerIF* stores[1] = {ipcStore};
  const uint8_t* buf = nullptr; size_t len = 0;
  StorageManagerIF* usedStore = nullptr;

  for (StorageManagerIF* s : stores) {
    if (s == nullptr) continue;
    const uint8_t* b = nullptr; size_t l = 0;
    auto rv = s->getData(sid, &b, &l);
    if (rv == returnvalue::OK && b != nullptr && l > 0) {
      buf = b; len = l; usedStore = s; break;      // First valid hit
    }
  }

  if (buf == nullptr || len == 0) {
    return returnvalue::FAILED;                     // Nothing to parse
  }

  // Find a valid STATUS frame inside the stored bytes
  const int idx = findStatusFrameCrc16(buf, len);
  ReturnValue_t rv = returnvalue::OK;
  if (idx >= 0) {
    RwProtocol::Status st{};
    const bool ok = RwProtocol::parseStatus(buf + idx, RwProtocol::STATUS_LEN, st);
    if (!ok) {
      rv = returnvalue::FAILED;                     // Parse error
    } else {
      // Build typed TM v1 (28 bytes)
      // Layout: ver(1) | oid(u32) | speed(i16) | torque(i16) | running(u8) |
      //         flags(u16)=0 | err(u16)=0 | crcCnt(u32)=0 | malCnt(u32)=0 | tsMs(u32) | sample(u16)
      uint8_t app[28] = {};
      size_t off = 0;
      uint32_t tsMs = 0;
      (void)Clock::getUptime(&tsMs);
      static uint16_t sample = 0;

      app[off++] = 1;  // version
      be_store_u32(&app[off], static_cast<uint32_t>(objectId)); off += 4;
      app[off++] = static_cast<uint8_t>((st.speedRpm  >> 8) & 0xFF);
      app[off++] = static_cast<uint8_t>( st.speedRpm        & 0xFF);
      app[off++] = static_cast<uint8_t>((st.torqueMnM >> 8) & 0xFF);
      app[off++] = static_cast<uint8_t>( st.torqueMnM       & 0xFF);
      app[off++] = st.running;
      be_store_u16(&app[off], 0);   off += 2;      // Flags
      be_store_u16(&app[off], 0);   off += 2;      // Err
      be_store_u32(&app[off], 0);   off += 4;      // CrcCnt placeholder
      be_store_u32(&app[off], 0);   off += 4;      // MalCnt placeholder
      be_store_u32(&app[off], tsMs); off += 4;     // Timestamp ms
      be_store_u16(&app[off], sample++); off += 2; // Monotonic sample id

      const auto prv =
          tmHelper.prepareTmPacket(static_cast<uint8_t>(Subservice::TM_STATUS_TYPED), app, off);
      rv = (prv == returnvalue::OK) ? tmHelper.storeAndSendTmPacket() : prv;
    }
  } else {
    rv = returnvalue::FAILED;                         // No STATUS frame found
  }

  if (usedStore != nullptr) {
    (void)usedStore->deleteData(sid);                 // Free store slot
  }
  return rv;
}

// -------- Reply router -------------------------------------------------------
// Handle DH replies, advance WAIT_DATA, complete TCs when appropriate
ReturnValue_t RwPusService::handleReply(const CommandMessage* reply, Command_t, uint32_t* state,
                                        CommandMessage*, object_id_t objectId, bool* isStep) {
#if RW_PUS_VERBOSE
  sif::info << "RwPusService::handleReply: cmd=0x" << std::hex << int(reply->getCommand())
            << " (" << cmdName(reply->getCommand()) << ")" << std::dec << std::endl;
#endif

  const auto cmd = reply->getCommand();
  CmdState st = CmdState::NONE;
  if (state != nullptr) st = static_cast<CmdState>(*state);

  // Data-like replies carry a store id; try to convert to typed TM
  const bool isDataLike = (cmd == ActionMessage::DATA_REPLY) ||
                          (cmd == DeviceHandlerMessage::REPLY_DIRECT_COMMAND_DATA) ||
                          (cmd == DeviceHandlerMessage::REPLY_RAW_REPLY);

  if (isDataLike) {
    const store_address_t sidAction = ActionMessage::getStoreId(reply);
    const store_address_t sidDh     = DeviceHandlerMessage::getStoreAddress(reply);
    const store_address_t sid =
        (sidAction.raw != StorageManagerIF::INVALID_ADDRESS) ? sidAction : sidDh;

    if (sid.raw == StorageManagerIF::INVALID_ADDRESS) {
      // No payload yet: keep WAIT_DATA alive (step-ack)
      if (st == CmdState::WAIT_DATA) {
        if (isStep) *isStep = true;
        return returnvalue::OK;
      }
      return returnvalue::OK; // Ignore stray
    }

    const auto rv = handleDataReplyAndEmitTm(sid, objectId);
    if (rv == returnvalue::OK) {
      if (state) *state = static_cast<uint32_t>(CmdState::NONE); // Clear after TM emission
      return CommandingServiceBase::EXECUTION_COMPLETE;          // TC finished
    }
    // Still expecting more data; keep stepping
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
        return CommandingServiceBase::EXECUTION_COMPLETE;        // Single-step TC
      }
      if (isStep) *isStep = true;                                // Remain in WAIT_DATA
      return returnvalue::OK;
    }
    case ActionMessage::COMPLETION_SUCCESS: {
      if (st == CmdState::WAIT_DATA) return returnvalue::OK;     // Completion but data pending
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
        if (isStep) *isStep = true;                              // Keep waiting
        return returnvalue::OK;
      }
      return returnvalue::OK;                                    // Ignore others
    }
  }
}

// -------- Unrequested replies -----------------------------------------------
// Spontaneous DH replies (e.g., periodic polls) -> emit typed TM
void RwPusService::handleUnrequestedReply(CommandMessage* reply) {
#if RW_PUS_VERBOSE
  const auto cmd = reply->getCommand();
  sif::warning << "RwPusService::handleUnrequestedReply: cmd=0x" << std::hex << int(cmd) << " ("
               << cmdName(cmd) << ")" << std::dec << std::endl;
#endif

  const store_address_t sidAction = ActionMessage::getStoreId(reply);
  const store_address_t sidDh     = DeviceHandlerMessage::getStoreAddress(reply);

  if (sidAction.raw != StorageManagerIF::INVALID_ADDRESS ||
      sidDh.raw != StorageManagerIF::INVALID_ADDRESS) {
    const store_address_t sid =
        (sidAction.raw != StorageManagerIF::INVALID_ADDRESS) ? sidAction : sidDh;

    // Attribute the reply to a RW: map senderQ -> objectId (fallback to last target)
    object_id_t obj = lastTargetObjectId_;
    const MessageQueueId_t sender = reply->getSender();
    auto it = qidToObj_.find(sender);
    if (it != qidToObj_.end()) {
      obj = it->second;
    }

    (void)handleDataReplyAndEmitTm(sid, obj); // best-effort TM emission
  }
}

// -------- Typed ACS HK (PUS 220/132) ----------------------------------------
// Compose and emit typed ACS HK (big-endian fields) from snapshot
ReturnValue_t RwPusService::emitAcsTypedHk(object_id_t acsObjectId) {
  auto* acs = ObjectManager::instance()->get<AcsController>(acsObjectId);
  if (acs == nullptr) {
    return CommandingServiceBase::INVALID_OBJECT;
  }

  AcsDiagSnapshot snap = acs->getDiag(); // Capture diagnostic snapshot

  // AppData: [OID(4) | ver(1) | enabled(1) | Kd[3]*f32 | tauDes[3]*f32 | tauWheelCmd[4]*f32 | dtMs(u32)]
  uint8_t app[50] = {};
  size_t off = 0;
  be_store_u32(&app[off], static_cast<uint32_t>(acsObjectId)); off += 4;
  app[off++] = snap.version;
  app[off++] = snap.enabled ? 1 : 0;
  for (int i = 0; i < 3; ++i) { be_store_f32(&app[off], snap.Kd[i]);        off += 4; }
  for (int i = 0; i < 3; ++i) { be_store_f32(&app[off], snap.tauDes[i]);    off += 4; }
  for (int i = 0; i < 4; ++i) { be_store_f32(&app[off], snap.tauWheelCmd[i]); off += 4; }
  be_store_u32(&app[off], snap.dtMs); off += 4;

  const auto prv = tmHelper.prepareTmPacket(static_cast<uint8_t>(Subservice::TM_ACS_HK_TYPED),
                                            app, off);
  if (prv != returnvalue::OK) return prv;
  return tmHelper.storeAndSendTmPacket();
}

// -------- Periodic operation -------------------------------------------------
// Every RW_TYPED_EVERY_N ticks: RW 220/131 and ACS 220/133 (if enabled)
void RwPusService::doPeriodicOperation() {
  ++serviceTick_;

  if ((serviceTick_ % RW_TYPED_EVERY_N) == 0) {
    // Prefer last addressed RW - otherwise pick any known RW from map
    object_id_t rwOid = rwPollOid_;
    if (rwOid == objects::NO_OBJECT ||
        ObjectManager::instance()->get<DeviceHandlerIF>(rwOid) == nullptr) {
      for (const auto& kv : qidToObj_) {
        object_id_t cand = kv.second;
        if (ObjectManager::instance()->get<DeviceHandlerIF>(cand) != nullptr) {
          rwOid = cand; break;
        }
      }
    }
    // Emit 220/131 if local HK data is new enough (else skip)
    if (rwOid != objects::NO_OBJECT && isRwHkNew(rwOid)) {
      (void)emitRwTypedFromHk(rwOid);
    }

    // Emit 220/133 only while ACS is enabled (cached state)
    if (acsEnabledCached_ && acsPollOid_ != objects::NO_OBJECT) {
      (void)emitAttYprTm(acsPollOid_);
    }
  }
}

// -------- Typed RW HK (PUS 220/131) -----------------------------------------
// Build and send TM 220/131 directly from RW local HK dataset
ReturnValue_t RwPusService::emitRwTypedFromHk(object_id_t rwObjectId) {
  using PoolIds = ReactionWheelsHandler::PoolIds;

  // Pull values from RW local pool (simple reads; validity is not enforced here)
  LocalPoolVariable<int16_t>  vSpeed (rwObjectId, static_cast<lp_id_t>(PoolIds::HK_SPEED_RPM));
  LocalPoolVariable<int16_t>  vTorque(rwObjectId, static_cast<lp_id_t>(PoolIds::HK_TORQUE_mNm));
  LocalPoolVariable<uint8_t>  vRun   (rwObjectId, static_cast<lp_id_t>(PoolIds::HK_RUNNING));
  LocalPoolVariable<uint16_t> vFlags (rwObjectId, static_cast<lp_id_t>(PoolIds::HK_FLAGS));
  LocalPoolVariable<uint32_t> vCrc   (rwObjectId, static_cast<lp_id_t>(PoolIds::HK_CRC_ERR_CNT));
  LocalPoolVariable<uint32_t> vMal   (rwObjectId, static_cast<lp_id_t>(PoolIds::HK_MALFORMED_CNT));
  LocalPoolVariable<uint32_t> vTsMs  (rwObjectId, static_cast<lp_id_t>(PoolIds::HK_TIMESTAMP_MS));

  (void)vSpeed.read();
  (void)vTorque.read();
  (void)vRun.read();
  (void)vFlags.read();
  (void)vCrc.read();
  (void)vMal.read();
  (void)vTsMs.read();

  // AppData: ver(1)=1 | oid(4) | speed(i16) | torque(i16) | running(u8)
  //          | flags(u16) | err(u16)=0 | crcCnt(u32) | malCnt(u32) | tsMs(u32) | sample(u16)
  uint8_t app[28] = {};
  size_t off = 0;

  app[off++] = 1;  // version
  be_store_u32(&app[off], static_cast<uint32_t>(rwObjectId)); off += 4;

  app[off++] = static_cast<uint8_t>((vSpeed.value   >> 8) & 0xFF);
  app[off++] = static_cast<uint8_t>( vSpeed.value         & 0xFF);
  app[off++] = static_cast<uint8_t>((vTorque.value  >> 8) & 0xFF);
  app[off++] = static_cast<uint8_t>( vTorque.value        & 0xFF);
  app[off++] = vRun.value;

  be_store_u16(&app[off], vFlags.value); off += 2;
  be_store_u16(&app[off], 0);            off += 2;  // err=0 (not evaluated here)
  be_store_u32(&app[off], vCrc.value);   off += 4;
  be_store_u32(&app[off], vMal.value);   off += 4;
  be_store_u32(&app[off], vTsMs.value);  off += 4;

  be_store_u16(&app[off], rwSample_++);  off += 2;

  const auto prv =
      tmHelper.prepareTmPacket(static_cast<uint8_t>(Subservice::TM_STATUS_TYPED), app, off);
  if (prv != returnvalue::OK) {
    return prv;
  }
  return tmHelper.storeAndSendTmPacket();
}

// -------- Typed Attitude YPR (PUS 220/133) ----------------------------------
// Compose and emit TM 220/133 using AcsController::getAttitudeTM()
ReturnValue_t RwPusService::emitAttYprTm(object_id_t acsObjectId) {
  auto* acs = ObjectManager::instance()->get<AcsController>(acsObjectId);
  if (acs == nullptr) {
    return CommandingServiceBase::INVALID_OBJECT;
  }

  // Gate by runtime state: only emit while enabled
  AcsDiagSnapshot diag = acs->getDiag();
  if (!diag.enabled) {
    return returnvalue::OK; // do not emit when disabled
  }

  AcsController::AttitudeTM att{};
  acs->getAttitudeTM(att);               // fill yaw/pitch/roll + error angle

  // AppData (big-endian):
  // ver(1)=1 | oid(4) | enabled(1) |
  // refYaw(f32) refPitch(f32) refRoll(f32) |
  // trueYaw(f32) truePitch(f32) trueRoll(f32) |
  // errAngleDeg(f32) | dtMs(u32) | sample(u16)
  uint8_t app[1 + 4 + 1 + 7 * 4 + 4 + 2] = {};
  size_t off = 0;

  app[off++] = 1; // version
  be_store_u32(&app[off], static_cast<uint32_t>(acsObjectId)); off += 4;
  app[off++] = diag.enabled ? 1 : 0;

  auto putf = [&](float f) { be_store_f32(&app[off], f); off += 4; };
  putf(att.refYawDeg);
  putf(att.refPitchDeg);
  putf(att.refRollDeg);
  putf(att.trueYawDeg);
  putf(att.truePitchDeg);
  putf(att.trueRollDeg);
  putf(att.errAngleDeg);

  be_store_u32(&app[off], diag.dtMs); off += 4;
  be_store_u16(&app[off], attSample_++); off += 2;

  const auto prv =
      tmHelper.prepareTmPacket(static_cast<uint8_t>(Subservice::TM_ATT_YPR_TYPED), app, off);
  if (prv != returnvalue::OK) {
    return prv;
  }
  return tmHelper.storeAndSendTmPacket();
}

// -------- Check for new data helper ----------------------------------------------------
// True if RW HK timestamp age <= maxAgeMs (protect against uptime wrap)
bool RwPusService::isRwHkNew(object_id_t rwOid, uint32_t maxAgeMs) const {
  using PoolIds = ReactionWheelsHandler::PoolIds;

  LocalPoolVariable<uint32_t> vTsMs(rwOid, static_cast<lp_id_t>(PoolIds::HK_TIMESTAMP_MS));
  if (vTsMs.read() != returnvalue::OK || !vTsMs.isValid()) {
    return false;                             // missing/invalid timestamp
  }

  uint32_t nowMs = 0;
  Clock::getUptime(&nowMs);
  if (nowMs < vTsMs.value) {
    return false;                             // uptime reset detected
  }

  const uint32_t age = nowMs - vTsMs.value;
  return age <= maxAgeMs;
}
