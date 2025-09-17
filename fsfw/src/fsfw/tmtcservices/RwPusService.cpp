#include "RwPusService.h"

#include <cstring>
#include "fsfw/action/ActionMessage.h"
#include "fsfw/devicehandlers/DeviceHandlerIF.h"
#include "fsfw/devicehandlers/DeviceHandlerMessage.h"
#include "fsfw/modes/ModeMessage.h"
#include "fsfw/objectmanager/ObjectManager.h"
#include "fsfw/serviceinterface/ServiceInterfaceStream.h"
#include "fsfw/storagemanager/StorageManagerIF.h"
#include "commonObjects.h"
#include "fsfw/ipc/MessageQueueSenderIF.h"

// Small helpers for big-endian decoding (keep local to this TU)
namespace {
inline uint32_t be32(const uint8_t* p) {
  return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | uint32_t(p[3]);
}
}  // namespace

RwPusService::RwPusService(object_id_t objectId, uint16_t apid, uint8_t serviceId,
                           uint8_t numParallelCommands, uint16_t commandTimeoutSeconds)
    : CommandingServiceBase(objectId, apid, "PUS 220 RW CMD", serviceId, numParallelCommands,
                            commandTimeoutSeconds) {}

ReturnValue_t RwPusService::initialize() {
  auto res = CommandingServiceBase::initialize();
  if (res != returnvalue::OK) {
    return res;
  }
  ipcStore = ObjectManager::instance()->get<StorageManagerIF>(objects::IPC_STORE);
  if (ipcStore == nullptr) {
    sif::warning << "RwPusService: IPC_STORE not available!" << std::endl;
    return returnvalue::FAILED;
  }
  return returnvalue::OK;
}

ReturnValue_t RwPusService::isValidSubservice(uint8_t subservice) {
  // Validate only TC subservices. Do NOT list TM subservices here.
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

ReturnValue_t RwPusService::getMessageQueueAndObject(uint8_t /*subservice*/, const uint8_t* tcData,
                                                     size_t tcLen, MessageQueueId_t* id,
                                                     object_id_t* objectId) {
  if (tcLen < 4) {
    return CommandingServiceBase::INVALID_TC;
  }
  *objectId = static_cast<object_id_t>(be32(tcData));
  auto* dh = ObjectManager::instance()->get<DeviceHandlerIF>(*objectId);
  if (dh == nullptr) {
    return CommandingServiceBase::INVALID_OBJECT;
  }
  *id = dh->getCommandQueue();
  return returnvalue::OK;
}

ReturnValue_t RwPusService::prepareCommand(CommandMessage* message, uint8_t subservice,
                                           const uint8_t* tcData, size_t tcLen, uint32_t*,
                                           object_id_t) {
  // Trace incoming TC
  sif::info << "RwPusService: TC subservice=" << int(subservice) << " len=" << tcLen << std::endl;

  if (ipcStore == nullptr) {
    return returnvalue::FAILED;
  }
  if (tcLen < 4) {
    return CommandingServiceBase::INVALID_TC;
  }
  const uint8_t* app = tcData + 4;
  const size_t appLen = tcLen - 4;

  switch (static_cast<Subservice>(subservice)) {
    case Subservice::SET_SPEED: {
      if (appLen < 2) return CommandingServiceBase::INVALID_TC;

      // Store native int16_t RPM for the device handler (handler builds the raw frame)
      int16_t rpm = static_cast<int16_t>((app[0] << 8) | app[1]);
      store_address_t sid = store_address_t::invalid();
      uint8_t* p = nullptr;
      auto res = ipcStore->getFreeElement(&sid, sizeof(rpm), &p);
      if (res != returnvalue::OK) return res;
      std::memcpy(p, &rpm, sizeof(rpm));

      // Send as action/command with your deviceCommandId (0x01)
      ActionMessage::setCommand(message, 0x01 /* CMD_SET_SPEED */, sid);
      return returnvalue::OK;
    }

    case Subservice::STOP: {
      ActionMessage::setCommand(message, 0x02 /* CMD_STOP */, store_address_t::invalid());
      return returnvalue::OK;
    }

    case Subservice::STATUS: {
      ActionMessage::setCommand(message, 0x03 /* CMD_STATUS */, store_address_t::invalid());
      return returnvalue::OK;
    }

    case Subservice::SET_MODE: {
      if (appLen < 2) return CommandingServiceBase::INVALID_TC;
      const uint8_t mode = app[0];
      const uint8_t submode = app[1];
      sif::info << "RwPusService: SET_MODE req -> mode=" << int(mode) << ", sub=" << int(submode)
                << std::endl;
      ModeMessage::setModeMessage(message, ModeMessage::CMD_MODE_COMMAND, mode, submode);
      return returnvalue::OK;
    }

    default:
      return CommandingServiceBase::INVALID_TC;
  }
}

// --- RwPusService::handleReply (build TM with tmHelper, service=220 is implicit) ---
ReturnValue_t RwPusService::handleReply(const CommandMessage* reply, Command_t /*previousCommand*/,
                                        uint32_t* /*state*/, CommandMessage* /*next*/,
                                        object_id_t objectId, bool* isStep) {
  // Trace reply opcode
  sif::info << "RwPusService::handleReply cmd=0x" << std::hex << int(reply->getCommand())
            << std::dec << std::endl;

  // --- Action framework housekeeping ---
  switch (reply->getCommand()) {
    case ActionMessage::STEP_SUCCESS:    *isStep = true; return returnvalue::OK;
    case ActionMessage::STEP_FAILED:     *isStep = true; return ActionMessage::getReturnCode(reply);
    case ActionMessage::COMPLETION_SUCCESS:            return CommandingServiceBase::EXECUTION_COMPLETE;
    case ActionMessage::COMPLETION_FAILED:             return ActionMessage::getReturnCode(reply);
    default: break;
  }

  // --- Resolve store address by reply type ---
  store_address_t sid = store_address_t::invalid();
  const Command_t cmd = reply->getCommand();

  if (cmd == ActionMessage::DATA_REPLY ||
      cmd == DeviceHandlerMessage::REPLY_DIRECT_COMMAND_DATA) {
    // DATA_REPLY path: store id sits in ActionMessage
    sid = ActionMessage::getStoreId(reply);
  } else if (cmd == DeviceHandlerMessage::REPLY_RAW_REPLY) {
    // RAW_REPLY path: store id sits in DeviceHandlerMessage
    sid = DeviceHandlerMessage::getStoreAddress(reply);
  }

  if (sid == store_address_t::invalid()) {
    // Not a data-carrying reply; do not escalate to [1,6]
    sif::warning << "RwPusService: Reply 0x" << std::hex << int(cmd)
                 << " has no store data. Ignoring." << std::dec << std::endl;
    return returnvalue::OK;
  }

  // --- Pull store payload ---
  const uint8_t* buf = nullptr;
  size_t len = 0;
  if (ipcStore == nullptr || ipcStore->getData(sid, &buf, &len) != returnvalue::OK) {
    return CommandingServiceBase::INVALID_REPLY;
  }

  // Tiny hex dump (first up to 24 bytes) to verify content visually
  {
    size_t dump = len < 24 ? len : size_t{24};
    sif::info << "RwPusService: store len=" << len << " head=";
    for (size_t i = 0; i < dump; ++i) {
      sif::info << std::hex << (i ? " " : "") << int(buf[i]) << std::dec;
    }
    sif::info << std::endl;
  }

  // --- Find 'AB 10' device frame anywhere in the buffer (dataset-safe) ---
  auto crc8 = [](const uint8_t* d) -> uint8_t {
    uint8_t c = 0x00;
    for (int i = 0; i < 7; ++i) {
      c ^= d[i];
      for (int j = 0; j < 8; ++j) {
        c = (c & 0x80) ? static_cast<uint8_t>((c << 1) ^ 0x07) : static_cast<uint8_t>(c << 1);
      }
    }
    return c;
  };

  size_t start = SIZE_MAX;
  // Scan full payload; CRC check filters false positives
  for (size_t i = 0; i + 8 <= len; ++i) {
    if (buf[i] == 0xAB && buf[i + 1] == 0x10 && crc8(buf + i) == buf[i + 7]) {
      start = i;
      break;
    }
  }

  ReturnValue_t rv = CommandingServiceBase::INVALID_REPLY;

  if (start != SIZE_MAX) {
    const uint8_t* p = buf + start;
    const int16_t speed   = static_cast<int16_t>((p[2] << 8) | p[3]);
    const int16_t torque  = static_cast<int16_t>((p[4] << 8) | p[5]);
    const uint8_t running = p[6];

    // Build AppData: [object_id(4) | speed(2) | torque(2) | running(1)]
    uint8_t app[4 + 2 + 2 + 1];
    app[0] = static_cast<uint8_t>((objectId >> 24) & 0xFF);
    app[1] = static_cast<uint8_t>((objectId >> 16) & 0xFF);
    app[2] = static_cast<uint8_t>((objectId >> 8) & 0xFF);
    app[3] = static_cast<uint8_t>(objectId & 0xFF);
    app[4] = static_cast<uint8_t>((speed >> 8) & 0xFF);
    app[5] = static_cast<uint8_t>(speed & 0xFF);
    app[6] = static_cast<uint8_t>((torque >> 8) & 0xFF);
    app[7] = static_cast<uint8_t>(torque & 0xFF);
    app[8] = running;

    if (tmHelper.prepareTmPacket(static_cast<uint8_t>(Subservice::TM_STATUS), app,
                                 sizeof(app)) == returnvalue::OK) {
      sif::info << "RwPusService: sending TM_STATUS (svc=220, sub=130) "
                << "speed=" << speed << " rpm, torque=" << torque
                << " mNm, running=" << int(running) << std::endl;
      rv = tmHelper.storeAndSendTmPacket();
    }
  } else {
    sif::warning << "RwPusService: No 'AB 10' frame found in store payload, ignoring." << std::endl;
  }

  // Always free IPC store element
  (void)ipcStore->deleteData(sid);

  *isStep = true;  // data replies are steps; completion may follow
  return rv;
}

