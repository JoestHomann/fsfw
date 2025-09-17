// RwPusService.cpp
#include "RwPusService.h"

#include <cstring>
#include <iomanip>      // debug
#include <algorithm>    // debug
#include "fsfw/action/ActionMessage.h"
#include "fsfw/devicehandlers/DeviceHandlerIF.h"
#include "fsfw/devicehandlers/DeviceHandlerMessage.h"
#include "fsfw/modes/ModeMessage.h"
#include "fsfw/objectmanager/ObjectManager.h"
#include "fsfw/returnvalues/returnvalue.h"
#include "fsfw/serviceinterface/ServiceInterfaceStream.h"
#include "fsfw/storagemanager/StorageManagerIF.h"
#include "commonObjects.h"
#include "fsfw/ipc/MessageQueueSenderIF.h"
#include "fsfw/storagemanager/storeAddress.h"

// --- small helpers ---
namespace {
inline uint32_t be32(const uint8_t* p) {
  return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | uint32_t(p[3]);
}

// same CRC8 as in the device handler
uint8_t crc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int j = 0; j < 8; ++j) {
      if (crc & 0x80) crc = static_cast<uint8_t>((crc << 1) ^ 0x07);
      else            crc = static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

// search 8-byte reply AB 10 .. .. .. .. .. CRC in a buffer
int findStatusFrame(const uint8_t* buf, size_t len) {
  if (len < 8) return -1;
  for (size_t i = 0; i + 7 < len; ++i) {
    if (buf[i] == 0xAB && buf[i + 1] == 0x10) {
      if (crc8(buf + i, 7) == buf[i + 7]) {
        return static_cast<int>(i);
      }
    }
  }
  return -1;
}

// debug: hex dump
void dumpHexWarn(const char* tag, const uint8_t* p, size_t n) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
  sif::warning << tag << " (" << n << "): ";
  for (size_t i = 0; i < n; ++i) {
    sif::warning << std::hex << std::setw(2) << std::setfill('0') << int(p[i]) << " ";
  }
  sif::warning << std::dec << std::endl;
#endif
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
  tmStore  = ObjectManager::instance()->get<StorageManagerIF>(objects::TM_STORE);
  tcStore  = ObjectManager::instance()->get<StorageManagerIF>(objects::TC_STORE);
  if (ipcStore == nullptr) {
    sif::warning << "RwPusService: IPC_STORE not available!" << std::endl;
    return returnvalue::FAILED;
  }

  sif::warning << "RwPusService: init ok. myQ=0x" << std::hex << this->getCommandQueue() << std::dec
               << " ipc=" << (void*)ipcStore << " tm=" << (void*)tmStore << " tc=" << (void*)tcStore
               << std::endl;

  return returnvalue::OK;
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

ReturnValue_t RwPusService::getMessageQueueAndObject(uint8_t, const uint8_t* tcData,
                                                     size_t tcLen, MessageQueueId_t* id,
                                                     object_id_t* objectId) {
  if (tcLen < 4) {
    return CommandingServiceBase::INVALID_TC;
  }
  *objectId = static_cast<object_id_t>(be32(tcData));

  // routing visibility
  sif::warning << "PUS220 route: objId=0x" << std::hex << *objectId << std::dec << std::endl;

  auto* dh = ObjectManager::instance()->get<DeviceHandlerIF>(*objectId);
  if (dh == nullptr) {
    sif::warning << "PUS220 route: INVALID_OBJECT for objId=0x" << std::hex << *objectId
                 << std::dec << std::endl;
    return CommandingServiceBase::INVALID_OBJECT;
  }
  *id = dh->getCommandQueue();

  sif::warning << "PUS220 route: commandQueueId=0x" << std::hex << *id << std::dec << std::endl;
  return returnvalue::OK;
}

ReturnValue_t RwPusService::prepareCommand(CommandMessage* message, uint8_t subservice,
                                           const uint8_t* tcData, size_t tcLen, uint32_t*,
                                           object_id_t) {
  sif::info << "RwPusService: TC subservice=" << int(subservice) << " len=" << tcLen << std::endl;

  if (ipcStore == nullptr) {
    return returnvalue::FAILED;
  }
  if (tcLen < 4) {
    return CommandingServiceBase::INVALID_TC;
  }
  const uint8_t* app = tcData + 4;
  const size_t appLen = tcLen - 4;

  sif::warning << "PUS220 prepare: subsvc=" << int(subservice)
               << " appLen=" << appLen << std::endl;

  switch (static_cast<Subservice>(subservice)) {
    case Subservice::SET_SPEED: {
      if (appLen < 2) return CommandingServiceBase::INVALID_TC;
      const int16_t rpm = static_cast<int16_t>((app[0] << 8) | app[1]);

      store_address_t sid{};
      uint8_t* p = nullptr;
      auto res = ipcStore->getFreeElement(&sid, sizeof(rpm), &p);
      if (res != returnvalue::OK) return res;
      std::memcpy(p, &rpm, sizeof(rpm));

      sif::warning << "PUS220 prepare SET_SPEED: rpm=" << rpm
                   << " store=0x" << std::hex << sid.raw << std::dec << std::endl;

      ActionMessage::setCommand(message, 0x01 /* CMD_SET_SPEED */, sid);
      return returnvalue::OK;
    }

    case Subservice::STOP: {
      // Keep Action path uniform: dummy store
      store_address_t sid{};
      uint8_t* p = nullptr;
      auto res = ipcStore->getFreeElement(&sid, 1, &p);
      if (res != returnvalue::OK) return res;
      *p = 0x00;

      sif::warning << "PUS220 prepare STOP (with dummy store) sid=0x"
                   << std::hex << sid.raw << std::dec << std::endl;

      ActionMessage::setCommand(message, 0x02 /* CMD_STOP */, sid);
      return returnvalue::OK;
    }

    case Subservice::STATUS: {
      // Keep Action path uniform: dummy store
      store_address_t sid{};
      uint8_t* p = nullptr;
      auto res = ipcStore->getFreeElement(&sid, 1, &p);
      if (res != returnvalue::OK) return res;
      *p = 0x00;

      sif::warning << "PUS220 prepare STATUS (with dummy store) sid=0x"
                   << std::hex << sid.raw << std::dec << std::endl;

      ActionMessage::setCommand(message, 0x03 /* CMD_STATUS */, sid);
      return returnvalue::OK;
    }

    case Subservice::SET_MODE: {
      if (appLen < 2) return CommandingServiceBase::INVALID_TC;
      const uint8_t mode = app[0];
      const uint8_t submode = app[1];
      sif::info << "RwPusService: SET_MODE req -> mode=" << int(mode)
                << ", sub=" << int(submode) << std::endl;
      ModeMessage::setModeMessage(message, ModeMessage::CMD_MODE_COMMAND, mode, submode);
      return returnvalue::OK;
    }

    default:
      return CommandingServiceBase::INVALID_TC;
  }
}

// --- Reply handling: read store id correctly (ActionMessage::DATA_REPLY), try stores,
//     parse RW frame and emit TM(220,130)
ReturnValue_t RwPusService::handleReply(const CommandMessage* reply, Command_t,
                                        uint32_t*, CommandMessage*, object_id_t objectId,
                                        bool* isStep) {
  const auto cmd = reply->getCommand();
  sif::info << "RwPusService::handleReply cmd=0x" << std::hex << int(cmd) << std::dec << std::endl;

  sif::warning << "[PUS220] cmd=0x" << std::hex << int(cmd)
               << " DIRECT=0x" << int(DeviceHandlerMessage::REPLY_DIRECT_COMMAND_DATA)
               << " RAW=0x"    << int(DeviceHandlerMessage::REPLY_RAW_REPLY)
               << " DATA_REPLY=0x" << int(ActionMessage::DATA_REPLY)
               << std::dec << std::endl;

  // Determine correct store address
  store_address_t sid{StorageManagerIF::INVALID_ADDRESS};
  if (cmd == ActionMessage::DATA_REPLY) {
    sif::warning << "[PUS220] got DATA_REPLY from DH -> will read IPC/TM/TC store" << std::endl;
    sid = ActionMessage::getStoreId(reply);
    sif::warning << "[PUS220] DATA_REPLY: storeId=0x" << std::hex << sid.raw << std::dec
                 << std::endl;
  } else {
    sid = DeviceHandlerMessage::getStoreAddress(reply);
    if (sid.raw != StorageManagerIF::INVALID_ADDRESS) {
      sif::warning << "[PUS220] DeviceHandler storeId=0x" << std::hex << sid.raw << std::dec
                   << std::endl;
    }
  }

  // 1) If a store address is present, try stores
  if (sid.raw != StorageManagerIF::INVALID_ADDRESS) {
    StorageManagerIF* stores[3] = {ipcStore, tmStore, tcStore};

    const uint8_t* buf = nullptr;
    size_t len = 0;
    StorageManagerIF* usedStore = nullptr;

    for (int i = 0; i < 3; ++i) {
      if (stores[i] == nullptr) continue;
      const uint8_t* b = nullptr;
      size_t l = 0;
      auto rv = stores[i]->getData(sid, &b, &l);
      sif::warning << "[PUS220 generic] try "
                   << (i == 0 ? "IPC" : (i == 1 ? "TM" : "TC"))
                   << " rv=" << rv << " len=" << l << std::endl;
      if (rv == returnvalue::OK && b != nullptr && l > 0) {
        buf = b; len = l; usedStore = stores[i];
        break;
      }
    }

    if (buf != nullptr && len > 0) {
      dumpHexWarn("[PUS220 generic] head", buf, (len < 32 ? len : size_t(32)));

      const int idx = findStatusFrame(buf, len);
      sif::warning << "[PUS220 generic] findStatusFrame idx=" << idx
                   << " (len=" << len << ")" << std::endl;

      ReturnValue_t rv = returnvalue::OK;  // do not escalate if frame missing
      if (idx >= 0) {
        const uint8_t* p = buf + idx;
        const int16_t speed   = static_cast<int16_t>((p[2] << 8) | p[3]);
        const int16_t torque  = static_cast<int16_t>((p[4] << 8) | p[5]);
        const uint8_t running = p[6];

        if (running > 1) {
          sif::warning << "[PUS220 generic] 'running' flag suspicious: " << int(running)
                       << " (expected 0/1)" << std::endl;
        }

        // AppData: [object_id(4) | speed(2) | torque(2) | running(1)]
        uint8_t app[4 + 2 + 2 + 1];
        app[0] = static_cast<uint8_t>((objectId >> 24) & 0xFF);
        app[1] = static_cast<uint8_t>((objectId >> 16) & 0xFF);
        app[2] = static_cast<uint8_t>((objectId >>  8) & 0xFF);
        app[3] = static_cast<uint8_t>((objectId      ) & 0xFF);
        app[4] = static_cast<uint8_t>((speed  >> 8) & 0xFF);
        app[5] = static_cast<uint8_t>( speed        & 0xFF);
        app[6] = static_cast<uint8_t>((torque >> 8) & 0xFF);
        app[7] = static_cast<uint8_t>( torque       & 0xFF);
        app[8] = running;

        const auto prv = tmHelper.prepareTmPacket(static_cast<uint8_t>(Subservice::TM_STATUS),
                                                  app, sizeof(app));
        sif::warning << "[PUS220 generic] prepareTmPacket rv=" << prv << std::endl;
        if (prv == returnvalue::OK) {
          rv = tmHelper.storeAndSendTmPacket();
          sif::warning << "[PUS220 generic] storeAndSend rv=" << rv << std::endl;
        } else {
          rv = prv;
        }
      } else {
        sif::warning << "[PUS220 generic] no AB 10 .. .. .. .. .. CRC frame in payload" << std::endl;
      }

      if (usedStore != nullptr) {
        auto delRv = usedStore->deleteData(sid);
        if (delRv != returnvalue::OK) {
          sif::warning << "[PUS220 generic] deleteData sid=0x" << std::hex << sid.raw
                       << std::dec << " failed rv=" << delRv << std::endl;
        }
      }

      *isStep = true;
      return rv;
    }

    // Had a store id but nothing usable
    sif::warning << "[PUS220 generic] sid=0x" << std::hex << sid.raw
                 << std::dec << " has no usable payload -> ignore" << std::endl;
    *isStep = true;
    return returnvalue::OK;
  } else {
    // Visibility: DATA_REPLY without valid store id is unexpected
    if (cmd == ActionMessage::DATA_REPLY) {
      sif::warning << "[PUS220] DATA_REPLY without valid store id (sid invalid)" << std::endl;
    }
  }

  // 2) Control-flow replies without data
  switch (cmd) {
    case ActionMessage::STEP_SUCCESS:
      sif::warning << "[PUS220] STEP_SUCCESS" << std::endl;
      *isStep = true;
      return returnvalue::OK;

    case ActionMessage::STEP_FAILED:
      sif::warning << "[PUS220] STEP_FAILED rv=" << ActionMessage::getReturnCode(reply) << std::endl;
      *isStep = true;
      return ActionMessage::getReturnCode(reply);

    case ActionMessage::COMPLETION_SUCCESS:
      sif::warning << "[PUS220] COMPLETION_SUCCESS" << std::endl;
      return CommandingServiceBase::EXECUTION_COMPLETE;

    case ActionMessage::COMPLETION_FAILED:
      sif::warning << "[PUS220] COMPLETION_FAILED rv=" << ActionMessage::getReturnCode(reply) << std::endl;
      return ActionMessage::getReturnCode(reply);

    case DeviceHandlerMessage::REPLY_DIRECT_COMMAND_DATA:
    case DeviceHandlerMessage::REPLY_RAW_REPLY:
      sif::warning << "[PUS220 fallback] got DIRECT/RAW without store id; ignoring" << std::endl;
      return returnvalue::OK;

    default:
      sif::warning << "PUS220 handleReply: unhandled cmd=0x"
                   << std::hex << int(cmd) << std::dec << std::endl;
      return returnvalue::OK;
  }
}
