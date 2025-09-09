#include "RwPusService.h"

#include <cstring>
#include "fsfw/action/ActionMessage.h"
#include "fsfw/devicehandlers/DeviceHandlerIF.h"
#include "fsfw/devicehandlers/DeviceHandlerMessage.h"
#include "fsfw/modes/ModeMessage.h"
#include "fsfw/objectmanager/ObjectManager.h"
#include "fsfw/serviceinterface/ServiceInterfaceStream.h"
#include "fsfw/storagemanager/StorageManagerIF.h"

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
  // Validate only TC subservices. TM subservices (like TM_STATUS) must NOT be listed here.
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
  // Debug trace
  sif::info << "RwPusService: TC subservice=" << int(subservice) << " len=" << tcLen << std::endl;

  if (ipcStore == nullptr) {
    return returnvalue::FAILED;
  }
  if (tcLen < 4) {
    return CommandingServiceBase::INVALID_TC;
  }
  const uint8_t* app  = tcData + 4;
  const size_t   appLen = tcLen - 4;

  switch (static_cast<Subservice>(subservice)) {
    case Subservice::SET_SPEED: {
      if (appLen < 2) {
        return CommandingServiceBase::INVALID_TC;
      }
      // Big-endian RPM from TC -> native int16 for device handler
      int16_t rpm = static_cast<int16_t>((app[0] << 8) | app[1]);

      store_address_t sid = store_address_t::invalid();
      uint8_t* p = nullptr;
      auto res = ipcStore->getFreeElement(&sid, sizeof(rpm), &p);
      if (res != returnvalue::OK) {
        return res;
      }
      std::memcpy(p, &rpm, sizeof(rpm));

      // Send as ACTION to the device handler; this triggers buildCommandFromCommand()
      ActionMessage::setCommand(message, 0x01 /* CMD_SET_SPEED */, sid);
      return returnvalue::OK;
    }

    case Subservice::STOP: {
      // No parameters
      ActionMessage::setCommand(message, 0x02 /* CMD_STOP */, store_address_t::invalid());
      return returnvalue::OK;
    }

    case Subservice::STATUS: {
      // No parameters
      ActionMessage::setCommand(message, 0x03 /* CMD_STATUS */, store_address_t::invalid());
      return returnvalue::OK;
    }

    case Subservice::SET_MODE: {
      if (appLen < 2) {
        return CommandingServiceBase::INVALID_TC;
      }
      const uint8_t mode    = app[0];
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

ReturnValue_t RwPusService::handleReply(const CommandMessage* reply, Command_t,
                                        uint32_t*, CommandMessage*, object_id_t objectId, bool*) {
  switch (reply->getCommand()) {
    // Direct-command reply with payload stored in IPC (preferred path for STATUS)
    case DeviceHandlerMessage::REPLY_DIRECT_COMMAND_DATA:
    // Raw reply path (kept for completeness if RAW is ever used)
    case DeviceHandlerMessage::REPLY_RAW_REPLY: {
      const store_address_t sid = DeviceHandlerMessage::getStoreAddress(reply);

      const uint8_t* p = nullptr;
      size_t len = 0;
      if (ipcStore == nullptr || ipcStore->getData(sid, &p, &len) != returnvalue::OK) {
        return CommandingServiceBase::INVALID_REPLY;
      }

      // Expect exactly 8 bytes: AB 0x10 <spdH spdL> <torH torL> <running> <crc>
      if (len >= 8 && p[0] == 0xAB && p[1] == 0x10) {
        const int16_t speed   = static_cast<int16_t>((p[2] << 8) | p[3]);
        const int16_t torque  = static_cast<int16_t>((p[4] << 8) | p[5]);
        const uint8_t running = p[6];

        // Build compact AppData: [object_id(4) | speed(2) | torque(2) | running(1)]
        uint8_t app[4 + 2 + 2 + 1];
        app[0] = static_cast<uint8_t>((objectId >> 24) & 0xFF);
        app[1] = static_cast<uint8_t>((objectId >> 16) & 0xFF);
        app[2] = static_cast<uint8_t>((objectId >> 8)  & 0xFF);
        app[3] = static_cast<uint8_t>( objectId        & 0xFF);
        app[4] = static_cast<uint8_t>((speed  >> 8) & 0xFF);
        app[5] = static_cast<uint8_t>( speed        & 0xFF);
        app[6] = static_cast<uint8_t>((torque >> 8) & 0xFF);
        app[7] = static_cast<uint8_t>( torque       & 0xFF);
        app[8] = running;

        // Emit TM packet: service already 220 by base, subservice = TM_STATUS (e.g. 130)
        ReturnValue_t tmRes = tmHelper.prepareTmPacket(
            static_cast<uint8_t>(Subservice::TM_STATUS), app, sizeof(app));
        if (tmRes == returnvalue::OK) {
          tmHelper.storeAndSendTmPacket();
        }
      }

      // Always release the IPC element before returning
      ipcStore->deleteData(sid);
      return CommandingServiceBase::EXECUTION_COMPLETE;
    }

    // No data to forward; acknowledge completion
    case DeviceHandlerMessage::REPLY_RAW_COMMAND:
      return CommandingServiceBase::EXECUTION_COMPLETE;

    default:
      return CommandingServiceBase::INVALID_REPLY;
  }
}
