#include "RwPusService.h"

#include <cstring>
#include "fsfw/action/ActionMessage.h"
#include "fsfw/devicehandlers/DeviceHandlerIF.h"
#include "fsfw/devicehandlers/DeviceHandlerMessage.h"
#include "fsfw/modes/ModeMessage.h"
#include "fsfw/objectmanager/ObjectManager.h"
#include "fsfw/serviceinterface/ServiceInterfaceStream.h"
#include "fsfw/storagemanager/StorageManagerIF.h"
#include "fsfw/storagemanager/storeAddress.h"
#include "fsfw/ipc/MessageQueueIF.h"
#include "fsfw/ipc/FwMessageTypes.h"
#include "commonObjects.h"

// ---- Local copy of the device command IDs to avoid cross-includes ----
// Keep these values in sync with RwCommanderHandler.
namespace rwcmd {
static constexpr DeviceCommandId_t CMD_SET_SPEED = 0x01;
static constexpr DeviceCommandId_t CMD_STOP      = 0x02;
static constexpr DeviceCommandId_t CMD_STATUS    = 0x03;
}
// ---------------------------------------------------------------------

// Small helpers for big-endian decoding (keep local to this TU)
namespace {
inline uint32_t be32(const uint8_t* p) {
  return (static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16) |
         (static_cast<uint32_t>(p[2]) << 8) | static_cast<uint32_t>(p[3]);
}
}  // namespace

RwPusService::RwPusService(object_id_t objectId, uint16_t apid, uint8_t serviceId,
                           uint8_t numParallelCommands, uint16_t commandTimeoutSeconds)
    // FSFW newer API: needs name, queue depth and (optional) verification reporter
    : CommandingServiceBase(
          objectId, apid, "RwPusService",            // service name (for debugging)
          serviceId, numParallelCommands, commandTimeoutSeconds,
          20 /*queueDepth*/, nullptr /*VerificationReporterIF*/) {}

ReturnValue_t RwPusService::initialize() {
  ReturnValue_t result = CommandingServiceBase::initialize();
  if (result != returnvalue::OK) {
    return result;
  }
  ipcStore = ObjectManager::instance()->get<StorageManagerIF>(objects::IPC_STORE);
  return ipcStore != nullptr ? returnvalue::OK : returnvalue::FAILED;
}

ReturnValue_t RwPusService::isValidSubservice(uint8_t subservice) {
  switch (subservice) {
    case Subservice::SET_SPEED:
    case Subservice::STOP:
    case Subservice::STATUS:
    case Subservice::SET_MODE:
      return returnvalue::OK;
    default:
      return CommandingServiceBase::INVALID_SUBSERVICE;
  }
}

ReturnValue_t RwPusService::getMessageQueueAndObject(uint8_t /*subservice*/,
                                                     const uint8_t* tcData, size_t tcLen,
                                                     MessageQueueId_t* id,
                                                     object_id_t* objectId) {
  if (tcLen < 4) {
    return CommandingServiceBase::INVALID_TC;
  }

  // AppData starts with destination object ID in big-endian
  *objectId = static_cast<object_id_t>((tcData[0] << 24) | (tcData[1] << 16) |
                                       (tcData[2] << 8) | tcData[3]);

  // Look up the targeted device handler and return its command queue
  DeviceHandlerIF* dh = ObjectManager::instance()->get<DeviceHandlerIF>(*objectId);
  if (dh == nullptr) {
    // Target object not found in the ObjectManager
    return CommandingServiceBase::INVALID_OBJECT;
  }

  // This is the queue where the TC will be sent
  *id = dh->getCommandQueue();
  return returnvalue::OK;
}

ReturnValue_t RwPusService::prepareCommand(CommandMessage* message, uint8_t subservice,
                                           const uint8_t* tcData, size_t tcLen, uint32_t*,
                                           object_id_t) {
  // AppData: [object_id(4) | payload...]
  const uint8_t* payload = tcData + 4;
  const size_t payloadLen = (tcLen >= 4) ? (tcLen - 4) : 0;

  switch (subservice) {
    case Subservice::SET_SPEED: {
      // Expect 2 bytes RPM in app data; forward payload to DH as ACTION
      if (payloadLen != 2) {
        return CommandingServiceBase::INVALID_TC;
      }
      store_address_t addr;
      if (ipcStore->addData(&addr, payload, 2) != returnvalue::OK) {
        return CommandingServiceBase::INVALID_TC;
      }
      // Send as Action so the DeviceHandler's executeAction is called
      ActionMessage::setCommand(message, rwcmd::CMD_SET_SPEED, addr);
      return returnvalue::OK;
    }
    case Subservice::STOP: {
      // Action without payload
      store_address_t invalid;
      invalid.raw = store_address_t::INVALID_RAW;
      ActionMessage::setCommand(message, rwcmd::CMD_STOP, invalid);
      return returnvalue::OK;
    }
    case Subservice::STATUS: {
      // Action without payload → DH builds AA 03 00 00 CRC
      store_address_t invalid;
      invalid.raw = store_address_t::INVALID_RAW;
      ActionMessage::setCommand(message, rwcmd::CMD_STATUS, invalid);
      return returnvalue::OK;
    }
    case Subservice::SET_MODE: {
      if (payloadLen != 2) {
        return CommandingServiceBase::INVALID_TC;
      }
      // Command constant belongs to ModeMessage
      ModeMessage::setModeMessage(message, ModeMessage::CMD_MODE_COMMAND, payload[0], payload[1]);
      return returnvalue::OK;
    }
    default:
      return CommandingServiceBase::INVALID_SUBSERVICE;
  }
}

ReturnValue_t RwPusService::handleReply(const CommandMessage* reply, Command_t,
                                        uint32_t*, CommandMessage*, object_id_t objectId, bool*) {
  // Ignore mode acks/info so we don't spam logs (cmd=0x102 etc.)
  if (reply->getMessageType() == messagetypes::MODE_COMMAND) {
    return returnvalue::OK;
  }

  // Trace incoming replies
  sif::info << "RwPusService::handleReply cmd=0x"
            << std::hex << int(reply->getCommand()) << std::dec << std::endl;

  // 1) Generic acks from CommandMessage
  switch (reply->getCommand()) {
    case CommandMessage::REPLY_COMMAND_OK:
      return returnvalue::OK;
    case CommandMessage::REPLY_REJECTED:
      return reply->getReplyRejectedReason();
    default:
      break; // fall through to device-specific handling below
  }

  // 2) Known device handler raw-data replies (preferred fast paths)
  switch (reply->getCommand()) {
    case DeviceHandlerMessage::REPLY_DIRECT_COMMAND_DATA:
    case DeviceHandlerMessage::REPLY_RAW_REPLY: {
      const store_address_t sid = DeviceHandlerMessage::getStoreAddress(reply);
      const uint8_t* p = nullptr;
      size_t len = 0;
      if (ipcStore == nullptr || ipcStore->getData(sid, &p, &len) != returnvalue::OK) {
        return CommandingServiceBase::INVALID_REPLY;
      }

      // Expect 8B: AB <id> <spdH spdL> <torH torL> <running> <crc>
      if (len < 8 || p[0] != 0xAB) {
        ipcStore->deleteData(sid);
        return CommandingServiceBase::INVALID_REPLY;
      }

      const int16_t speed   = static_cast<int16_t>((p[2] << 8) | p[3]);
      const int16_t torque  = static_cast<int16_t>((p[4] << 8) | p[5]);
      const uint8_t running = p[6];

      // Build TM 220/130
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

      if (tmHelper.prepareTmPacket(static_cast<uint8_t>(Subservice::TM_STATUS), app, sizeof(app))
          == returnvalue::OK) {
        sif::info << "RwPusService: sending TM_STATUS (svc=220, sub=130)"
                  << " speed=" << speed << " rpm, torque=" << torque
                  << " mNm, running=" << int(running) << std::endl;
        tmHelper.storeAndSendTmPacket();
      } else {
        sif::error << "RwPusService: failed to prepare TM_STATUS" << std::endl;
      }

      ipcStore->deleteData(sid);
      return CommandingServiceBase::EXECUTION_COMPLETE;
    }

    case DeviceHandlerMessage::REPLY_RAW_COMMAND:
      // No TM needed for raw command echo; just acknowledge completion.
      return CommandingServiceBase::EXECUTION_COMPLETE;

    default:
      break; // try robust fallback below
  }

  // 3) Robust fallback: any device handler message carrying store data
  if (reply->getMessageType() == messagetypes::DEVICE_HANDLER_COMMAND) {
    const store_address_t sid = DeviceHandlerMessage::getStoreAddress(reply);
    if (sid.raw != store_address_t::INVALID_RAW) {
      const uint8_t* p = nullptr;
      size_t len = 0;
      if (ipcStore && ipcStore->getData(sid, &p, &len) == returnvalue::OK) {
        if (len >= 8 && p[0] == 0xAB) {
          const int16_t speed   = static_cast<int16_t>((p[2] << 8) | p[3]);
          const int16_t torque  = static_cast<int16_t>((p[4] << 8) | p[5]);
          const uint8_t running = p[6];

          uint8_t app[9];
          app[0] = static_cast<uint8_t>((objectId >> 24) & 0xFF);
          app[1] = static_cast<uint8_t>((objectId >> 16) & 0xFF);
          app[2] = static_cast<uint8_t>((objectId >> 8)  & 0xFF);
          app[3] = static_cast<uint8_t>( objectId        & 0xFF);
          app[4] = static_cast<uint8_t>((speed  >> 8) & 0xFF);
          app[5] = static_cast<uint8_t>( speed        & 0xFF);
          app[6] = static_cast<uint8_t>((torque >> 8) & 0xFF);
          app[7] = static_cast<uint8_t>( torque       & 0xFF);
          app[8] = running;

          if (tmHelper.prepareTmPacket(static_cast<uint8_t>(Subservice::TM_STATUS), app, sizeof(app))
              == returnvalue::OK) {
            tmHelper.storeAndSendTmPacket();
          }
          ipcStore->deleteData(sid);
          return CommandingServiceBase::EXECUTION_COMPLETE;
        }
        ipcStore->deleteData(sid);
      }
    }
  }

  // 4) Still here? Then it's something we don't handle.
  sif::error << "RwPusService: unhandled reply cmd=0x"
             << std::hex << int(reply->getCommand()) << std::dec << std::endl;
  return CommandingServiceBase::INVALID_REPLY;
}
