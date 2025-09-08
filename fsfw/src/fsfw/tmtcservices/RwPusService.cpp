#include "RwPusService.h"

#include "fsfw/devicehandlers/DeviceHandlerIF.h"
#include "fsfw/devicehandlers/DeviceHandlerMessage.h"
#include "fsfw/objectmanager/ObjectManager.h"
#include "fsfw/serviceinterface/ServiceInterfaceStream.h"
#include "fsfw/storagemanager/StorageManagerIF.h"

namespace {
constexpr uint8_t START_BYTE_CMD   = 0xAA;
constexpr uint8_t CMD_SET_SPEED    = 0x01;
constexpr uint8_t CMD_STOP         = 0x02;
constexpr uint8_t CMD_STATUS       = 0x03;
}

RwPusService::RwPusService(object_id_t objectId, uint16_t apid, uint8_t serviceId,
                           uint8_t numParallelCommands, uint16_t commandTimeoutSeconds)
    : CommandingServiceBase(objectId, apid, "PUS 220 RW CMD", serviceId, numParallelCommands,
                            commandTimeoutSeconds) {}

ReturnValue_t RwPusService::initialize() {
  auto result = CommandingServiceBase::initialize();
  if (result != returnvalue::OK) {
    return result;
  }
  // Acquire IPC store (used to pass raw buffer to device handlers)
  ipcStore = ObjectManager::instance()->get<StorageManagerIF>(objects::IPC_STORE);
  if (ipcStore == nullptr) {
    sif::warning << "RwPusService: IPC_STORE not available!" << std::endl;
    return returnvalue::FAILED;
  }
  return returnvalue::OK;
}

ReturnValue_t RwPusService::isValidSubservice(uint8_t subservice) {
  switch (static_cast<Subservice>(subservice)) {
    case Subservice::SET_SPEED:
    case Subservice::STOP:
    case Subservice::STATUS:
      return returnvalue::OK;
    default:
      return AcceptsTelecommandsIF::INVALID_SUBSERVICE;
  }
}

ReturnValue_t RwPusService::getMessageQueueAndObject(uint8_t /*subservice*/, const uint8_t* tcData,
                                                     size_t tcDataLen, MessageQueueId_t* id,
                                                     object_id_t* objectId) {
  // First 4 bytes are the destination object ID in BE
  if (tcDataLen < 4) {
    return CommandingServiceBase::INVALID_TC;
  }
  *objectId = static_cast<object_id_t>(be32ToU32(tcData));
  // Look up the device handler
  auto* dh = ObjectManager::instance()->get<DeviceHandlerIF>(*objectId);
  if (dh == nullptr) {
    return CommandingServiceBase::INVALID_OBJECT;
  }
  *id = dh->getCommandQueue();
  return returnvalue::OK;
}

ReturnValue_t RwPusService::prepareCommand(CommandMessage* message, uint8_t subservice,
                                           const uint8_t* tcData, size_t tcDataLen,
                                           uint32_t* /*state*/, object_id_t /*objectId*/) {
  // Debug: show incoming TC meta
  sif::info << "RwPusService: TC subservice=" << static_cast<int>(subservice)
            << " len=" << tcDataLen << std::endl;

  if (ipcStore == nullptr) {
    return returnvalue::FAILED;
  }

  // AppData: first 4 bytes = target object_id (BE), remaining = payload
  if (tcDataLen < 4) {
    return CommandingServiceBase::INVALID_TC;
  }
  const uint8_t* app = tcData + 4;
  size_t appLen = tcDataLen - 4;

  uint8_t frame[5] = {START_BYTE_CMD, 0, 0, 0, 0};

  switch (static_cast<Subservice>(subservice)) {
    case Subservice::SET_SPEED: {
      if (appLen < 2) {
        return CommandingServiceBase::INVALID_TC;
      }
      const int16_t rpm = be16ToI16(app);            // signed BE -> host
      const uint16_t u = static_cast<uint16_t>(rpm); // send as 16-bit two's complement
      frame[1] = CMD_SET_SPEED;
      frame[2] = static_cast<uint8_t>((u >> 8) & 0xFF);
      frame[3] = static_cast<uint8_t>(u & 0xFF);
      frame[4] = crc8(frame, 4);
      break;
    }
    case Subservice::STOP: {
      frame[1] = CMD_STOP;
      frame[2] = 0x00;
      frame[3] = 0x00;
      frame[4] = crc8(frame, 4);
      break;
    }
    case Subservice::STATUS: {
      frame[1] = CMD_STATUS;
      frame[2] = 0x00;
      frame[3] = 0x00;
      frame[4] = crc8(frame, 4);
      break;
    }
    default:
      return CommandingServiceBase::INVALID_TC;
  }

  // Store the 5-byte raw frame in IPC store
  store_address_t addr;
  ReturnValue_t res = ipcStore->addData(&addr, frame, sizeof(frame));
  if (res != returnvalue::OK) {
    sif::warning << "RwPusService: ipcStore->addData failed: " << static_cast<int>(res) << std::endl;
    return res;
  }

  // Create RAW command message for the targeted device handler
  DeviceHandlerMessage::setDeviceHandlerRawCommandMessage(message, addr);

  // Optional debug: print the frame we are forwarding
  sif::info << "RwPusService: forwarding RAW frame ["
            << "0x" << std::hex << static_cast<int>(frame[0]) << " "
            << "0x" << static_cast<int>(frame[1]) << " "
            << "0x" << static_cast<int>(frame[2]) << " "
            << "0x" << static_cast<int>(frame[3]) << " "
            << "0x" << static_cast<int>(frame[4]) << std::dec << "]" << std::endl;

  // We’re done after one step (fire-and-forget)
  return CommandingServiceBase::EXECUTION_COMPLETE;
}


ReturnValue_t RwPusService::handleReply(const CommandMessage* reply, Command_t /*previousCommand*/,
                                        uint32_t* /*state*/, CommandMessage* /*optionalNextCommand*/,
                                        object_id_t /*objectId*/, bool* /*isStep*/) {
  // Optional: Inspect raw replies for logging. Full TM generation is typically handled by PUS-2.
  switch (reply->getCommand()) {
    case DeviceHandlerMessage::REPLY_RAW_COMMAND:
    case DeviceHandlerMessage::REPLY_RAW_REPLY: {
      // You could fetch the raw bytes from IPC store and log them here.
      // We keep it simple and just acknowledge.
      return CommandingServiceBase::EXECUTION_COMPLETE;
    }
    default:
      return CommandingServiceBase::INVALID_REPLY;
  }
}

uint8_t RwPusService::crc8(const uint8_t* data, size_t len) {
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
