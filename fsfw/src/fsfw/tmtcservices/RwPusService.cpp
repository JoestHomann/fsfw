#include "RwPusService.h"

#include <cstring>
#include "fsfw/devicehandlers/DeviceHandlerIF.h"
#include "fsfw/devicehandlers/DeviceHandlerMessage.h"
#include "fsfw/modes/ModeMessage.h"
#include "fsfw/objectmanager/ObjectManager.h"
#include "fsfw/serviceinterface/ServiceInterfaceStream.h"
#include "fsfw/storagemanager/StorageManagerIF.h"
#include "fsfw/action/ActionMessage.h"

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
  sif::info << "RwPusService: TC subservice=" << int(subservice)
            << " len=" << tcLen << std::endl;

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
      // AppData carries RPM big-endian; store native int16_t for the device handler
      int16_t rpm = static_cast<int16_t>((app[0] << 8) | app[1]);

      store_address_t sid = store_address_t::invalid();
      uint8_t* p = nullptr;
      auto res = ipcStore->getFreeElement(&sid, sizeof(rpm), &p);
      if (res != returnvalue::OK) return res;
      std::memcpy(p, &rpm, sizeof(rpm));

      ActionMessage::setCommand(message, 0x01 /*CMD_SET_SPEED*/, sid);
      return returnvalue::OK;
    }
    case Subservice::STOP: {
      ActionMessage::setCommand(message, 0x02 /*CMD_STOP*/, store_address_t::invalid());
      return returnvalue::OK;
    }
    case Subservice::STATUS: {
      ActionMessage::setCommand(message, 0x03 /*CMD_STATUS*/, store_address_t::invalid());
      return returnvalue::OK;
    }
    case Subservice::SET_MODE: {
      if (appLen < 2) return CommandingServiceBase::INVALID_TC;
      const uint8_t mode    = app[0];
      const uint8_t submode = app[1];
      sif::info << "RwPusService: SET_MODE req -> mode="
            << int(mode) << ", sub=" << int(submode) << std::endl;
      ModeMessage::setModeMessage(message, ModeMessage::CMD_MODE_COMMAND, mode, submode);
      return returnvalue::OK;
    }
    default:
      return CommandingServiceBase::INVALID_TC;
  }
}

ReturnValue_t RwPusService::handleReply(const CommandMessage* reply, Command_t,
                                        uint32_t*, CommandMessage*, object_id_t, bool*) {
  switch (reply->getCommand()) {
    case DeviceHandlerMessage::REPLY_RAW_COMMAND:
    case DeviceHandlerMessage::REPLY_RAW_REPLY:
      // Acknowledge; full TM generation is out-of-scope here.
      return CommandingServiceBase::EXECUTION_COMPLETE;
    default:
      return CommandingServiceBase::INVALID_REPLY;
  }
}
