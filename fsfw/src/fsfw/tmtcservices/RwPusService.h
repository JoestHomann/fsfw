#pragma once
#include "fsfw/tmtcservices/CommandingServiceBase.h"
#include "fsfw/storagemanager/StorageManagerIF.h"

/**
 * PUS custom service to command the Reaction Wheel in its native protocol.
 *
 * Subservices:
 *   1  SET_SPEED  (AppData: [object_id:u32 BE][rpm:int16 BE])
 *   2  STOP       (AppData: [object_id:u32 BE])
 *   3  STATUS     (AppData: [object_id:u32 BE])
 *
 * It assembles 5-byte raw frames 0xAA <CMD> <valH> <valL> <CRC8> and forwards them
 * using DeviceHandlerMessage::CMD_RAW. The targeted device handler MUST be in MODE_RAW.
 *
 * No TM is generated here; replies are printed to the log if received as raw replies.
 * (For full wiretapping TM, prefer FSFW PUS Service 2.)
 */
class RwPusService : public CommandingServiceBase {
 public:
  enum : uint8_t { SERVICE_ID = 220 };  // pick a free service ID

  enum class Subservice : uint8_t {
    SET_SPEED = 1,
    STOP      = 2,
    STATUS    = 3,
  };

  RwPusService(object_id_t objectId, uint16_t apid, uint8_t serviceId = SERVICE_ID,
               uint8_t numParallelCommands = 4, uint16_t commandTimeoutSeconds = 10);

 protected:
  // CSB hooks
  ReturnValue_t isValidSubservice(uint8_t subservice) override;
  ReturnValue_t getMessageQueueAndObject(uint8_t subservice, const uint8_t* tcData,
                                         size_t tcDataLen, MessageQueueId_t* id,
                                         object_id_t* objectId) override;
  ReturnValue_t prepareCommand(CommandMessage* message, uint8_t subservice, const uint8_t* tcData,
                               size_t tcDataLen, uint32_t* state, object_id_t objectId) override;
  ReturnValue_t handleReply(const CommandMessage* reply, Command_t previousCommand, uint32_t* state,
                            CommandMessage* optionalNextCommand, object_id_t objectId,
                            bool* isStep) override;

  // ExecutableObjectIF
  ReturnValue_t initialize() override;

 private:
  StorageManagerIF* ipcStore = nullptr;

  static uint8_t crc8(const uint8_t* data, size_t len);
  static int16_t be16ToI16(const uint8_t* p) { return static_cast<int16_t>((p[0] << 8) | p[1]); }
  static uint32_t be32ToU32(const uint8_t* p) {
    return (static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16) |
           (static_cast<uint32_t>(p[2]) << 8) | static_cast<uint32_t>(p[3]);
  }
};
