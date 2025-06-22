#ifndef FSFW_EXAMPLE_COMMON_CFDPTMFUNNEL_H
#define FSFW_EXAMPLE_COMMON_CFDPTMFUNNEL_H

#include "fsfw/objectmanager/SystemObject.h"
#include "fsfw/storagemanager/StorageManagerIF.h"
#include "fsfw/tmtcservices/AcceptsTelemetryIF.h"
#include "fsfw/tmtcservices/TmTcMessage.h"

class CfdpTmFunnel : public AcceptsTelemetryIF, public SystemObject {
 public:
  CfdpTmFunnel(object_id_t objectId, uint16_t cfdpInCcsdsApid,
               const AcceptsTelemetryIF& downlinkDestination, StorageManagerIF& tmStore);
  [[nodiscard]] const char* getName() const override;
  [[nodiscard]] MessageQueueId_t getReportReceptionQueue(uint8_t virtualChannel) const override;

  ReturnValue_t performOperation(uint8_t opCode);
  ReturnValue_t initialize() override;

 private:
  ReturnValue_t handlePacket(TmTcMessage& msg);

  uint16_t sourceSequenceCount = 0;
  uint16_t cfdpInCcsdsApid;
  MessageQueueIF* msgQueue;
  StorageManagerIF& tmStore;
};
#endif  // FSFW_EXAMPLE_COMMON_CFDPTMFUNNEL_H
