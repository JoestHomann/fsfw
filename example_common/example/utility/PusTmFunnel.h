#ifndef FSFW_EXAMPLE_COMMON_PUSTMFUNNEL_H
#define FSFW_EXAMPLE_COMMON_PUSTMFUNNEL_H

#include <fsfw/ipc/MessageQueueIF.h>
#include <fsfw/objectmanager/SystemObject.h>
#include <fsfw/tasks/ExecutableObjectIF.h>
#include <fsfw/tmtcservices/AcceptsTelemetryIF.h>
#include <fsfw/tmtcservices/TmTcMessage.h>

#include "fsfw/timemanager/TimeReaderIF.h"

/**
 * @brief       TM Recipient.
 * @details
 * Main telemetry receiver. All generated telemetry is funneled into
 * this object.
 * @ingroup     utility
 * @author      J. Meier
 */
class PusTmFunnel : public AcceptsTelemetryIF, public SystemObject {
 public:
  explicit PusTmFunnel(object_id_t objectId, const AcceptsTelemetryIF &downlinkDestination,
                       TimeReaderIF &timeReader, StorageManagerIF &tmStore,
                       uint32_t messageDepth = 20);
  [[nodiscard]] const char *getName() const override;
  ~PusTmFunnel() override;

  [[nodiscard]] MessageQueueId_t getReportReceptionQueue(uint8_t virtualChannel) const override;
  ReturnValue_t performOperation(uint8_t operationCode);

 private:
  uint16_t sourceSequenceCount = 0;
  TimeReaderIF &timeReader;
  StorageManagerIF &tmStore;
  MessageQueueIF *tmQueue = nullptr;
  ReturnValue_t handlePacket(TmTcMessage &message);
};

#endif  // FSFW_EXAMPLE_COMMON_PUSTMFUNNEL_H
