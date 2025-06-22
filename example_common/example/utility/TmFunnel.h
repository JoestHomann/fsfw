#ifndef MISSION_UTILITY_TMFUNNEL_H_
#define MISSION_UTILITY_TMFUNNEL_H_

#include <fsfw/ipc/MessageQueueIF.h>
#include <fsfw/objectmanager/SystemObject.h>
#include <fsfw/tasks/ExecutableObjectIF.h>
#include <fsfw/tmtcservices/AcceptsTelemetryIF.h>
#include <fsfw/tmtcservices/TmTcMessage.h>

#include "CfdpTmFunnel.h"
#include "PusTmFunnel.h"
#include "fsfw/timemanager/TimeReaderIF.h"

/**
 * @brief       TM Recipient.
 * @details
 * Main telemetry receiver. All generated telemetry is funneled into
 * this object.
 * @ingroup     utility
 * @author      J. Meier
 */
class TmFunnel : public ExecutableObjectIF, public SystemObject {
 public:
  TmFunnel(object_id_t objectId, PusTmFunnel& pusFunnel, CfdpTmFunnel& cfdpFunnel);
  ~TmFunnel() override;

  ReturnValue_t performOperation(uint8_t operationCode) override;
  ReturnValue_t initialize() override;

 private:
  PusTmFunnel& pusFunnel;
  CfdpTmFunnel& cfdpFunnel;
};

#endif /* MISSION_UTILITY_TMFUNNEL_H_ */
