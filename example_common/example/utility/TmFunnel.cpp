#include "TmFunnel.h"

#include <fsfw/ipc/QueueFactory.h>

TmFunnel::TmFunnel(object_id_t objectId, PusTmFunnel& pusFunnel, CfdpTmFunnel& cfdpFunnel)
    : SystemObject(objectId), pusFunnel(pusFunnel), cfdpFunnel(cfdpFunnel) {}

TmFunnel::~TmFunnel() = default;

ReturnValue_t TmFunnel::performOperation(uint8_t operationCode) {
  pusFunnel.performOperation(operationCode);
  cfdpFunnel.performOperation(operationCode);
  return returnvalue::OK;
}

ReturnValue_t TmFunnel::initialize() { return returnvalue::OK; }
