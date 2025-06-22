#include "PusTmFunnel.h"

#include "fsfw/ipc/QueueFactory.h"
#include "fsfw/objectmanager.h"
#include "fsfw/tmtcpacket/pus/tm/PusTmZcWriter.h"

PusTmFunnel::PusTmFunnel(object_id_t objectId, const AcceptsTelemetryIF &downlinkDestination,
                         TimeReaderIF &timeReader, StorageManagerIF &tmStore, uint32_t messageDepth)
    : SystemObject(objectId), timeReader(timeReader), tmStore(tmStore) {
  tmQueue = QueueFactory::instance()->createMessageQueue(messageDepth,
                                                         MessageQueueMessage::MAX_MESSAGE_SIZE);
  tmQueue->setDefaultDestination(downlinkDestination.getReportReceptionQueue());
}

PusTmFunnel::~PusTmFunnel() = default;

MessageQueueId_t PusTmFunnel::getReportReceptionQueue(uint8_t virtualChannel) const {
  return tmQueue->getId();
}

ReturnValue_t PusTmFunnel::performOperation(uint8_t) {
  TmTcMessage currentMessage;
  ReturnValue_t status = tmQueue->receiveMessage(&currentMessage);
  while (status == returnvalue::OK) {
    status = handlePacket(currentMessage);
    if (status != returnvalue::OK) {
      sif::warning << "TmFunnel packet handling failed" << std::endl;
      break;
    }
    status = tmQueue->receiveMessage(&currentMessage);
  }

  if (status == MessageQueueIF::EMPTY) {
    return returnvalue::OK;
  }
  return status;
}

ReturnValue_t PusTmFunnel::handlePacket(TmTcMessage &message) {
  uint8_t *packetData = nullptr;
  size_t size = 0;
  ReturnValue_t result = tmStore.modifyData(message.getStorageId(), &packetData, &size);
  if (result != returnvalue::OK) {
    return result;
  }
  PusTmZeroCopyWriter packet(timeReader, packetData, size);
  result = packet.parseDataWithoutCrcCheck();
  if (result != returnvalue::OK) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::warning << "PusTmFunnel::handlePacket: Error parsing received PUS packet" << std::endl;
#endif
    return result;
  }
  packet.setSequenceCount(sourceSequenceCount++);
  sourceSequenceCount = sourceSequenceCount % ccsds::LIMIT_SEQUENCE_COUNT;
  packet.updateErrorControl();

  result = tmQueue->sendToDefault(&message);
  if (result != returnvalue::OK) {
    tmStore.deleteData(message.getStorageId());
#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::error << "PusTmFunnel::handlePacket: Error sending TM to downlink handler" << std::endl;
#endif
  }
  return result;
}

const char *PusTmFunnel::getName() const { return "PUS TM Funnel"; }
