#include "CfdpTmFunnel.h"

#include "fsfw/ipc/QueueFactory.h"
#include "fsfw/tmtcpacket/ccsds/SpacePacketCreator.h"
#include "fsfw/tmtcservices/TmTcMessage.h"

CfdpTmFunnel::CfdpTmFunnel(object_id_t objectId, uint16_t cfdpInCcsdsApid,
                           const AcceptsTelemetryIF& downlinkDestination, StorageManagerIF& tmStore)
    : SystemObject(objectId), cfdpInCcsdsApid(cfdpInCcsdsApid), tmStore(tmStore) {
  msgQueue = QueueFactory::instance()->createMessageQueue(5);
  msgQueue->setDefaultDestination(downlinkDestination.getReportReceptionQueue());
}

const char* CfdpTmFunnel::getName() const { return "CFDP TM Funnel"; }

MessageQueueId_t CfdpTmFunnel::getReportReceptionQueue(uint8_t virtualChannel) const {
  return msgQueue->getId();
}

ReturnValue_t CfdpTmFunnel::performOperation(uint8_t) {
  TmTcMessage currentMessage;
  ReturnValue_t status = msgQueue->receiveMessage(&currentMessage);
  while (status == returnvalue::OK) {
    status = handlePacket(currentMessage);
    if (status != returnvalue::OK) {
      sif::warning << "CfdpTmFunnel packet handling failed" << std::endl;
      break;
    }
    status = msgQueue->receiveMessage(&currentMessage);
  }

  if (status == MessageQueueIF::EMPTY) {
    return returnvalue::OK;
  }
  return status;
}

ReturnValue_t CfdpTmFunnel::initialize() { return returnvalue::OK; }

ReturnValue_t CfdpTmFunnel::handlePacket(TmTcMessage& msg) {
  const uint8_t* cfdpPacket = nullptr;
  size_t cfdpPacketLen = 0;
  ReturnValue_t result = tmStore.getData(msg.getStorageId(), &cfdpPacket, &cfdpPacketLen);
  if (result != returnvalue::OK) {
    return result;
  }
  auto spacePacketHeader =
      SpacePacketCreator(ccsds::PacketType::TM, false, cfdpInCcsdsApid,
                         ccsds::SequenceFlags::UNSEGMENTED, sourceSequenceCount++, 0);
  sourceSequenceCount = sourceSequenceCount & ccsds::LIMIT_SEQUENCE_COUNT;
  spacePacketHeader.setCcsdsLenFromTotalDataFieldLen(cfdpPacketLen);
  uint8_t* newPacketData = nullptr;
  store_address_t newStoreId{};
  result =
      tmStore.getFreeElement(&newStoreId, spacePacketHeader.getFullPacketLen(), &newPacketData);
  if (result != returnvalue::OK) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::warning << "CfdpTmFunnel::handlePacket: Error getting TM store element of size "
                 << spacePacketHeader.getFullPacketLen() << std::endl;
#endif
    return result;
  }
  size_t serSize = 0;
  result =
      spacePacketHeader.serializeBe(&newPacketData, &serSize, spacePacketHeader.getFullPacketLen());
  if (result != returnvalue::OK) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::error << "CfdpTmFunnel::handlePacket: Error serializing packet" << std::endl;
#endif
    return result;
  }
  std::memcpy(newPacketData, cfdpPacket, cfdpPacketLen);
  // Delete old packet
  tmStore.deleteData(msg.getStorageId());
  msg.setStorageId(newStoreId);
  result = msgQueue->sendToDefault(&msg);
  if (result != returnvalue::OK) {
    tmStore.deleteData(msg.getStorageId());
#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::error << "CfdpTmFunnel::handlePacket: Error sending TM to downlink handler" << std::endl;
#endif
  }
  return result;
}
