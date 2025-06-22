#ifndef FSFW_EXAMPLE_HOSTED_CONFIG_H
#define FSFW_EXAMPLE_HOSTED_CONFIG_H

#include "fsfw/cfdp.h"

namespace cfdp {

extern PacketInfoListBase* PACKET_LIST_PTR;
extern LostSegmentsListBase* LOST_SEGMENTS_PTR;

class ExampleUserHandler : public UserBase {
 public:
  explicit ExampleUserHandler(HasFileSystemIF& vfs) : cfdp::UserBase(vfs) {}

  void transactionIndication(const cfdp::TransactionId& id) override {}
  void eofSentIndication(const cfdp::TransactionId& id) override {}
  void transactionFinishedIndication(const cfdp::TransactionFinishedParams& params) override {
    sif::info << "File transaction finished for transaction with " << params.id << std::endl;
  }
  void metadataRecvdIndication(const cfdp::MetadataRecvdParams& params) override {
    sif::info << "Metadata received for transaction with " << params.id << std::endl;
  }
  void fileSegmentRecvdIndication(const cfdp::FileSegmentRecvdParams& params) override {}
  void reportIndication(const cfdp::TransactionId& id, cfdp::StatusReportIF& report) override {}
  void suspendedIndication(const cfdp::TransactionId& id, cfdp::ConditionCode code) override {}
  void resumedIndication(const cfdp::TransactionId& id, size_t progress) override {}
  void faultIndication(const cfdp::TransactionId& id, cfdp::ConditionCode code,
                       size_t progress) override {}
  void abandonedIndication(const cfdp::TransactionId& id, cfdp::ConditionCode code,
                           size_t progress) override {}
  void eofRecvIndication(const cfdp::TransactionId& id) override {
    sif::info << "EOF PDU received for transaction with " << id << std::endl;
  }
};

class ExampleFaultHandler : public cfdp::FaultHandlerBase {
 public:
  void noticeOfSuspensionCb(cfdp::TransactionId& id, cfdp::ConditionCode code) override {
    sif::warning << "Notice of suspension detected for transaction " << id
                 << " with condition code: " << cfdp::getConditionCodeString(code) << std::endl;
  }
  void noticeOfCancellationCb(cfdp::TransactionId& id, cfdp::ConditionCode code) override {
    sif::warning << "Notice of suspension detected for transaction " << id
                 << " with condition code: " << cfdp::getConditionCodeString(code) << std::endl;
  }
  void abandonCb(cfdp::TransactionId& id, cfdp::ConditionCode code) override {
    sif::warning << "Transaction " << id
                 << " was abandoned, condition code : " << cfdp::getConditionCodeString(code)
                 << std::endl;
  }
  void ignoreCb(cfdp::TransactionId& id, cfdp::ConditionCode code) override {
    sif::warning << "Fault ignored for transaction " << id
                 << ", condition code: " << cfdp::getConditionCodeString(code) << std::endl;
  }
};

}  // namespace cfdp

#endif  // FSFW_EXAMPLE_HOSTED_CONFIG_H
