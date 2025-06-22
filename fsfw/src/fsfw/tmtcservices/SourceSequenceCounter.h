#ifndef FSFW_TMTCSERVICES_SOURCESEQUENCECOUNTER_H_
#define FSFW_TMTCSERVICES_SOURCESEQUENCECOUNTER_H_

#include "fsfw/tmtcpacket/ccsds/SpacePacketReader.h"

class SourceSequenceCounter {
 private:
  uint16_t sequenceCount = 0;

 public:
  SourceSequenceCounter(uint16_t initialSequenceCount = 0) : sequenceCount(initialSequenceCount) {}
  void increment() { sequenceCount = (sequenceCount + 1) % (ccsds::LIMIT_SEQUENCE_COUNT); }
  void decrement() { sequenceCount = (sequenceCount - 1) % (ccsds::LIMIT_SEQUENCE_COUNT); }
  uint16_t get() const { return this->sequenceCount; }
  void reset(uint16_t toValue = 0) { sequenceCount = toValue % (ccsds::LIMIT_SEQUENCE_COUNT); }
  SourceSequenceCounter& operator++(int) {
    this->increment();
    return *this;
  }
  SourceSequenceCounter& operator--(int) {
    this->decrement();
    return *this;
  }
  SourceSequenceCounter& operator=(const uint16_t& newCount) {
    sequenceCount = newCount;
    return *this;
  }

  operator uint16_t() { return this->get(); }
};

#endif /* FSFW_TMTCSERVICES_SOURCESEQUENCECOUNTER_H_ */
