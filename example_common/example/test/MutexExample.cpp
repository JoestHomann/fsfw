#include "MutexExample.h"

#include <fsfw/ipc/MutexFactory.h>
#include <fsfw/serviceinterface/ServiceInterface.h>

void MutexExample::example() {
  MutexIF *mutex = MutexFactory::instance()->createMutex();
  MutexIF *mutex2 = MutexFactory::instance()->createMutex();

  ReturnValue_t result = mutex->lockMutex(MutexIF::TimeoutType::WAITING, 2 * 60 * 1000);
  if (result != returnvalue::OK) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::error << "MutexExample::example: Lock Failed with " << result << std::endl;
#else
    sif::printError("MutexExample::example: Lock Failed with %hu\n", result);
#endif
  }

  result = mutex2->lockMutex(MutexIF::TimeoutType::BLOCKING);
  if (result != returnvalue::OK) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::error << "MutexExample::example: Lock Failed with " << result << std::endl;
#else
    sif::printError("MutexExample::example: Lock Failed with %hu\n", result);
#endif
  }

  result = mutex->unlockMutex();
  if (result != returnvalue::OK) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::error << "MutexExample::example: Unlock Failed with " << result << std::endl;
#else
    sif::printError("MutexExample::example: Unlock Failed with %hu\n", result);
#endif
  }

  result = mutex2->unlockMutex();
  if (result != returnvalue::OK) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
    sif::error << "MutexExample::example: Unlock Failed with " << result << std::endl;
#else
    sif::printError("MutexExample::example: Unlock Failed with %hu\n", result);
#endif
  }
}
