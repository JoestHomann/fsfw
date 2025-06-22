#ifndef MISSION_UTILITY_TASKCREATION_H_
#define MISSION_UTILITY_TASKCREATION_H_

#include <fsfw/objectmanager/SystemObjectIF.h>
#include <fsfw/serviceinterface/ServiceInterface.h>

namespace task {

void printInitError(const char *objName, object_id_t objectId) {
#if FSFW_CPP_OSTREAM_ENABLED == 1
  sif::error << "InitMission: Adding object " << objName << "(" << std::setw(8) << std::setfill('0')
             << std::hex << objectId << std::dec << ") failed." << std::endl;
#else
  sif::printError("InitMission: Adding object %s (0x%08x) failed.\n", objName,
                  static_cast<unsigned int>(objectId));
#endif
}

}  // namespace task

#endif /* MISSION_UTILITY_TASKCREATION_H_ */
