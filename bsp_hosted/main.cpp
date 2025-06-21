//Test commit
#include <bsp_hosted/core/InitMission.h>
#include <bsp_hosted/core/ObjectFactory.h>
#include <fsfw/objectmanager/ObjectManager.h>
#include <fsfw/platform.h>
#include <fsfw/serviceinterface/ServiceInterface.h>
#include <fsfw/tasks/TaskFactory.h>

#include <chrono>

#include "example/test/MutexExample.h"
#include "example/utility/utility.h"

#ifdef PLATFORM_WIN
static const char* COMPILE_PRINTOUT = "Windows";
#elif defined(PLATFORM_UNIX)
static const char* COMPILE_PRINTOUT = "Linux";
#else
static const char* COMPILE_PRINTOUT = "unknown OS";
#endif

#if FSFW_CPP_OSTREAM_ENABLED == 1
ServiceInterfaceStream sif::debug("DEBUG", false);
ServiceInterfaceStream sif::info("INFO", false);
ServiceInterfaceStream sif::warning("WARNING", false);
ServiceInterfaceStream sif::error("ERROR", false, true, true);
#endif

int main() {
  utility::commonInitPrint("Hosted", COMPILE_PRINTOUT);

#if FSFW_CPP_OSTREAM_ENABLED == 1
  sif::info << "Producing system objects.." << std::endl;
#else
  sif::printInfo("Producing system objects..\n");
#endif /* FSFW_CPP_OSTREAM_ENABLED == 1 */

  ObjectManager* objManager = ObjectManager::instance();
  objManager->setObjectFactoryFunction(ObjectFactory::produce, nullptr);

#if FSFW_CPP_OSTREAM_ENABLED == 1
  sif::info << "Objects created successfully.." << std::endl;
  sif::info << "Initializing objects.." << std::endl;
#else
  sif::printInfo("Objects created successfully..\n");
#endif /* FSFW_CPP_OSTREAM_ENABLED == 1 */

  objManager->initialize();

#if FSFW_CPP_OSTREAM_ENABLED == 1
  sif::info << "Creating tasks.." << std::endl;
#else
  sif::printInfo("Creating tasks..\n");
#endif /* FSFW_CPP_OSTREAM_ENABLED == 1 */

  InitMission::createTasks();

  MutexExample::example();
  // PusPacketCreator::createPusPacketAndPrint();

  /* Permanent loop. */
  for (;;) {
    /* Sleep main thread, not needed anymore. */
    TaskFactory::delayTask(5000);
  }
  return 0;
}
