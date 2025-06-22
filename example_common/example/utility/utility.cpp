#include "utility.h"

#include <FSFWConfig.h>
#include <OBSWVersion.h>
#include <fsfw/serviceinterface/ServiceInterface.h>

void utility::commonInitPrint(const char *const os, const char *const board) {
  if (os == nullptr or board == nullptr) {
    return;
  }
#if FSFW_CPP_OSTREAM_ENABLED == 1
  std::cout << "-- FSFW Example (" << os << ") v" << FSFW_EXAMPLE_VERSION << "."
            << FSFW_EXAMPLE_SUBVERSION << "." << FSFW_EXAMPLE_REVISION << " --" << std::endl;
  std::cout << "-- Compiled for " << board << " --" << std::endl;
  std::cout << "-- Compiled on " << __DATE__ << " " << __TIME__ << " --" << std::endl;
#else
  printf("\n\r-- FSFW Example (%s) v%d.%d.%d --\n", os, FSFW_EXAMPLE_VERSION,
         FSFW_EXAMPLE_SUBVERSION, FSFW_EXAMPLE_REVISION);
  printf("-- Compiled for %s --\n", board);
  printf("-- Compiled on %s %s --\n", __DATE__, __TIME__);
#endif
}
