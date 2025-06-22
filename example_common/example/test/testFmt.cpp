#include "testFmt.h"

void fmtTests() {
  sif::fdebug(__FILENAME__, __LINE__, "Hello {} {}", "World\n");
  sif::fdebug_t(__FILENAME__, __LINE__, "Hallo\n");
  FSFW_LOGD("{}", "Hallo\n");
  // MY_LOG("{}", "test\n");
  // sif::finfo_t("Hallo\n");
  // sif::finfo("Hallo\n");
  // sif::fwarning("Hello\n");
  // sif::fwarning_t("Hello\n");
  // sif::ferror("Hello\n");
  // sif::ferror_t("Hello\n");
}