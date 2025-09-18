#ifndef COMMON_COMMONSYSTEMOBJECTS_H_
#define COMMON_COMMONSYSTEMOBJECTS_H_

#include <fsfw/objectmanager/frameworkObjects.h>

#include <cstdint>

namespace objects {
enum commonObjects : object_id_t {

  /* 0x41 ('A') for Assemblies */
  TEST_ASSEMBLY = 0x4100CAFE,

  /* 0x43 ('C') for Controllers */
  TEST_CONTROLLER = 0x4301CAFE,

  /* 0x44 ('D') for Device Handlers */
  TEST_DEVICE_HANDLER_0 = 0x4401AFFE,
  TEST_DEVICE_HANDLER_1 = 0x4402AFFE,

  // Reaction Wheel Handler
  //RW_HANDLER = 0x4403AFFE, // JH

  /* 0x49 ('I') for Communication Interfaces */
  TEST_ECHO_COM_IF = 0x4900AFFE,
  //RW_SERIAL_COM_IF = 0x4901AFFE, // JH

  /* 0x63 ('C') for core objects */
  CCSDS_DISTRIBUTOR = 0x63000000,
  PUS_DISTRIBUTOR = 0x63000001,
  TM_FUNNEL = 0x63000002,
  CFDP_DISTRIBUTOR = 0x63000003,
  CFDP_HANDLER = 0x63000004,
  PUS_TM_FUNNEL = 0x63000005,
  CFDP_TM_FUNNEL = 0x64000006,

  /* 0x74 ('t') for test and example objects  */
  TEST_TASK = 0x7400CAFE,
  TEST_DUMMY_1 = 0x74000001,
  TEST_DUMMY_2 = 0x74000002,
  TEST_DUMMY_3 = 0x74000003,
  TEST_DUMMY_4 = 0x74000004,
  TEST_DUMMY_5 = 0x74000005,

  // NEW ReactionWheelsHandler & Pus objects ---------------------
 RW_SERIAL_COM_IF = 0x7302,
 RW_HANDLER       = 0x4402,
 RW_PUS_SERVICE       = 0x5020,

};
}

#endif /* COMMON_COMMONSYSTEMOBJECTS_H_ */
