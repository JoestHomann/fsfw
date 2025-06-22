#include "GenericFactory.h"

#include "common/definitions.h"
#include "definitions.h"
#include "example/cfdp/Config.h"
#include "example/test/FsfwExampleTask.h"
#include "example/test/FsfwReaderTask.h"
#include "example/utility/CfdpTmFunnel.h"
#include "example/utility/TmFunnel.h"
#include "fsfw/FSFW.h"
#include "fsfw/cfdp.h"
#include "fsfw/cfdp/CfdpDistributor.h"
#include "fsfw/devicehandlers/CookieIF.h"
#include "fsfw/events/EventManager.h"
#include "fsfw/health/HealthTable.h"
#include "fsfw/internalerror/InternalErrorReporter.h"
#include "fsfw/ipc/QueueFactory.h"
#include "fsfw/pus/CService200ModeCommanding.h"
#include "fsfw/pus/Service11TelecommandScheduling.h"
#include "fsfw/pus/Service17Test.h"
#include "fsfw/pus/Service1TelecommandVerification.h"
#include "fsfw/pus/Service20ParameterManagement.h"
#include "fsfw/pus/Service2DeviceAccess.h"
#include "fsfw/pus/Service3Housekeeping.h"
#include "fsfw/pus/Service5EventReporting.h"
#include "fsfw/pus/Service8FunctionManagement.h"
#include "fsfw/pus/Service9TimeManagement.h"
#include "fsfw/tcdistribution/CcsdsDistributor.h"
#include "fsfw/tcdistribution/PusDistributor.h"
#include "fsfw/timemanager/CdsShortTimeStamper.h"
#include "fsfw/tmtcservices/VerificationReporter.h"
#include "fsfw_hal/host/HostFilesystem.h"
#include "fsfw_tests/integration/assemblies/TestAssembly.h"
#include "fsfw_tests/integration/controller/TestController.h"
#include "fsfw_tests/integration/devices/TestCookie.h"
#include "fsfw_tests/integration/devices/TestDeviceHandler.h"
#include "fsfw_tests/integration/devices/TestEchoComIF.h"
#include "fsfw_tests/internal/InternalUnitTester.h"
#include "objects/systemObjectList.h"

#if OBSW_ADD_CFDP_COMPONENTS == 1
namespace cfdp {
EntityId REMOTE_CFDP_ID(cfdp::WidthInBytes::TWO_BYTES, common::COMMON_CFDP_CLIENT_ENTITY_ID);
RemoteEntityCfg GROUND_REMOTE_CFG(REMOTE_CFDP_ID);
OneRemoteConfigProvider REMOTE_CFG_PROVIDER(GROUND_REMOTE_CFG);
HostFilesystem HOST_FS;
ExampleUserHandler USER_HANDLER(HOST_FS);
ExampleFaultHandler EXAMPLE_FAULT_HANDLER;
}  // namespace cfdp
#endif

void ObjectFactory::produceGenericObjects(PusTmFunnel **pusFunnel,
                                          const AcceptsTelemetryIF &tmtcBridge,
                                          CcsdsDistributor **ccsdsDistrib,
                                          StorageManagerIF &tcStore, StorageManagerIF &tmStore) {
#if OBSW_ADD_CORE_COMPONENTS == 1
  /* Framework objects */
  new EventManager(objects::EVENT_MANAGER);
  new HealthTable(objects::HEALTH_TABLE);
  new InternalErrorReporter(objects::INTERNAL_ERROR_REPORTER);
  auto *stamperAndReader = new CdsShortTimeStamper(objects::TIME_STAMPER);
  new VerificationReporter();
  *ccsdsDistrib =
      new CcsdsDistributor(common::COMMON_PUS_APID, objects::CCSDS_DISTRIBUTOR, &tcStore);
  new PusDistributor(common::COMMON_PUS_APID, objects::PUS_DISTRIBUTOR, *ccsdsDistrib);
  *pusFunnel = new PusTmFunnel(objects::PUS_TM_FUNNEL, tmtcBridge, *stamperAndReader, tmStore);
  auto *cfdpFunnel =
      new CfdpTmFunnel(objects::CFDP_TM_FUNNEL, common::COMMON_CFDP_APID, tmtcBridge, tmStore);
  new TmFunnel(objects::TM_FUNNEL, **pusFunnel, *cfdpFunnel);
#endif /* OBSW_ADD_CORE_COMPONENTS == 1 */

  /* PUS stack */
#if OBSW_ADD_PUS_STACK == 1
  new Service1TelecommandVerification(objects::PUS_SERVICE_1_VERIFICATION, common::COMMON_PUS_APID,
                                      pus::PUS_SERVICE_1, objects::PUS_TM_FUNNEL, 5);
  new Service2DeviceAccess(objects::PUS_SERVICE_2_DEVICE_ACCESS, common::COMMON_PUS_APID,
                           pus::PUS_SERVICE_2, 3, 10);
  new Service3Housekeeping(objects::PUS_SERVICE_3_HOUSEKEEPING, common::COMMON_PUS_APID,
                           pus::PUS_SERVICE_3);
  new Service5EventReporting(PsbParams(objects::PUS_SERVICE_5_EVENT_REPORTING,
                                       common::COMMON_PUS_APID, pus::PUS_SERVICE_5),
                             20, 40);
  new Service8FunctionManagement(objects::PUS_SERVICE_8_FUNCTION_MGMT, common::COMMON_PUS_APID,
                                 pus::PUS_SERVICE_8, 3, 10);
  new Service9TimeManagement(
      PsbParams(objects::PUS_SERVICE_9_TIME_MGMT, common::COMMON_PUS_APID, pus::PUS_SERVICE_9));
  new Service17Test(
      PsbParams(objects::PUS_SERVICE_17_TEST, common::COMMON_PUS_APID, pus::PUS_SERVICE_17));
  new Service20ParameterManagement(objects::PUS_SERVICE_20_PARAMETERS, common::COMMON_PUS_APID,
                                   pus::PUS_SERVICE_20);
#if OBSW_ADD_CORE_COMPONENTS == 1
  new Service11TelecommandScheduling<cfg::OBSW_MAX_SCHEDULED_TCS>(
      PsbParams(objects::PUS_SERVICE_11_TC_SCHEDULER, common::COMMON_PUS_APID, pus::PUS_SERVICE_11),
      *ccsdsDistrib);
#endif
  new CService200ModeCommanding(objects::PUS_SERVICE_200_MODE_MGMT, common::COMMON_PUS_APID,
                                pus::PUS_SERVICE_200);
#endif /* OBSW_ADD_PUS_STACK == 1 */

#if OBSW_ADD_TASK_EXAMPLE == 1
  /* Demo objects */
  new FsfwExampleTask(objects::TEST_DUMMY_1);
  new FsfwExampleTask(objects::TEST_DUMMY_2);
  new FsfwExampleTask(objects::TEST_DUMMY_3);

  bool enablePrintout = false;
#if OBSW_TASK_EXAMPLE_PRINTOUT == 1
  enablePrintout = true;
#endif
  new FsfwReaderTask(objects::TEST_DUMMY_4, enablePrintout);
#endif /* OBSW_ADD_TASK_EXAMPLE == 1 */

#if OBSW_ADD_DEVICE_HANDLER_DEMO == 1

#if OBSW_DEVICE_HANDLER_PRINTOUT == 1
  bool enableInfoPrintout = true;
#else
  bool enableInfoPrintout = false;
#endif /* OBSW_DEVICE_HANDLER_PRINTOUT == 1 */

  /* Demo device handler object */
  size_t expectedMaxReplyLen = 64;
  CookieIF *testCookie = new TestCookie(static_cast<address_t>(testdevice::DeviceIndex::DEVICE_0),
                                        expectedMaxReplyLen);
  new TestEchoComIF(objects::TEST_ECHO_COM_IF);
  new TestDevice(objects::TEST_DEVICE_HANDLER_0, objects::TEST_ECHO_COM_IF, testCookie,
                 testdevice::DeviceIndex::DEVICE_0, enableInfoPrintout);
  testCookie = new TestCookie(static_cast<address_t>(testdevice::DeviceIndex::DEVICE_1),
                              expectedMaxReplyLen);
  new TestDevice(objects::TEST_DEVICE_HANDLER_1, objects::TEST_ECHO_COM_IF, testCookie,
                 testdevice::DeviceIndex::DEVICE_1, enableInfoPrintout);

  new TestAssembly(objects::TEST_ASSEMBLY, objects::NO_OBJECT, objects::TEST_DEVICE_HANDLER_0,
                   objects::TEST_DEVICE_HANDLER_1);

#endif /* OBSW_ADD_DEVICE_HANDLER_DEMO == 1 */

  /* Demo controller object */
#if OBSW_ADD_CONTROLLER_DEMO == 1

#if OBSW_CONTROLLER_PRINTOUT == 1
#endif
  new TestController(objects::TEST_CONTROLLER, objects::NO_OBJECT);

#endif /* OBSW_ADD_CONTROLLER_DEMO == 1 */

#if OBSW_PERFORM_INTERNAL_UNITTEST == 1
  InternalUnitTester::TestConfig testCfg;
  testCfg.testArrayPrinter = false;
#if defined FSFW_OSAL_HOST
  // Not implemented yet for hosted OSAL (requires C++20)
  testCfg.testSemaphores = false;
#endif
  InternalUnitTester unittester;
  unittester.performTests(testCfg);
#endif /* OBSW_PERFORM_INTERNAL_UNITTEST == 1 */

#if OBSW_ADD_CFDP_COMPONENTS == 1
  using namespace cfdp;
  MessageQueueIF *cfdpMsgQueue = QueueFactory::instance()->createMessageQueue(32);
  CfdpDistribCfg cfg(objects::CFDP_DISTRIBUTOR, tcStore, cfdpMsgQueue);
  new CfdpDistributor(cfg);

  auto *msgQueue = QueueFactory::instance()->createMessageQueue(32);
  UnsignedByteField<uint16_t> remoteEntityId(common::COMMON_CFDP_CLIENT_ENTITY_ID);
  cfdp::EntityId remoteId(remoteEntityId);
  cfdp::RemoteEntityCfg remoteCfg(remoteId);
  remoteCfg.defaultChecksum = cfdp::ChecksumType::CRC_32;
  FsfwHandlerParams params(objects::CFDP_HANDLER, HOST_FS, *cfdpFunnel, tcStore, tmStore,
                           *msgQueue);
  cfdp::IndicationCfg indicationCfg;
  UnsignedByteField<uint16_t> apid(common::COMMON_CFDP_APID);
  cfdp::EntityId localId(apid);
  GROUND_REMOTE_CFG.defaultChecksum = cfdp::ChecksumType::CRC_32;
  if (PACKET_LIST_PTR == nullptr or LOST_SEGMENTS_PTR == nullptr) {
    sif::error << "CFDP: No packet list or lost segments container set" << std::endl;
  }
  CfdpHandlerCfg cfdpCfg(localId, indicationCfg, USER_HANDLER, EXAMPLE_FAULT_HANDLER,
                         *PACKET_LIST_PTR, *LOST_SEGMENTS_PTR, REMOTE_CFG_PROVIDER);
  auto *cfdpHandler = new CfdpHandler(params, cfdpCfg);
  CcsdsDistributorIF::DestInfo info("CFDP Destination", common::COMMON_CFDP_APID,
                                    cfdpHandler->getRequestQueue(), true);
  (*ccsdsDistrib)->registerApplication(info);
#endif
}

void Factory::setStaticFrameworkObjectIds() {
  MonitoringReportContent<float>::timeStamperId = objects::TIME_STAMPER;
  MonitoringReportContent<double>::timeStamperId = objects::TIME_STAMPER;
  MonitoringReportContent<uint32_t>::timeStamperId = objects::TIME_STAMPER;
  MonitoringReportContent<int32_t>::timeStamperId = objects::TIME_STAMPER;
  MonitoringReportContent<int16_t>::timeStamperId = objects::TIME_STAMPER;
  MonitoringReportContent<uint16_t>::timeStamperId = objects::TIME_STAMPER;

  PusServiceBase::PUS_DISTRIBUTOR = objects::PUS_DISTRIBUTOR;
  PusServiceBase::PACKET_DESTINATION = objects::PUS_TM_FUNNEL;

  CommandingServiceBase::defaultPacketSource = objects::PUS_DISTRIBUTOR;
  CommandingServiceBase::defaultPacketDestination = objects::PUS_TM_FUNNEL;

  VerificationReporter::DEFAULT_RECEIVER = objects::PUS_SERVICE_1_VERIFICATION;
}
