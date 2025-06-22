#ifndef MISSION_CORE_GENERICFACTORY_H_
#define MISSION_CORE_GENERICFACTORY_H_

#include <fsfw/objectmanager/SystemObjectIF.h>

#include "OBSWConfig.h"
#include "example/utility/PusTmFunnel.h"
#include "fsfw/cfdp/handler/DestHandler.h"
#include "fsfw/storagemanager/StorageManagerIF.h"

class TmFunnel;
class CcsdsDistributor;

namespace ObjectFactory {

/**
 * @brief   Produce hardware independant objects. Called by bsp specific
 *          object factory.
 */
void produceGenericObjects(PusTmFunnel** pusFunnel, const AcceptsTelemetryIF& tmtcBridge,
                           CcsdsDistributor** ccsdsDistributor, StorageManagerIF& tcStore,
                           StorageManagerIF& tmStore);

}  // namespace ObjectFactory

#endif /* MISSION_CORE_GENERICFACTORY_H_ */
