#ifndef MISSION_DEMO_DEMODEFINITIONS_H_
#define MISSION_DEMO_DEMODEFINITIONS_H_

#include <fsfw/datapoollocal/LocalPoolVariable.h>
#include <fsfw/datapoollocal/StaticLocalDataSet.h>

/**
 * @brief 	This demo set showcases the local data pool functionality of the
 * 			FSFW
 * @details
 * Each demo object will have an own instance of this set class, which contains
 * pool variables (for read and write access respectively).
 */
class FsfwDemoSet : public StaticLocalDataSet<3> {
 public:
  static constexpr uint32_t DEMO_SET_ID = 0;

  enum PoolIds { VARIABLE, VARIABLE_LIMIT };

  FsfwDemoSet(HasLocalDataPoolIF *hkOwner) : StaticLocalDataSet(hkOwner, DEMO_SET_ID) {}

  lp_var_t<uint32_t> variableRead =
      lp_var_t<uint32_t>(sid.objectId, PoolIds::VARIABLE, this, pool_rwm_t::VAR_READ);
  lp_var_t<uint32_t> variableWrite =
      lp_var_t<uint32_t>(sid.objectId, PoolIds::VARIABLE, this, pool_rwm_t::VAR_WRITE);
  lp_var_t<uint16_t> variableLimit =
      lp_var_t<uint16_t>(sid.objectId, PoolIds::VARIABLE_LIMIT, this);

 private:
};

/**
 * This set will enable object to read the dummy variables from the dataset
 * above. An example application would be a consumer object like a controller
 * which reads multiple sensor values at once.
 */
class CompleteDemoReadSet : public StaticLocalDataSet<3> {
 public:
  static constexpr uint32_t DEMO_SET_ID = 0;

  CompleteDemoReadSet(object_id_t owner, gp_id_t variable1, gp_id_t variable2, gp_id_t variable3)
      : StaticLocalDataSet(sid_t(owner, DEMO_SET_ID)),
        variable1(variable1, this, pool_rwm_t::VAR_READ),
        variable2(variable2, this, pool_rwm_t::VAR_READ),
        variable3(variable3, this, pool_rwm_t::VAR_READ) {}

  lp_var_t<uint32_t> variable1;
  lp_var_t<uint32_t> variable2;
  lp_var_t<uint32_t> variable3;

 private:
};

#endif /* MISSION_DEMO_DEMODEFINITIONS_H_ */
