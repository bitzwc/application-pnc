

#pragma once

#include <memory>

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/planning_interface_base/scenario_base/stage.h"
#include "modules/planning/scenarios/obstacle_avoid_parking/scenario_obstacle_avoid_parking.h"
namespace apollo {
namespace planning {

class StageParking : public Stage {
public:
    StageResult Process(const common::TrajectoryPoint& planning_init_point, Frame* frame) override;

private:
    StageResult FinishStage();

    ObstacleAvoidParkingConfig scenario_config_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::StageParking, Stage)

}  // namespace planning
}  // namespace apollo
