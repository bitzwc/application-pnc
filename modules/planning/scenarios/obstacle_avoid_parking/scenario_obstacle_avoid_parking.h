/******************************************************************************
 * @file scenario_obstacle_avoid_parking.h
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>

#include "modules/common_msgs/map_msgs/map_id.pb.h"
#include "modules/planning/scenarios/obstacle_avoid_parking/proto/obstacle_avoid_parking.pb.h"

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_interface_base/scenario_base/scenario.h"

#include "gtest/gtest.h"
#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
namespace apollo {
namespace planning {

// 场景上下文信息
struct ObstacleAvoidParkingContext : public ScenarioContext {
    ObstacleAvoidParkingConfig scenario_config;
    std::string target_parking_spot_id;
    bool pre_stop_rightaway_flag = false;
    hdmap::MapPathPoint pre_stop_rightaway_point;
};

class ScenarioObstacleAvoidParking : public Scenario {
public:
    bool Init(std::shared_ptr<DependencyInjector> injector, const std::string& name) override;

    /**
     * @brief Get the scenario context.
     */
    ObstacleAvoidParkingContext* GetContext() override {
        return &context_;
    }

    bool IsTransferable(const Scenario* const other_scenario, const Frame& frame) override;

private:
    static bool SearchTargetParkingSpotOnPath(
            const hdmap::Path& nearby_path,
            const std::string& target_parking_id,
            hdmap::PathOverlap* parking_space_overlap);
    static bool CheckDistanceToParkingSpot(
            const Frame& frame,
            const common::VehicleState& vehicle_state,
            const hdmap::Path& nearby_path,
            const double parking_start_range,
            const hdmap::PathOverlap& parking_space_overlap);

private:
    bool init_ = false;                    // 是否初始化
    ObstacleAvoidParkingContext context_;  // 场景上下文
    const hdmap::HDMap* hdmap_ = nullptr;  // 高精地图指针
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::ScenarioObstacleAvoidParking, Scenario)

}  // namespace planning
}  // namespace apollo