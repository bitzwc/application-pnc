/*
 * 绕过障碍物，经过空置的停车位，停到固定的停车位？

    切入条件：
    1、前方有障碍物
    2、无其他车道可以变道
    3、停车位都是空置
    返回true，切入该scenario

    stage1:变道绕行
    停车位的空间也作为可以占用的车道
    路径规划和决策
    速度规划和决策
    stage2:准备停车
    stage3:openspace停车
 */

#include "modules/planning/scenarios/obstacle_avoid_parking/scenario_obstacle_avoid_parking.h"
#include "modules/planning/planning_base/common/frame.h"

namespace apollo {
namespace planning {

using apollo::common::VehicleState;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;
using apollo::hdmap::ParkingSpaceInfoConstPtr;
using apollo::hdmap::Path;
using apollo::hdmap::PathOverlap;

// 继承父类Scenario的Init方法，实现场景的初始化，injector是依赖注入器？
bool ScenarioObstacleAvoidParking::Init(std::shared_ptr<DependencyInjector> injector, const std::string& name) {
    if (init_) {
        return true;
    }

    if (!Scenario::Init(injector, name)) {
        AERROR << "failed to init scenario" << Name();
        return false;
    }

    if (!Scenario::LoadConfig<ObstacleAvoidParkingConfig>(&context_.scenario_config)) {
        AERROR << "fail to get config of scenario" << Name();
        return false;
    }
    hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
    CHECK_NOTNULL(hdmap_);
    init_ = true;
    return true;
}

// 如何把parking space增加到可变道的空间范围内？
// 修改借道绕行task使用的路径边界，原来用的是车道边界，这里调整为停车位的边界 boundary
bool ScenarioObstacleAvoidParking::IsTransferable(const Scenario* const other_scenario, const Frame& frame) {
    if (other_scenario == nullptr || frame.reference_line_info().empty()) {
        return false;
    }
    // 1.是否有障碍物
    AINFO << "参考线数量: " << frame.reference_line_info().size();
    AINFO << "障碍物数量:" << frame.obstacles().size();
    for (auto obstacle : frame.obstacles()) {
        AINFO << "障碍物ID:" << obstacle->Id() << ",障碍物速度: " << obstacle->speed();
        if (obstacle->Id() != "DEST") {
            // 自车坐标
            const auto& reference_line_info = frame.reference_line_info().front();
            const auto& reference_line = reference_line_info.reference_line();
            const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();

            // 障碍物坐标
            double obstacle_x = obstacle->PerceptionBoundingBox().center_x();
            double obstacle_y = obstacle->PerceptionBoundingBox().center_y();
            common::PointENU obstacle_position;
            obstacle_position.set_x(obstacle_x);
            obstacle_position.set_y(obstacle_y);
            common::SLPoint obstacle_sl;
            reference_line.XYToSL(obstacle_position, &obstacle_sl);

            const double adc_to_obstacle_dis = obstacle_sl.s() - adc_front_edge_s;
            AINFO << "obstacle_point_s:" << obstacle_sl.s() << ",adc_front_edge_s:" << adc_front_edge_s
                  << ",adc_to_obstacle_dis: " << adc_to_obstacle_dis;
            // 如果障碍物和自车距离小于xx，则进入该场景
            if (adc_to_obstacle_dis < 10.0) {
                return true;
            }
        }
    }
    return false;

    // 2.旁边停车位上没有车

    // 目标停车位
    apollo::hdmap::ParkingSpaceInfoConstPtr target_parking_space;
    std::string target_parking_spot_id;
    double parking_spot_range_to_start = context_.scenario_config.parking_spot_range_to_start();
    auto parking_command = frame.local_view().planning_command->has_parking_command();
    auto parking_spot_id = frame.local_view().planning_command->parking_command().has_parking_spot_id();

    // AINFO << "frame.local_view().planning_command.command_id:"
    //     << frame.local_view().planning_command->command_id() << std::endl;
    // 这里切换场景时，不满足条件，没有停车命令和停车点
    AINFO << "parking_command: " << parking_command << ", parking_spot_id: " << parking_spot_id << std::endl;

    // 找到停车点
    AINFO << "target_parking_spot_id: " << target_parking_spot_id;
    AINFO << "min_x: " << target_parking_space->polygon().min_x();
    AINFO << "max_x: " << target_parking_space->polygon().max_x();
    AINFO << "min_y: " << std::fixed << std::setprecision(2) << target_parking_space->polygon().min_y();
    AINFO << "max_y: " << std::fixed << std::setprecision(2) << target_parking_space->polygon().max_y();

    const auto& nearby_path = frame.reference_line_info().front().reference_line().map_path();
    PathOverlap parking_space_overlap;
    const auto& vehicle_state = frame.vehicle_state();

    // if (!SearchTargetParkingSpotOnPath(nearby_path, target_parking_spot_id, &parking_space_overlap)) {
    //     AINFO << "No such parking spot found after searching all path forward "
    //              "possible"
    //           << target_parking_spot_id;
    //     return false;
    // }
    // if (!CheckDistanceToParkingSpot(
    //             frame, vehicle_state, nearby_path, parking_spot_range_to_start, parking_space_overlap)) {
    //     AINFO << "target parking spot found, but too far, distance larger than "
    //              "pre-defined distance"
    //           << target_parking_spot_id;
    //     return false;
    // }
    context_.target_parking_spot_id = target_parking_spot_id;
    return true;
}

}  // namespace planning
}  // namespace apollo
