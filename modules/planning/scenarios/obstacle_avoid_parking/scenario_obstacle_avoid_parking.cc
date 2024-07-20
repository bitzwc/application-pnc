/*
    绕过障碍物，经过空置的停车位，停到固定的停车位
    场景切换条件：
        1、前方有障碍物
        2、障碍物旁的停车位都是空置
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
    // 1.是否靠近障碍物
    bool is_close_to_obstacle = false;
    // 靠近障碍物的最短距离
    double close_to_obstacle_distance = context_.scenario_config.close_to_obstacle_distance();
    AINFO << "参考线数量: " << frame.reference_line_info().size();
    AINFO << "障碍物数量:" << frame.obstacles().size();
    common::PointENU obstacle_position;
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
            obstacle_position.set_x(obstacle_x);
            obstacle_position.set_y(obstacle_y);
            common::SLPoint obstacle_sl;
            reference_line.XYToSL(obstacle_position, &obstacle_sl);

            const double adc_to_obstacle_dis = obstacle_sl.s() - adc_front_edge_s;
            AINFO << "obstacle_point_s:" << obstacle_sl.s() << ",adc_front_edge_s:" << adc_front_edge_s
                  << ",adc_to_obstacle_dis: " << adc_to_obstacle_dis;
            // 如果障碍物和自车距离小于close_to_obstacle_distance，则进入该场景
            if (adc_to_obstacle_dis < close_to_obstacle_distance) {
                is_close_to_obstacle = true;
                break;
            }
        }
    }
    if (!is_close_to_obstacle) {
        return false;
    }

    // 2.旁边停车位上没有车
    std::vector<apollo::hdmap::ParkingSpaceInfoConstPtr> parking_spaces;
    double parking_spot_range_to_start = context_.scenario_config.parking_spot_range_to_start();
    bool is_parking_spot_occ = false;

    // 查找障碍物附近的停车位
    if (hdmap_->GetParkingSpaces(obstacle_position, parking_spot_range_to_start, &parking_spaces) == 0
        && parking_spaces.size() > 0) {
        for (auto parking_space : parking_spaces) {
            // 获取车位的位置、是否被车辆占用
            is_parking_spot_occ = parking_space->polygon().AABoundingBox().IsPointIn(
                    Vec2d(obstacle_position.x(), obstacle_position.y()));
            AINFO << "find parking spot id: " << parking_space->parking_space().id().id()
                  << ",Bounding Box: " << parking_space->polygon().AABoundingBox().DebugString()
                  << ",车位上是否有障碍物: " << is_parking_spot_occ;
            if (is_parking_spot_occ) {
                return false;
            }
        }
    }

    // 获取目标停车位
    std::string target_parking_spot_id;
    if (!GetTargetParkingSpotId(target_parking_spot_id, frame)) {
        return false;
    }

    AINFO << "找到目标停车位: " << target_parking_spot_id;
    context_.target_parking_spot_id = target_parking_spot_id;
    // 把可以借道的停车位写入场景上下文中，给下游stage使用
    context_.parking_spaces = parking_spaces;
    return true;
}

// 获取目标停车位
bool ScenarioObstacleAvoidParking::GetTargetParkingSpotId(std::string& target_parking_spot_id, const Frame& frame) {
    auto has_parking_command = frame.local_view().planning_command->has_parking_command();
    auto has_parking_spot_id = frame.local_view().planning_command->parking_command().has_parking_spot_id();
    auto parking_spot_id = frame.local_view().planning_command->parking_command().parking_spot_id();

    AINFO << "has_parking_command: " << has_parking_command << ", has_parking_spot_id: " << has_parking_spot_id;
    // 如果规划命令中已有停车相关命令，则使用已有的停车点id
    if (has_parking_command && has_parking_spot_id) {
        target_parking_spot_id = parking_spot_id;
    }

    // 没有规划停车命令
    if (!has_parking_command) {
        // 将路由终点作为目的地（地图红线的结束位置）
        const auto routing_end = frame.local_view().end_lane_way_point;
        common::PointENU end_position;
        end_position.set_x(routing_end->pose().x());
        end_position.set_y(routing_end->pose().y());
        AINFO << "end_position:" << end_position.x() << "," << end_position.y();
        if (nullptr == routing_end) {
            return false;
        }
        // 是否停车点id为空
        if (target_parking_spot_id.empty()) {
            std::vector<apollo::hdmap::ParkingSpaceInfoConstPtr> parking_spaces;
            // 最小停车位距离
            double min_parking_spot_dis = std::numeric_limits<double>::infinity();
            // 查找结束位置的parking_spot_range_to_start范围内停车位
            double parking_spot_range_to_start = context_.scenario_config.parking_spot_range_to_start();
            if (hdmap_->GetParkingSpaces(end_position, parking_spot_range_to_start, &parking_spaces) == 0
                && parking_spaces.size() > 0) {
                for (auto parking_space_info : parking_spaces) {
                    // 计算停车位和导航终点的距离
                    double distance
                            = parking_space_info->polygon().DistanceTo(Vec2d(end_position.x(), end_position.y()));
                    if (distance < min_parking_spot_dis) {
                        min_parking_spot_dis = distance;
                        target_parking_spot_id = parking_space_info->parking_space().id().id();
                    }
                }
            }
        }
    }
    return !target_parking_spot_id.empty();
}

}  // namespace planning
}  // namespace apollo
