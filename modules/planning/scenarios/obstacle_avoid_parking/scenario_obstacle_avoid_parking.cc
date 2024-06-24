/******************************************************************************
 * @file scenario_obstacle_avoid_parking.cc
 *****************************************************************************/

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
    // 1.是否有障碍物
    AINFO << frame.reference_line_info().size();
    // for (auto reference_line : frame.reference_line_info()) {
    //     for (const auto* obstacle : reference_line.path_decision()->obstacles().Items()) {
    //         if (obstacle->IsVirtual()) {
    //             continue;
    //         }
    //         if (!obstacle->IsStatic()) {
    //             continue;
    //         }
    //         if (obstacle->LongitudinalDecision().has_stop()) {
    //             AINFO << "需要停车的障碍物id：" << obstacle->Id();
    //             AINFO << "需要停车的障碍物速度：" << obstacle->speed();
    //             AINFO << "需要停车的障碍物x坐标：" << obstacle->GetBoundingBox().center_x();
    //             AINFO << "需要停车的障碍物y坐标：" << obstacle->GetBoundingBox().center_y();
    //             AINFO << "进入避障停车场景, ScenarioObstacleAvoidParking";
    //             return true;
    //         }
    //     }
    // }
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

    if (other_scenario == nullptr || frame.reference_line_info().empty()) {
        return false;

        if (parking_command && parking_spot_id) {
            target_parking_spot_id = frame.local_view().planning_command->parking_command().parking_spot_id();
        }
    }

    if (!parking_command) {
        // 将路由终点作为目的地（地图红线的结束位置）
        const auto routing_end = frame.local_view().end_lane_way_point;
        common::PointENU end_position;
        end_position.set_x(routing_end->pose().x());
        end_position.set_y(routing_end->pose().y());
        AINFO << "end_position:" << end_position.x() << "," << end_position.y();

        if (nullptr == routing_end) {
            return false;
        }

        common::SLPoint dest_sl;
        const auto& reference_line_info = frame.reference_line_info().front();
        const auto& reference_line = reference_line_info.reference_line();
        reference_line.XYToSL(routing_end->pose(), &dest_sl);
        const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
        const double adc_distance_to_dest = dest_sl.s() - adc_front_edge_s;

        // 离目的地超过停车查找范围则不停车
        if (adc_distance_to_dest > parking_spot_range_to_start) {
            AINFO << "adc_distance_to_dest: " << adc_distance_to_dest
                  << ",parking_spot_range_to_start: " << parking_spot_range_to_start;
            return false;
        }

        if (target_parking_spot_id.empty()) {
            std::vector<apollo::hdmap::ParkingSpaceInfoConstPtr> parking_spaces;
            // 查找结束位置的parking_spot_range_to_start范围内停车位
            if (hdmap_->GetParkingSpaces(end_position, 10, &parking_spaces) == 0
                // if (hdmap_->GetParkingSpaces(end_position, parking_spot_range_to_start, &parking_spaces) == 0
                && parking_spaces.size() > 0) {
                for (auto parking_space : parking_spaces) {
                    // 这里是按照什么顺序输出的停车位？取了最后一个停车位
                    target_parking_space = parking_space;
                    std::string parking_space_id = target_parking_space->parking_space().id().id();
                    target_parking_spot_id = parking_space_id;
                }
            }
        }
    }

    if (target_parking_spot_id.empty()) {
        return false;
    }

    // 绕过障碍物，经过空置的停车位，停到固定的停车位？

    // 切入条件：
    // 1、前方有障碍物
    // 2、无其他车道可以变道
    // 3、停车位都是空置
    // 返回true，切入该scenario

    // stage1:变道绕行
    // 停车位的空间也作为可以占用的车道
    // 路径规划和决策
    // 速度规划和决策
    // stage2:准备停车
    // stage3:openspace停车

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

// 搜索路径上重叠区域是否包含停车点位
bool ScenarioObstacleAvoidParking::SearchTargetParkingSpotOnPath(
        const Path& nearby_path,
        const std::string& target_parking_id,
        PathOverlap* parking_space_overlap) {
    const auto& parking_space_overlaps = nearby_path.parking_space_overlaps();
    for (const auto& parking_overlap : parking_space_overlaps) {
        if (parking_overlap.object_id == target_parking_id) {
            *parking_space_overlap = parking_overlap;
            return true;
        }
    }
    return false;
}

// 检测是否在进入停车范围内
bool ScenarioObstacleAvoidParking::CheckDistanceToParkingSpot(
        const Frame& frame,
        const VehicleState& vehicle_state,
        const Path& nearby_path,
        const double parking_start_range,
        const PathOverlap& parking_space_overlap) {
    // TODO(Jinyun) parking overlap s are wrong on map, not usable
    const hdmap::HDMap* hdmap = hdmap::HDMapUtil::BaseMapPtr();
    hdmap::Id id;
    double center_point_s, center_point_l;
    id.set_id(parking_space_overlap.object_id);
    ParkingSpaceInfoConstPtr target_parking_spot_ptr = hdmap->GetParkingSpaceById(id);
    // 获取停车位四个角的坐标和中心的坐标
    Vec2d left_bottom_point = target_parking_spot_ptr->polygon().points().at(0);
    Vec2d right_bottom_point = target_parking_spot_ptr->polygon().points().at(1);
    Vec2d right_top_point = target_parking_spot_ptr->polygon().points().at(2);
    Vec2d left_top_point = target_parking_spot_ptr->polygon().points().at(3);
    Vec2d center_point = (left_bottom_point + right_bottom_point + right_top_point + left_top_point) / 4.0;
    // 坐标转换，自然坐标系转Frenet坐标系
    nearby_path.GetNearestPoint(center_point, &center_point_s, &center_point_l);
    double vehicle_point_s = 0.0;
    double vehicle_point_l = 0.0;
    Vec2d vehicle_vec(vehicle_state.x(), vehicle_state.y());
    // 坐标转换
    nearby_path.GetNearestPoint(vehicle_vec, &vehicle_point_s, &vehicle_point_l);
    if (std::abs(center_point_s - vehicle_point_s) < parking_start_range) {
        return true;
    }
    return false;
}

}  // namespace planning
}  // namespace apollo
