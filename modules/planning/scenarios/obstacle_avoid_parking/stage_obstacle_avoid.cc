
#include <string>
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/scenarios/obstacle_avoid_parking/stage_obstacle_avoid.h"

/*
    第一个阶段:靠近泊车位阶段
    通过获取best_parking_space.cc场景类中查找到的目标停车位，使用ExecuteTaskOnReferenceLine
    直到自车停止（速度小于一定的大小，并且位置接近参考线的停止墙），进入下一个stage

    道路边界、停车位边界
*/
namespace apollo {
namespace planning {
bool StageObstacleAvoid::Init(
        const StagePipeline& config,
        const std::shared_ptr<DependencyInjector>& injector,
        const std::string& config_dir,
        void* context) {
    if (!Stage::Init(config, injector, config_dir, context)) {
        return false;
    }
    scenario_config_.CopyFrom(GetContextAs<ObstacleAvoidParkingContext>()->scenario_config);
    return true;
}
StageResult StageObstacleAvoid::Process(const common::TrajectoryPoint& planning_init_point, Frame* frame) {
    ADEBUG << "stage: StageObstacleAvoid";
    CHECK_NOTNULL(frame);
    StageResult result;
    auto scenario_context = GetContextAs<ObstacleAvoidParkingContext>();

    if (scenario_context->target_parking_spot_id.empty()) {
        return result.SetStageStatus(StageStatusType::ERROR);
    }

    // 从场景上下文信息中获取几个字段的数据，包括：目标停车点id、是否马上预停车、预停车点
    *(frame->mutable_open_space_info()->mutable_target_parking_spot_id()) = scenario_context->target_parking_spot_id;
    frame->mutable_open_space_info()->set_pre_stop_rightaway_flag(scenario_context->pre_stop_rightaway_flag);
    *(frame->mutable_open_space_info()->mutable_pre_stop_rightaway_point())
            = scenario_context->pre_stop_rightaway_point;
    auto* reference_lines = frame->mutable_reference_line_info();
    for (auto& reference_line : *reference_lines) {
        // 基于障碍物的路径决策对象？
        auto* path_decision = reference_line.path_decision();
        if (nullptr == path_decision) {
            continue;
        }
        // 找到目的地（障碍物）
        auto* dest_obstacle = path_decision->Find(FLAGS_destination_obstacle_id);
        if (nullptr == dest_obstacle) {
            continue;
        }
        // 对目的地（障碍物）增加纵向决策？
        ObjectDecisionType decision;
        decision.mutable_ignore();
        dest_obstacle->EraseDecision();
        dest_obstacle->AddLongitudinalDecision("ignore-dest-in-valet-parking", decision);
    }
    result = ExecuteTaskOnOpenSpace(frame);

    scenario_context->pre_stop_rightaway_flag = frame->open_space_info().pre_stop_rightaway_flag();
    scenario_context->pre_stop_rightaway_point = frame->open_space_info().pre_stop_rightaway_point();

    // 检查自车是否已经停车，这个是进入下一个stage的条件
    if (CheckADCStop(*frame)) {
        next_stage_ = "BEST_PARKING_PARKING";
        return StageResult(StageStatusType::FINISHED);
    }
    if (result.HasError()) {
        AERROR << "StopSignUnprotectedStagePreStop planning error";
        return result.SetStageStatus(StageStatusType::ERROR);
    }

    // 否则一直执行该stage的task
    return result.SetStageStatus(StageStatusType::RUNNING);
}

bool StageObstacleAvoid::CheckADCStop(const Frame& frame) {
    const auto& reference_line_info = frame.reference_line_info().front();
    const double adc_speed = injector_->vehicle_state()->linear_velocity();
    const double max_adc_stop_speed
            = common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param().max_abs_speed_when_stopped();
    // 自车速度要比设置的最大停车速度要小
    if (adc_speed > max_adc_stop_speed) {
        ADEBUG << "ADC not stopped: speed[" << adc_speed << "]";
        return false;
    }

    // check stop close enough to stop line of the stop_sign
    // 自车位置离开放空间停止墙的距离要小于最大有效停车距离
    const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
    const double stop_fence_start_s = frame.open_space_info().open_space_pre_stop_fence_s();
    const double distance_stop_line_to_adc_front_edge = stop_fence_start_s - adc_front_edge_s;

    if (distance_stop_line_to_adc_front_edge > scenario_config_.max_valid_stop_distance()) {
        ADEBUG << "not a valid stop. too far from stop line.";
        return false;
    }
    return true;
}

}  // namespace planning
}  // namespace apollo
