#include "modules/planning/scenarios/best_parking_space/stage_parking.h"
#include "modules/planning/planning_base/common/frame.h"

/*
    第二个阶段:泊车阶段
    车辆在第一个stage已经停在停止墙前面
*/
namespace apollo {
namespace planning {

StageResult StageBestParking::Process(const common::TrajectoryPoint& planning_init_point, Frame* frame) {
    AERROR << "1";
    // Open space planning doesn't use planning_init_point from upstream because
    // of different stitching strategy
    auto scenario_context = GetContextAs<BestParkingSpaceContext>();
    // 使用open space轨迹
    frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);

    AINFO << "stage parking get target_parking_spot_id: " << scenario_context->target_parking_spot_id;
    *(frame->mutable_open_space_info()->mutable_target_parking_spot_id()) = scenario_context->target_parking_spot_id;
    StageResult result = ExecuteTaskOnOpenSpace(frame);
    if (result.HasError()) {
        AERROR << "StageParking planning error";
        return result.SetStageStatus(StageStatusType::ERROR);
    }
    return result.SetStageStatus(StageStatusType::RUNNING);
}

StageResult StageBestParking::FinishStage() {
    return StageResult(StageStatusType::FINISHED);
}

}  // namespace planning
}  // namespace apollo
