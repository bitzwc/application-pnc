/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/prediction/evaluator/model_manager/model/semantic_lstm_pedestrian_torch_gpu/semantic_lstm_pedestrian_torch_model.h"

#include <string>
#include <utility>
#include <vector>

#include "modules/prediction/proto/prediction_conf.pb.h"

#include "cyber/common/file.h"
#include "modules/prediction/evaluator/warm_up/warm_up.h"

namespace apollo {
namespace prediction {

bool SemanticLstmPedestrianGpuTorch::Init() {
  ModelConf model_config;
  int status;

  if (init_ != 0) {
    return true;
  }

  std::string class_name =
      abi::__cxa_demangle(typeid(*this).name(), 0, 0, &status);

  std::string default_config_path =
      apollo::cyber::plugin_manager::PluginManager::Instance()
          ->GetPluginConfPath<ModelBase>(class_name,
                                         "conf/default_conf.pb.txt");

  if (!cyber::common::GetProtoFromFile(default_config_path, &model_config)) {
    AERROR << "Unable to load model conf file: " << default_config_path;
    return false;
  }
  model_path_ = model_config.model_path();
  init_ = 1;

  return LoadModel();
}

bool SemanticLstmPedestrianGpuTorch::LoadModel() {
  auto device = torch::Device(torch::kCPU);
  if (torch::cuda::is_available()) {
    device = torch::Device(torch::kCUDA);
  }

  model_instance_ = torch::jit::load(model_path_, device);

  torch::set_num_threads(1);

  // Fake intput for the first frame
  torch::Tensor img_tensor = torch::randn({1, 3, 224, 224});
  torch::Tensor obstacle_pos = torch::randn({1, 20, 2});
  torch::Tensor obstacle_pos_step = torch::randn({1, 20, 2});
  std::vector<torch::jit::IValue> torch_inputs;
  torch::Tensor torch_default_output_tensor;

  torch_inputs.push_back(c10::ivalue::Tuple::create(
      {std::move(img_tensor.to(device)), std::move(obstacle_pos.to(device)),
       std::move(obstacle_pos_step.to(device))}));

  // warm up to avoid very slow first inference later
  WarmUp(torch_inputs, &model_instance_, &torch_default_output_tensor);
  return true;
}

bool SemanticLstmPedestrianGpuTorch::Inference(
    const std::vector<void*>& input_buffer, unsigned int input_size,
    std::vector<void*>* output_buffer, unsigned int output_size) {
  ACHECK(input_size == input_buffer.size() && input_size == 3);
  ACHECK(output_size == output_buffer->size() && output_size == 1);

  if (init_ == 0) {
    Init();
  }

  auto device = torch::Device(torch::kCPU);
  if (torch::cuda::is_available()) {
    device = torch::Device(torch::kCUDA);
  }
  torch::Tensor img_tensor =
      torch::from_blob(input_buffer[0], {1, 3, 224, 224});
  torch::Tensor obstacle_pos = torch::from_blob(input_buffer[1], {1, 20, 2});
  torch::Tensor obstacle_pos_step =
      torch::from_blob(input_buffer[2], {1, 20, 2});

  std::vector<torch::jit::IValue> torch_inputs;

  torch_inputs.push_back(c10::ivalue::Tuple::create(
      {std::move(img_tensor.to(device)), std::move(obstacle_pos.to(device)),
       std::move(obstacle_pos_step.to(device))}));

  torch::Tensor torch_output_tensor =
      model_instance_.forward(torch_inputs).toTensor().to(torch::kCPU);
  memcpy((*output_buffer)[0], torch_output_tensor.data_ptr<float>(),
         1 * 30 * 2 * sizeof(float));
  return true;
}

void SemanticLstmPedestrianGpuTorch::Destory() {}

}  // namespace prediction
}  // namespace apollo
