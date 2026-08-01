#include "mujoco_micro/policy_runner.hpp"

#include <cmath>
#include <stdexcept>

namespace mujoco_micro
{

void PolicyRunner::load(const std::string & model_path)
{
  if (model_path.empty()) {
    throw std::runtime_error("policy.model_path is empty");
  }
  try {
    network_ = cv::dnn::readNetFromONNX(model_path);
    network_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    network_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
  } catch (const cv::Exception & error) {
    throw std::runtime_error(
            "failed to load ONNX policy '" + model_path + "': " + error.what());
  }

  std::array<float, kObservationSize> zero_observation{};
  float test_action = 0.0F;
  std::string error_message;
  loaded_ = true;
  model_path_ = model_path;
  if (!infer(zero_observation, test_action, error_message)) {
    loaded_ = false;
    throw std::runtime_error(
            "ONNX policy input/output validation failed: " + error_message);
  }
}

bool PolicyRunner::infer(
  const std::array<float, kObservationSize> & observation, float & action,
  std::string & error_message)
{
  if (!loaded_) {
    error_message = "policy is not loaded";
    return false;
  }
  for (std::size_t i = 0; i < observation.size(); ++i) {
    if (!std::isfinite(observation[i])) {
      error_message = "observation contains a non-finite value at index " + std::to_string(i);
      return false;
    }
    input_.at<float>(0, static_cast<int>(i)) = observation[i];
  }

  try {
    network_.setInput(input_);
    const cv::Mat output = network_.forward();
    if (output.type() != CV_32F || output.total() != 1U) {
      error_message = "expected one float32 action, received type=" +
        std::to_string(output.type()) + " count=" + std::to_string(output.total());
      return false;
    }
    action = output.ptr<float>()[0];
  } catch (const cv::Exception & error) {
    error_message = error.what();
    return false;
  }
  if (!std::isfinite(action)) {
    error_message = "policy returned a non-finite action";
    return false;
  }
  return true;
}

void RecoveryPolicyRunner::load(const std::string & model_path)
{
  if (model_path.empty()) {
    throw std::runtime_error("recovery.model_path is empty");
  }
  try {
    network_ = cv::dnn::readNetFromONNX(model_path);
    network_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    network_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
  } catch (const cv::Exception & error) {
    throw std::runtime_error(
            "failed to load recovery ONNX policy '" + model_path + "': " + error.what());
  }

  std::array<float, kObservationSize> zero_observation{};
  std::array<float, kActionSize> test_action{};
  std::string error_message;
  loaded_ = true;
  model_path_ = model_path;
  if (!infer(zero_observation, test_action, error_message)) {
    loaded_ = false;
    throw std::runtime_error(
            "recovery ONNX policy input/output validation failed: " + error_message);
  }
}

bool RecoveryPolicyRunner::infer(
  const std::array<float, kObservationSize> & observation,
  std::array<float, kActionSize> & action, std::string & error_message)
{
  if (!loaded_) {
    error_message = "recovery policy is not loaded";
    return false;
  }
  for (std::size_t i = 0; i < observation.size(); ++i) {
    if (!std::isfinite(observation[i])) {
      error_message = "recovery observation contains a non-finite value at index " +
        std::to_string(i);
      return false;
    }
    input_.at<float>(0, static_cast<int>(i)) = observation[i];
  }

  try {
    network_.setInput(input_);
    const cv::Mat output = network_.forward();
    if (output.type() != CV_32F || output.total() != kActionSize) {
      error_message = "expected three float32 recovery actions, received type=" +
        std::to_string(output.type()) + " count=" + std::to_string(output.total());
      return false;
    }
    for (std::size_t i = 0; i < action.size(); ++i) {
      action[i] = output.ptr<float>()[i];
    }
  } catch (const cv::Exception & error) {
    error_message = error.what();
    return false;
  }
  for (std::size_t i = 0; i < action.size(); ++i) {
    if (!std::isfinite(action[i])) {
      error_message = "recovery policy returned a non-finite action at index " +
        std::to_string(i);
      return false;
    }
  }
  return true;
}

}  // namespace mujoco_micro
