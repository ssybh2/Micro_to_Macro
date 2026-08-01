#pragma once

#include <array>
#include <string>

#include <opencv2/dnn.hpp>

namespace mujoco_micro
{

class PolicyRunner
{
public:
  static constexpr std::size_t kObservationSize = 11;

  void load(const std::string & model_path);
  bool infer(
    const std::array<float, kObservationSize> & observation, float & action,
    std::string & error_message);

  bool loaded() const noexcept {return loaded_;}
  const std::string & model_path() const noexcept {return model_path_;}

private:
  cv::dnn::Net network_{};
  cv::Mat input_ = cv::Mat(1, static_cast<int>(kObservationSize), CV_32F);
  std::string model_path_{};
  bool loaded_{false};
};

class RecoveryPolicyRunner
{
public:
  static constexpr std::size_t kObservationSize = 24;
  static constexpr std::size_t kActionSize = 3;

  void load(const std::string & model_path);
  bool infer(
    const std::array<float, kObservationSize> & observation,
    std::array<float, kActionSize> & action, std::string & error_message);

  bool loaded() const noexcept {return loaded_;}
  const std::string & model_path() const noexcept {return model_path_;}

private:
  cv::dnn::Net network_{};
  cv::Mat input_ = cv::Mat(1, static_cast<int>(kObservationSize), CV_32F);
  std::string model_path_{};
  bool loaded_{false};
};

}  // namespace mujoco_micro
