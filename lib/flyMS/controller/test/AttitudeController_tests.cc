#include "flyMS/common/constants.h"
#include "flyMS/controller/AttitudeController.h"
#include "flyMS/util/yaml_serialization.h"
#include "gtest/gtest.h"

namespace {

/// @brief Clamps values by limits set by maxes
/// @param maxes vales will be clamped to be in the range -maxes to maxes
/// @param values values to clamp
void clamp_by_effort(const std::array<float, 3>& maxes, std::vector<float>& values) {
  if (values.size() <= maxes.size()) {
    throw std::invalid_argument("values needs to be > args in clamp_by_effort");
  }
  auto clamp_value = [](const auto max, auto& value) { value = std::clamp(value, -max, max); };

  clamp_value(maxes[0], values[flyMS::kFLYMS_ROLL_INDEX]);
  clamp_value(maxes[1], values[flyMS::kFLYMS_PITCH_INDEX]);
  clamp_value(maxes[2], values[flyMS::kFLYMS_YAW_INDEX]);
};

}  // namespace

class AttitudeControllerTestFixture : public ::testing::Test {
 public:
  flyMS::AttitudeControllerConfig make_simple_p_config() {
    return flyMS::AttitudeControllerConfig{
        .roll_PID_inner = just_p,
        .roll_PID_outer = just_p,
        .pitch_PID_inner = just_p,
        .pitch_PID_outer = just_p,
        .yaw_PID = just_p,
        .max_control_effort = max_effort,
        .pid_LPF_const_sec = 1.,
    };
  }
  flyMS::AttitudeController make_simple_p_controller() { return flyMS::AttitudeController(make_simple_p_config()); }

  std::array<double, 3> just_p{1., 0., 0.};     // Sample p controller
  std::array<float, 3> max_effort{1., 2., 3.};  // Sample max_efforts
};

TEST_F(AttitudeControllerTestFixture, Ctor) {
  flyMS::AttitudeControllerConfig conf{.pid_LPF_const_sec = 1.};
  flyMS::AttitudeController controller(conf);
}

TEST_F(AttitudeControllerTestFixture, YamlCtor) {
  flyMS::AttitudeControllerConfig conf{.pid_LPF_const_sec = 1.};
  YAML::Node conf_yml = flyMS::to_yaml(conf);
  flyMS::AttitudeController controller(conf_yml);
}

TEST_F(AttitudeControllerTestFixture, BadLpfCtor) {
  flyMS::AttitudeControllerConfig conf{.pid_LPF_const_sec = 0.};
  EXPECT_THROW(flyMS::AttitudeController controller(conf), std::invalid_argument);
}

TEST_F(AttitudeControllerTestFixture, ZeroPids) {
  flyMS::AttitudeControllerConfig conf{.pid_LPF_const_sec = 1.};
  flyMS::AttitudeController controller(conf);
  controller.zero_pids();
}

TEST_F(AttitudeControllerTestFixture, SimplePController) {
  // Test both stabilized and acro flight modes. Also test making them from both the struct and yaml node
  flyMS::AttitudeController controller_stab = make_simple_p_controller();
  YAML::Node controller_acro_config = flyMS::to_yaml(make_simple_p_config());
  flyMS::AttitudeController controller_acro(controller_acro_config);

  const int num_iterations = 10;
  for (auto i = 0; i < num_iterations; i++) {
    flyMS::TRPY<float> setpoints;

    // Assign random values to setpoints
    for (auto& setpoint : setpoints.vector()) {
      setpoint = static_cast<double>(std::rand()) / RAND_MAX * 1.5;
    }

    auto state = flyMS::StateData::zeros();
    auto output_stab = controller_stab.calculate_control_loop(setpoints, state, flyMS::FlightMode::STABILIZED);
    auto output_acro = controller_acro.calculate_control_loop(setpoints, state, flyMS::FlightMode::ACRO);

    clamp_by_effort(max_effort, setpoints.vector());

    for (auto j = 1; j < 4; j++) {
      EXPECT_FLOAT_EQ(output_stab[j], setpoints[j]);
      EXPECT_FLOAT_EQ(output_acro[j], setpoints[j]);
    }
  }
}
