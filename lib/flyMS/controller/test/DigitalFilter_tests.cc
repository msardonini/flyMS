/**
 * @file DigitalFilter_tests.cc
 * @author Mike Sardonini
 * @brief Unit tests for the DigitalFilter class
 * @version 0.1
 * @date 2022-09-21
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "flyMS/controller/DigitalFilter.h"
#include "gtest/gtest.h"
#include "spdlog/spdlog.h"

TEST(DigitalFilterTests, Ctor) { flyMS::DigitalFilter({1., 0.}, {1., 0.}); }

TEST(DigitalFilterTests, EmptyCtor) { EXPECT_THROW(flyMS::DigitalFilter({}, {}), std::invalid_argument); }
TEST(DigitalFilterTests, InvalidCtor) { EXPECT_THROW(flyMS::DigitalFilter({1.}, {1., 0.}), std::invalid_argument); }

TEST(DigitalFilterTests, LowPass) {
  std::vector<double> num{0.156832694556443,  1.427422676153595,  5.976558883724950,  15.145889466394246,
                          25.712846449561717, 30.545847498393631, 25.712846449561738, 15.145889466394268,
                          5.976558883724963,  1.427422676153600,  0.156832694556444};
  std::vector<double> den{1.000000000000000,  5.633261445575803,  15.452644550671394, 26.460962250802702,
                          31.096445930527029, 26.068432134671180, 15.818327061618843, 6.914360323080047,
                          2.147477269285763,  0.453950599328629,  0.058793907152724};

  flyMS::DigitalFilter filt(num, den);

  for (auto i = 0; i < 50; i++) {
    filt.update_filter(1.);
  }

  // The lowpass filter will approach 1 and level off and stabilize somewhere <1 but close
  EXPECT_NEAR(filt.update_filter(1.), 1.0, 0.05);
  EXPECT_LT(filt.update_filter(1.), 1.0);

  filt.zero_values();

  EXPECT_EQ(filt.update_filter(0.), 0.);
}

TEST(DigitalFilterTests, Integrator) {
  flyMS::DigitalFilter filt({0.f, 1.f}, {1.f, -1.f});

  for (auto i = 0; i < 50; i++) {
    EXPECT_EQ(i, filt.update_filter(1.));
  }
}

TEST(DigitalFilterTests, InitCtor) {
  flyMS::DigitalFilter filt;
  filt.init({0.f, 1.f}, {1.f, -1.f});

  for (auto i = 0; i < 50; i++) {
    EXPECT_EQ(i, filt.update_filter(1.));
  }
}

TEST(DigitalFilterTests, SimpleIntegratorTest) {
  flyMS::DigitalFilter filt({0.f, 1.f}, {1.f, -1.f});

  for (auto i = 0; i < 1000; i++) {
    EXPECT_DOUBLE_EQ(i, filt.update_filter(1.));
  }
}

TEST(DigitalFilterTests, NonLeadingOneInDen) {
  // This test issues warnings, turn off spdlog for this test
  auto level = spdlog::get_level();
  spdlog::set_level(spdlog::level::off);

  // By convention, the denominator should have a leading one. Test the edge case where it's not
  flyMS::DigitalFilter filt({0.f, 5.f}, {5.f, -5.f});

  for (auto i = 0; i < 10; i++) {
    EXPECT_DOUBLE_EQ(i, filt.update_filter(1.));
  }

  spdlog::set_level(level);
}
