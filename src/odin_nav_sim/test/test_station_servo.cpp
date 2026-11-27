#include "gtest/gtest.h"
#include "station_servo.hpp"

using odin_nav_sim::StationServo;
using odin_nav_sim::StationOffset;

TEST(StationServo, SixRecordedArrivalOffsetsWithFixedObservationBias)
{
  // Rows are measured wheel-centre x/y/yaw and the wall fit x/y/yaw, in m/rad.
  const double trials[][6] = {
    {.0214,-.1046,.1635, .0215,-.1101,.1663},
    {-.0063,-.1644,.1779, .0079,-.1649,.1522},
    {-.0076,-.1764,.1806, .0013,-.1821,.1642},
    {.0341,.0241,-.1901, .0381,.0456,-.1846},
    {.0152,.0160,-.1700, .0229,.0339,-.1864},
    {.0379,.0089,-.1754, .0381,.0187,-.1715}};
  for (const auto & row : trials) {
    StationServo servo;
    StationOffset p{row[0],row[1],row[2]};
    bool still = true, reached = false;
    // Simulate 10 Hz measurements and body-frame integration; preserve this stop's
    // tape-to-observation bias throughout the manoeuvre. This is a model assumption.
    for (int k = 1; k <= 500; ++k) {
      const double t = k * 0.1;
      auto cmd = servo.update(t, t, {p.x+row[3]-row[0], p.y+row[4]-row[1], p.yaw+row[5]-row[2]}, still);
      ASSERT_LE((cmd.vx != 0) + (cmd.vy != 0) + (cmd.wz != 0), 1);
      if (cmd.reached) {reached = true; break;}
      p.x += 0.1*(std::cos(p.yaw)*cmd.vx - std::sin(p.yaw)*cmd.vy);
      p.y += 0.1*(std::sin(p.yaw)*cmd.vx + std::cos(p.yaw)*cmd.vy);
      p.yaw += 0.1*cmd.wz;
      still = !cmd.vx && !cmd.vy && !cmd.wz;
    }
    EXPECT_TRUE(reached);
    EXPECT_LT(std::hypot(p.x, p.y), 0.03);
    EXPECT_LT(std::abs(p.yaw), 5*M_PI/180);
  }
}

TEST(StationServo, RequiresFreshSettledSamplesAndRejectsStallAndJump)
{
  StationServo servo;
  for (int i = 0; i != 20; ++i) {
    EXPECT_FALSE(servo.update(1+i*0.1, 1., {}, true).reached);
  }
  EXPECT_FALSE(servo.update(3., 3., {}, false).reached);
  EXPECT_FALSE(servo.update(3.1, 3.1, {}, true).reached);
  EXPECT_TRUE(servo.update(3.6, 3.6, {}, true).reached);
  EXPECT_THROW(servo.update(3.7, 3.7, {.2,0,0}, true), std::runtime_error);
  StationServo stalled;
  stalled.update(1., 1., {0,-.15,0}, true);
  EXPECT_GT(stalled.update(1.5, 1.5, {0,-.15,0}, true).vy, 0.);
  EXPECT_THROW(stalled.update(6., 6., {0,-.15,0}, true), std::runtime_error);
}
