#include <ros/ros.h>
#include <urdf_geometry_parser/urdf_geometry_parser.h>

#include <gtest/gtest.h>

class UrdfGeometryParserTest : public ::testing::Test{
public:
  UrdfGeometryParserTest() :
    ugp_(nh_, "base_link")
  {}

  ros::NodeHandle nh_;
  urdf_geometry_parser::UrdfGeometryParser ugp_;
};

TEST_F(UrdfGeometryParserTest, testRadius)
{
  double wheel_radius;
  bool result = ugp_.getJointRadius("front_left_wheel", wheel_radius);
  EXPECT_TRUE(result);
  EXPECT_DOUBLE_EQ(wheel_radius, 0.28);
}

TEST_F(UrdfGeometryParserTest, testDistance)
{
  double track, wheel_base;

  bool result_track = ugp_.getDistanceBetweenJoints("front_left_wheel", "front_right_wheel", track);
  EXPECT_TRUE(result_track);
  EXPECT_DOUBLE_EQ(track, 1.1);

  bool result_wb = ugp_.getDistanceBetweenJoints("front_right_wheel", "rear_right_wheel", wheel_base);
  EXPECT_TRUE(result_wb);
  EXPECT_DOUBLE_EQ(wheel_base, 1.9);
}

TEST_F(UrdfGeometryParserTest, testJointSteeringLimits)
{
  double steering_limit;
  bool result = ugp_.getJointSteeringLimits("rear_left_steering_joint", steering_limit);
  EXPECT_TRUE(result);
  EXPECT_NEAR(steering_limit, M_PI/6, 0.001);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "UrdfGeometryParserTestNode");
  return RUN_ALL_TESTS();
}
