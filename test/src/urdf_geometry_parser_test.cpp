#include <ros/ros.h>
#include <urdf_geometry_parser/urdf_geometry_parser.h>

TEST_F(UrdfGeometryParserTest, testRadius)
{
  ros::NodeHandle nh;
  std::string base_frame_id = "base_link";
  float wheel_radius;

  urdf_geometry_parser::UrdfGeometryParser uvk(nh, base_frame_id);

//    if(!uvk.getDistanceBetweenJoints(front_steering_names[0], front_steering_names[1], track_))
//      return false;
//    else
//      nh.setParam("track",track_);

  bool result = uvk.getJointRadius("front_left_wheel", wheel_radius);

//    if(!uvk.getDistanceBetweenJoints(front_wheel_names[0], rear_wheel_names[0], wheel_base_))
//      return false;
//    else
//      nh.setParam("wheel_base",wheel_base_);
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "urdf_geometry_parser_test");

  int ret = RUN_ALL_TESTS();

  return ret;
}
