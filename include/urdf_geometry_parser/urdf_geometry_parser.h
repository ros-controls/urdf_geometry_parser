#ifndef URDF_GEOMETRY_PARSER_URDF_GEOMETRY_PARSER_H
#define URDF_GEOMETRY_PARSER_URDF_GEOMETRY_PARSER_H

#include <ros/ros.h>

#include <urdf_parser/urdf_parser.h>

namespace urdf_geometry_parser {

class UrdfUtilities {

public:
  UrdfUtilities(ros::NodeHandle& root_nh, const std::string &base_link);

  /**
   * \brief Get transform vector between the joint and parent_link
   * \param [in] joint_name   Child joint_name from where to begin
   * \param [in] parent_link_name Name of link to find as parent of the joint
   * \param [out] transform_vector Distance vector between joint and parent_link [m]
   * \return true if the transform vector was found; false otherwise
   */
  bool getTransformVector(const std::string& joint_name,
                          const std::string& parent_link_name,
                          urdf::Vector3& transform_vector);

  /**
   * \brief Get distance between two joints from the URDF
   * \param first_joint_name Name of the first joint
   * \param second_joint_name Name of the second joint
   * \param distance Distance in meter between first and second joint
   */
  bool getDistanceBetweenJoints(const std::string& first_joint_name,
                                const std::string& second_joint_name,
                                double& distance);

  /**
   * \brief Get joint radius from the URDF
   * \param joint_name Name of the joint
   * \param distance Distance in meter between first and second joint
   */
  bool getJointRadius(const std::string& joint_name,
                                double& radius);

  /**
   * \brief Get joint steering limit from the URDF
   *        considering the upper and lower limit is the same
   * \param joint_name Name of the joint
   * \param steering_limit [rad]
   */
  bool getJointSteeringLimits(const std::string& joint_name,
                              double& steering_limit);
private:
  std::string base_link_;

  boost::shared_ptr<urdf::ModelInterface> model_;
};

}

#endif
