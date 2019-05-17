/*
 * Copyright 2018 Southwest Research Institute
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
 */

#ifndef DESCARTES_OPW_MODEL_H
#define DESCARTES_OPW_MODEL_H

#include "descartes_moveit/moveit_state_adapter.h"
#include "opw_kinematics/opw_parameters.h"

namespace descartes_opw_model
{

class OPWMoveitStateAdapter : public descartes_moveit::MoveitStateAdapter
{
public:
  OPWMoveitStateAdapter(const opw_kinematics::Parameters<double>& kin_params, const std::string& kin_base_frame,
                        const std::string& kin_tool_frame);

  bool initialize(const std::string& robot_description, const std::string& group_name,
                  const std::string& world_frame, const std::string& tcp_frame) override;

  bool getAllIK(const Eigen::Affine3d& pose, std::vector<std::vector<double> >& joint_poses) const override;

  bool getIK(const Eigen::Affine3d& pose, const std::vector<double>& seed_state,
             std::vector<double>& joint_pose) const override;

  bool getFK(const std::vector<double>& joint_pose, Eigen::Affine3d& pose) const override;

  /**
   * @brief Sets the internal state of the robot model to the argument. For the IKFast impl,
   * it also recomputes the transformations to/from the IKFast reference frames.
   */
  void setState(const moveit::core::RobotState& state);

protected:
  bool computeIKFastTransforms();

  /**
   * The IKFast implementation commonly solves between 'base_link' of a robot
   * and 'tool0'. We will commonly want to take advantage of an additional
   * fixed transformation from the robot flange, 'tool0', to some user defined
   * tool. This prevents the user from having to manually adjust tool poses to
   * account for this.
   */
  descartes_core::Frame tool0_to_tip_;

  /**
   * Likewise this parameter is used to accomodate transformations between the base
   * of the IKFast solver and the base of the MoveIt move group.
   */
  descartes_core::Frame world_to_base_;

  opw_kinematics::Parameters<double> kin_params_;

  std::string kin_base_frame_;
  std::string kin_tool_frame_;
};

}

#endif // DESCARTES_OPW_MODEL_H
