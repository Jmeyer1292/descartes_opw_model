#include "descartes_opw_model/descartes_opw_model.h"
#include <opw_kinematics/opw_kinematics.h>
#include <opw_kinematics/opw_utilities.h>

// Compute the 'joint distance' between two poses
static double distance(const std::vector<double>& a, const std::vector<double>& b)
{
  double cost = 0.0;
  for (size_t i = 0; i < a.size(); ++i)
    cost += std::abs(b[i] - a[i]);
  return cost;
}

// Compute the index of the closest joint pose in 'candidates' from 'target'
static size_t closestJointPose(const std::vector<double>& target, const std::vector<std::vector<double>>& candidates)
{
  size_t closest = 0;  // index into candidates
  double lowest_cost = std::numeric_limits<double>::max();
  for (size_t i = 0; i < candidates.size(); ++i)
  {
    assert(target.size() == candidates[i].size());
    double c = distance(target, candidates[i]);
    if (c < lowest_cost)
    {
      closest = i;
      lowest_cost = c;
    }
  }
  return closest;
}

descartes_opw_model::OPWMoveitStateAdapter::OPWMoveitStateAdapter(const opw_kinematics::Parameters<double> &kin_params,
                                                                  const std::string &kin_base_frame,
                                                                  const std::string &kin_tool_frame)
  : kin_params_(kin_params)
  , kin_base_frame_(kin_base_frame)
  , kin_tool_frame_(kin_tool_frame)
{
}

bool descartes_opw_model::OPWMoveitStateAdapter::initialize(const std::string &robot_description,
                                                            const std::string &group_name,
                                                            const std::string &world_frame,
                                                            const std::string &tcp_frame)
{
  if (!MoveitStateAdapter::initialize(robot_description, group_name, world_frame, tcp_frame))
  {
    return false;
  }

  return computeIKFastTransforms();
}

bool descartes_opw_model::OPWMoveitStateAdapter::getAllIK(const Eigen::Affine3d &pose,
                                                          std::vector<std::vector<double>> &joint_poses) const
{
  joint_poses.clear();

  // Transform input pose
  Eigen::Affine3d tool_pose = world_to_base_.frame_inv * pose * tool0_to_tip_.frame;

  std::array<double, 6*8> sols;
  opw_kinematics::inverse(kin_params_, tool_pose, sols.data());

  // Check the output
  std::vector<double> tmp (6); // temporary storage for API reasons
  for (int i = 0; i < 8; i++)
  {
    double* sol = sols.data() + 6 * i;
    if (opw_kinematics::isValid(sol))
    {
      opw_kinematics::harmonizeTowardZero(sol);

      // TODO: make this better...
      std::copy(sol, sol + 6, tmp.data());
      if (isValid(tmp))
      {
        joint_poses.push_back(tmp);
      }
    }
  }

  return joint_poses.size() > 0;
}

bool descartes_opw_model::OPWMoveitStateAdapter::getIK(const Eigen::Affine3d &pose,
                                                       const std::vector<double> &seed_state,
                                                       std::vector<double> &joint_pose) const
{
  // Descartes Robot Model interface calls for 'closest' point to seed position
  std::vector<std::vector<double>> joint_poses;
  if (!getAllIK(pose, joint_poses))
    return false;
  // Find closest joint pose; getAllIK() does isValid checks already
  joint_pose = joint_poses[closestJointPose(seed_state, joint_poses)];
  return true;
}

bool descartes_opw_model::OPWMoveitStateAdapter::getFK(const std::vector<double> &joint_pose,
                                                       Eigen::Affine3d &pose) const
{
  if (!isValid(joint_pose)) // TODO: Why is this a thing?
    return false;

  pose = opw_kinematics::forward<double>(kin_params_, joint_pose.data());
  pose = world_to_base_.frame * pose * tool0_to_tip_.frame_inv;
  return true;
}

void descartes_opw_model::OPWMoveitStateAdapter::setState(const moveit::core::RobotState &state)
{
  descartes_moveit::MoveitStateAdapter::setState(state);
  computeIKFastTransforms();
}

bool descartes_opw_model::OPWMoveitStateAdapter::computeIKFastTransforms()
{
  // look up the IKFast base and tool frame
  if (!robot_state_->knowsFrameTransform(kin_base_frame_))
  {
    logError("IkFastMoveitStateAdapter: Cannot find transformation to frame '%s' in group '%s'.",
             kin_base_frame_.c_str(), group_name_.c_str());
    return false;
  }

  if (!robot_state_->knowsFrameTransform(kin_tool_frame_))
  {
    logError("IkFastMoveitStateAdapter: Cannot find transformation to frame '%s' in group '%s'.",
             kin_tool_frame_.c_str(), group_name_.c_str());
    return false;
  }

  // calculate frames
  tool0_to_tip_ = descartes_core::Frame(robot_state_->getFrameTransform(tool_frame_).inverse() *
                                        robot_state_->getFrameTransform(kin_tool_frame_));

  world_to_base_ = descartes_core::Frame(world_to_root_.frame * robot_state_->getFrameTransform(kin_base_frame_));

  return true;
}
