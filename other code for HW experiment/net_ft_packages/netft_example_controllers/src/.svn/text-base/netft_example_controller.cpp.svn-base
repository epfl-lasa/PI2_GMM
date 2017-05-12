/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "netft_example_controllers/netft_example_controller.h"
#include "pluginlib/class_list_macros.h"
#include <math.h>
#include <boost/foreach.hpp>
#include <string>
#include <algorithm>

PLUGINLIB_DECLARE_CLASS(netft_example_controllers, NetFTExampleController, netft_example_controllers::NetFTExampleController, pr2_controller_interface::Controller)


namespace netft_example_controllers
{

NetFTExampleController::NetFTExampleController() : 
  max_force_(0.0),
  max_torque_(0.0),
  analog_in_(NULL),
  pub_cycle_count_(0),
  should_publish_(false)
{  
  // Nothing
}


NetFTExampleController::~NetFTExampleController() 
{
  // Nothing
}


bool NetFTExampleController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &node)
{
  if (!robot) 
    return false;
    
  std::string analog_in_name;
  if (!node.getParam("analog_in_name", analog_in_name))
  {
    ROS_ERROR("NetFTExampleController: No \"analog_in_name\" found on parameter namespace: %s",
              node.getNamespace().c_str());
    return false;
  }
  
  pr2_hardware_interface::HardwareInterface* hw = robot->model_->hw_;  
  analog_in_ = hw->getAnalogIn(analog_in_name);
  if (analog_in_ == NULL)
  {
    ROS_ERROR("NetFTExampleController: Cannot find AnalogIn named \"%s\"",
              analog_in_name.c_str());
    BOOST_FOREACH(const pr2_hardware_interface::AnalogInMap::value_type &v, hw->analog_ins_)
    {
      ROS_INFO("AnalogIn : %s", v.first.c_str());
    }
    return false;
  }
  ROS_INFO("NetFTExampleController: Using AnalogIn named \"%s\"", analog_in_name.c_str());
  
  // Initialize realtime publisher to publish to ROS topic 
  pub_.init(node, "force_torque_stats", 2);

  return true;
}


void NetFTExampleController::starting()
{
  // Nothing to do here
}


void NetFTExampleController::update()
{
  if (analog_in_->state_.state_.size() != 6)
  {
    ROS_ERROR_THROTTLE(5.0, "NetFTExampleController: AnalogInput is has unexpected size %d", 
                       int(analog_in_->state_.state_.size()));
    return;
  }

  double fx = analog_in_->state_.state_[0];
  double fy = analog_in_->state_.state_[1];
  double fz = analog_in_->state_.state_[2];
  double tx = analog_in_->state_.state_[3];
  double ty = analog_in_->state_.state_[4];
  double tz = analog_in_->state_.state_[5];

  double abs_force = sqrt( fx*fx + fy*fy + fz*fz );
  double abs_torque = sqrt( tx*tx + ty*ty + tz*tz );
  max_force_ = std::max(max_force_, abs_force);
  max_torque_ = std::max(max_torque_, abs_torque);

  // Publish data in ROS message every 10 cycles (about 100Hz)
  if (++pub_cycle_count_ > 10)
  {
    should_publish_ = true;
    pub_cycle_count_ = 0;
  }

  if (should_publish_ && pub_.trylock())
  {      
    should_publish_ = false;
    pub_.msg_.abs_force = abs_force;
    pub_.msg_.max_force = max_force_;
    pub_.msg_.abs_torque = abs_torque;
    pub_.msg_.max_torque = max_torque_;
    pub_.unlockAndPublish();
  }
}

}//namespace netft_example_controllers
