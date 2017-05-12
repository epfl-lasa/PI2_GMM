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

#include "netft_ethercat_hardware/netft.h"


// Product code = NetFT
// NetFT is not an EtherCAT device.  As such, it does not have EtherCAT productID 
// will not be loaded on demand because EML finds device with matching product ID.
// However, newer version of ethercat_hardware will use rosparam to load non-ethercat 
// plugins.  Non-ethercat plugins can take commands and provide data to realtime system
//PLUGINLIB_REGISTER_CLASS(NetFT, netft_ethercat_hardware::NetFT, EthercatDevice);
PLUGINLIB_DECLARE_CLASS(netft_ethercat_hardware, NetFT, netft_ethercat_hardware::NetFT, EthercatDevice);


namespace netft_ethercat_hardware
{


NetFT::NetFT() :
  hw_(NULL),
  netft_driver_(NULL),
  pub_(NULL),
  pub_old_(NULL),
  publish_period_(0.1)
{
}

NetFT::~NetFT()
{
  delete netft_driver_;
  delete pub_;
  delete pub_old_;
}

void NetFT::construct(ros::NodeHandle &nh)
{
  nh_ = nh;
}

int NetFT::initialize(pr2_hardware_interface::HardwareInterface *hw, bool)
{
  hw_ = hw;

  // Register analog inputs

  // Get device IP address from rosparam
  std::string address;
  if (!nh_.getParam("address", address))
  {
    ROS_ERROR("netft_ethercat_hardware : No param 'address' in namespace %s", nh_.getNamespace().c_str());
    return -1;
  }
 
  // Use rosparm when for select name of AnalogIn
  if (!nh_.getParam("analog_in_name", analog_in_.name_))
  {
    ROS_ERROR("netft_ethercat_hardware : No param 'analog_in_name' in namespace %s", nh_.getNamespace().c_str());
    return -1;
  }

  if (hw && !hw->addAnalogIn(&analog_in_))
  {
    ROS_FATAL("netft_ethercat_hardware : An analog input with the name '%s' already exists.", 
              analog_in_.name_.c_str());
    return -1;
  }
  analog_in_.state_.state_.resize(6);  // Reserve location for 6 analog values

  
  double publish_period;
  if (!nh_.getParam("ros_publish_period", publish_period))
  {
    ROS_ERROR("netft_ethercat_hardware : No param 'ros_publish_period' in namespace %s", nh_.getNamespace().c_str());
    return -1;
  }
  publish_period_ = ros::Duration(publish_period);
  last_publish_time_ = ros::Time::now();

  bool publish_wrench = false;
  if (!nh_.getParam("publish_wrench", publish_wrench))
  {
    publish_wrench = false;
  }

  if (publish_wrench)
  {
    ROS_WARN("Publishing NetFT data as geometry_msgs::Wrench is deprecated");
    pub_old_ = new realtime_tools::RealtimePublisher<geometry_msgs::Wrench>(nh_, "netft_data", 2);
  }
  else 
  {
    pub_ = new realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>(nh_, "netft_data", 2);
  }

  try 
  {
    netft_driver_ = new netft_rdt_driver::NetFTRDTDriver(address);
  }
  catch (std::exception &e)
  {
    ROS_ERROR("netft_ethercat_hardware : Error constructing NetFT driver : %s", e.what());
    return -1;
  }

  return 0;
}


bool NetFT::unpackState(unsigned char *, unsigned char *)
{
  // Take most recent UDP data and move to analog inputs
  geometry_msgs::WrenchStamped data;
  netft_driver_->getData(data);

  // Update analog inputs
  analog_in_.state_.state_.resize(6);
  analog_in_.state_.state_[0] = data.wrench.force.x;
  analog_in_.state_.state_[1] = data.wrench.force.y;
  analog_in_.state_.state_[2] = data.wrench.force.z;
  analog_in_.state_.state_[3] = data.wrench.torque.x;
  analog_in_.state_.state_[4] = data.wrench.torque.y;
  analog_in_.state_.state_[5] = data.wrench.torque.z;

  if ( (hw_->current_time_ - last_publish_time_) > publish_period_ )
  {
    last_publish_time_ += publish_period_;
    should_publish_ = true;
  }
  
  if (should_publish_)
  {
    if (tryPublish(data) || tryPublishOld(data))
    {
      should_publish_ = false;
    }
  }

  return true;
}

bool NetFT::tryPublish(const geometry_msgs::WrenchStamped &data)
{
  if (pub_ == NULL)
  {
    return false;
  }
  if (!pub_->trylock())
  {
    return false;
  }
   
  pub_->msg_ = data;
  pub_->unlockAndPublish();
  return true;
}


bool NetFT::tryPublishOld(const geometry_msgs::WrenchStamped &data)
{
  if (pub_old_ == NULL)
  {
    return false;
  }
  if (!pub_old_->trylock())
  {
    return false;
  }

  pub_old_->msg_ = data.wrench;
  pub_old_->unlockAndPublish();
  return true;
}



void NetFT::collectDiagnostics(EthercatCom *)
{
  // Don't allow default collection of diagnostics
}

void NetFT::diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *)
{
  netft_driver_->diagnostics(d);
  d.add("AnalogIn name", analog_in_.name_);
}
  
};
