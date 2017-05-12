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

#ifndef NETFT_ETHERCAT_HARDWARE__NETFT_H
#define NETFT_ETHERCAT_HARDWARE__NETFT_H

#include "netft_rdt_driver/netft_rdt_driver.h"
#include "ethercat_hardware/ethercat_device.h"
#include "realtime_tools/realtime_publisher.h"
#include "geometry_msgs/WrenchStamped.h"

namespace netft_ethercat_hardware
{


/**
   @class netft_ethercat_hardware::NetFT
   @author Derek King
   @brief NetFT driver plugin for ethercat_hardware

   NetFT is analog converter for Force/Torque sensors from ATI IA.  
   http://www.ati-ia.com/products/ft/ft_NetFT.aspx

   This driver allows data published by NetFT over UDP to be available to 
   realtime controllers as analog inputs.  This driver also publishes data to ROS topic.

   @section ROS ROS interface

   @param address  IPV4 address for NetFT box. Example "192.168.1.1". 
   @param ros_publish_period  Period in seconds that plugin will publish force/torqe data to ROS topic
   @param analog_in_name  Name to use when registering AnalogIn that contains force/torque data 
*/

class NetFT : public EthercatDevice
{
public:
  NetFT();
  ~NetFT();
  void construct(ros::NodeHandle &nh);
  int initialize(pr2_hardware_interface::HardwareInterface *, bool);
  //void packCommand(unsigned char *buffer, bool halt, bool reset);
  bool unpackState(unsigned char *this_buffer, unsigned char *prev_buffer);
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *);
  void collectDiagnostics(EthercatCom *com);

protected:
  pr2_hardware_interface::HardwareInterface *hw_;

  //! NodeHandle to use when getting necessary parameters and publishing
  ros::NodeHandle nh_;
  
  //! Driver interface to NetFT device.
  netft_rdt_driver::NetFTRDTDriver *netft_driver_;
  
  //! AnalogIn struct where device will put data for realtime controllers
  pr2_hardware_interface::AnalogIn analog_in_;

  //! Publish NetFT data to 
  realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped> *pub_;
  realtime_tools::RealtimePublisher<geometry_msgs::Wrench>    *pub_old_;  //!< Wrench message type is deprecated

  //! Last time NetFT data was published to ROS topic.  Used to limit publishing rate.
  ros::Time last_publish_time_;  
  ros::Duration publish_period_;
  bool should_publish_;


  bool tryPublish(const geometry_msgs::WrenchStamped &data);
  bool tryPublishOld(const geometry_msgs::WrenchStamped &data);

  // Double buffer for sharing data between UDP receive thread, and update function (realtime thread)
  //ThreadSafeDoubleBuffer<NetFTAnalog> double_buffer_;
};


}; //end namespace

#endif // NETFT_ETHERCAT_HARDWARE__NETFT_H
