////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Laelaps Mobile Robot ROS Package
//
// Link:      https://github.com/roadnarrows-robotics/laelaps
//
// ROS Node:  laelaps_control
//
// File:      laelaps_control.cpp
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief The ROS laelaps_control node class implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2015  RoadNarrows
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
 * 
 * Permission is hereby granted, without written agreement and without
 * license or royalty fees, to use, copy, modify, and distribute this
 * software and its documentation for any purpose, provided that
 * (1) The above copyright notice and the following two paragraphs
 * appear in all copies of the source code and (2) redistributions
 * including binaries reproduces these notices in the supporting
 * documentation.   Substantial modifications to this software may be
 * copyrighted by their authors and need not follow the licensing terms
 * described here, provided that the new terms are clearly indicated in
 * all files where they apply.
 * 
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
 * OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
 * PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
 * EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * 
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

//
// System
//
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include <string>
#include <map>

//
// Boost libraries
//
#include <boost/bind.hpp>

//
// ROS
//
#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"

//
// ROS generated core and industrial, and laelaps messages.
//
#include "sensor_msgs/JointState.h"
#include "industrial_msgs/RobotStatus.h"

//
// ROS generated Laelaps messages.
//
#include "laelaps_control/Dynamics.h"
#include "laelaps_control/RobotStatusExtended.h"
#include "laelaps_control/Gpio.h"
#include "laelaps_control/Velocity.h"

//
// ROS generatated Laelaps services.
//
#include "laelaps_control/ConfigGpio.h"
#include "laelaps_control/EStop.h"
#include "laelaps_control/Freeze.h"
#include "laelaps_control/GetCaps.h"
#include "laelaps_control/GetProductInfo.h"
#include "laelaps_control/Go.h"
#include "laelaps_control/IsAlarmed.h"
#include "laelaps_control/IsDescLoaded.h"
#include "laelaps_control/ReadGpio.h"
#include "laelaps_control/Release.h"
#include "laelaps_control/ReloadConfig.h"
#include "laelaps_control/ResetEStop.h"
#include "laelaps_control/SetRobotMode.h"
#include "laelaps_control/Stop.h"
#include "laelaps_control/WriteGpio.h"

//
// ROS generated action servers.
//

//
// RoadNarrows
//
#include "rnr/rnrconfig.h"
#include "rnr/log.h"

//
// RoadNarrows embedded laelaps library.
//
#include "Laelaps/laelaps.h"
#include "Laelaps/laeXmlCfg.h"
#include "Laelaps/laeUtils.h"
#include "Laelaps/laeRobot.h"

//
// Node headers.
//
#include "laelaps_control.h"


using namespace std;
using namespace laelaps;
using namespace laelaps_control;


//------------------------------------------------------------------------------
// LaelapsControl Class
//------------------------------------------------------------------------------

LaelapsControl::LaelapsControl(ros::NodeHandle &nh, double hz) :
    m_nh(nh), m_hz(hz)
{
}

LaelapsControl::~LaelapsControl()
{
  disconnect();
}

int LaelapsControl::configure(const string &strCfgFile)
{
  LaeXmlCfg xml;  // laelaps xml instance
  int       rc;   // return code

  if((rc = xml.load(m_robot.getLaelapsDesc(), LaeSysCfgPath, strCfgFile)) < 0)
  {
    ROS_ERROR("Loading XML file '%s' failed.", strCfgFile.c_str());
  }

  else if( (rc = m_robot.getLaelapsDesc().markAsDescribed()) < 0 )
  {
    ROS_ERROR("Failed to finalize descriptions.");
  }

  else
  {
    ROS_INFO("Laelaps description loaded:\n\t %s\n\t %s",
       xml.getFileName().c_str(),
       m_robot.getLaelapsDesc().getFullProdBrief().c_str());
    rc = LAE_OK;
  }

  return rc;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Services
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

void LaelapsControl::advertiseServices()
{
  string  strSvc;

  //strSvc = "calibrate";
  //m_services[strSvc] = m_nh.advertiseService(strSvc,
  //                                        &LaelapsControl::calibrate,
  //                                        &(*this));

  strSvc = "config_gpio";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &LaelapsControl::configGpio,
                                          &(*this));
 
  strSvc = "estop";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &LaelapsControl::estop,
                                          &(*this));

  strSvc = "freeze";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &LaelapsControl::freeze,
                                          &(*this));

  strSvc = "get_capabilites";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &LaelapsControl::getCaps,
                                          &(*this));

  strSvc = "get_product_info";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &LaelapsControl::getProductInfo,
                                          &(*this));

  strSvc = "go";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &LaelapsControl::go,
                                          &(*this));

  strSvc = "is_alarmed";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &LaelapsControl::isAlarmed,
                                          &(*this));

  strSvc = "is_desc_loaded";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &LaelapsControl::isDescLoaded,
                                          &(*this));

  strSvc = "read_gpio";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &LaelapsControl::readGpio,
                                          &(*this));

  strSvc = "release";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &LaelapsControl::release,
                                          &(*this));

  strSvc = "reload_config";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &LaelapsControl::reloadConfig,
                                          &(*this));

  strSvc = "reset_estop";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &LaelapsControl::resetEStop,
                                          &(*this));

  strSvc = "set_robot_mode";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &LaelapsControl::setRobotMode,
                                          &(*this));

  strSvc = "stop";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &LaelapsControl::stop,
                                          &(*this));

  strSvc = "write_gpio";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &LaelapsControl::writeGpio,
                                          &(*this));
}

bool LaelapsControl::configGpio(ConfigGpio::Request  &req,
                                  ConfigGpio::Response &rsp)
{
  const char *svc = "config_gpio";
  int         rc;

  ROS_DEBUG("%s", svc);

  // RDK rc = m_robot.x();

  if( rc == LAE_OK )
  {
    ROS_INFO("Configured GPIO.");
    return true;
  }
  else
  {
    ROS_ERROR("%s failed. %s(rc=%d).", svc, getStrError(rc), rc);
    return false;
  }
}

bool LaelapsControl::estop(EStop::Request  &req,
                             EStop::Response &rsp)
{
  const char *svc = "estop";

  ROS_DEBUG("%s", svc);

  m_robot.estop();

  ROS_INFO("ESTOPPED! You must issue a \"reset_estop\" to continue.");

  return true;
}

bool LaelapsControl::freeze(Freeze::Request  &req,
                              Freeze::Response &rsp)
{
  const char *svc = "freeze";

  ROS_DEBUG("%s", svc);

  m_robot.freeze();

  ROS_INFO("Robot position frozen.");

  return true;
}

bool LaelapsControl::getCaps(GetCaps::Request  &req,
                             GetCaps::Response &rsp)
{
  const char *svc = "get_capabilites";

  ROS_DEBUG("%s", svc);

  if( !m_robot.isDescribed() )
  {
    ROS_ERROR("%s failed: "
              "Robot description not loaded - unable to determine info.",
              svc);
    return false;
  }

  // RDK TODO
 
  return true;
}

bool LaelapsControl::getProductInfo(GetProductInfo::Request  &req,
                                    GetProductInfo::Response &rsp)
{
  const char *svc = "get_product_info";

  int   nMajor, nMinor, nRev;
  char  s[128];

  ROS_DEBUG("%s", svc);

  if( !m_robot.isDescribed() )
  {
    ROS_ERROR("%s failed: "
              "Robot description not loaded - unable to determine info.",
              svc);
    return false;
  }

  m_robot.getVersion(nMajor, nMinor, nRev);

  rsp.i.maj             = nMajor;
  rsp.i.min             = nMinor;
  rsp.i.rev             = nRev;
  rsp.i.version_string  = m_robot.getVersion();
  rsp.i.product_id      = m_robot.getProdId();
  rsp.i.product_name    = m_robot.getProdName();
  rsp.i.desc            = m_robot.getFullProdBrief();

  if( gethostname(s, sizeof(s)) < 0 )
  {
    strcpy(s, "laelaps");
  }
  s[sizeof(s)-1] = 0;

  rsp.i.hostname  = s;

  return true;
}

bool LaelapsControl::go(Go::Request  &req,
                        Go::Response &rsp)
{
  const char *svc = "go";

  ROS_DEBUG("%s", svc);

  if( !m_robot.isDescribed() )
  {
    ROS_ERROR("%s failed: "
              "Robot description not loaded - unable to determine info.",
              svc);
    return false;
  }

  // RDK TODO
 
  return true;
}

bool LaelapsControl::isAlarmed(IsAlarmed::Request  &req,
                                 IsAlarmed::Response &rsp)
{
  const char *svc = "is_alarmed";

  ROS_DEBUG("%s", svc);

  rsp.is_alarmed = m_robot.isAlarmed();

  if( rsp.is_alarmed )
  {
    ROS_WARN("Laelaps is alarmed.");
  }

  return true;
}

bool LaelapsControl::isDescLoaded(IsDescLoaded::Request  &req,
                                  IsDescLoaded::Response &rsp)
{
  const char *svc = "is_desc_loaded";

  ROS_DEBUG("%s", svc);

  rsp.is_desc_loaded = m_robot.getLaelapsDesc().isDescribed();

  if( !rsp.is_desc_loaded )
  {
    ROS_WARN("Laelaps description file not loaded.");
  }

  return true;
}

bool LaelapsControl::readGpio(ReadGpio::Request  &req,
                              ReadGpio::Response &rsp)
{
  const char *svc = "read_gpio";
  int         rc;

  ROS_DEBUG("%s", svc);

  // RDK rc = m_robot.x();

  if( rc == LAE_OK )
  {
    ROS_INFO("Read GPIO.");
    return true;
  }
  else
  {
    ROS_ERROR("%s failed. %s(rc=%d).", svc, getStrError(rc), rc);
    return false;
  }
}

bool LaelapsControl::release(Release::Request  &req,
                             Release::Response &rsp)
{
  const char *svc = "release";

  ROS_DEBUG("%s", svc);

  m_robot.release();

  ROS_INFO("Robot released, motors are undriven.");

  return true;
}

bool LaelapsControl::reloadConfig(Release::Request  &req,
                                  Release::Response &rsp)
{
  const char *svc = "reload_config";

  ROS_DEBUG("%s", svc);

  m_robot.reload();

  ROS_INFO("Robot configuration reloaded.");

  return true;
}

bool LaelapsControl::resetEStop(ResetEStop::Request  &req,
                                  ResetEStop::Response &rsp)
{
  const char *svc = "reset_estop";

  ROS_DEBUG("%s", svc);

  m_robot.resetEStop();

  ROS_INFO("EStop reset.");

  return true;
}

bool LaelapsControl::setRobotMode(SetRobotMode::Request  &req,
                                    SetRobotMode::Response &rsp)
{
  const char *svc = "set_robot_mode";

  ROS_DEBUG("%s", svc);

  m_robot.setRobotMode((LaeRobotMode)req.mode.val);

  ROS_INFO("Robot mode set to %d.", req.mode.val);

  return true;
}

bool LaelapsControl::stop(Stop::Request  &req,
                          Stop::Response &rsp)
{
  const char *svc = "stop";
  int         rc;

  ROS_DEBUG("%s", svc);

  ROS_INFO("Stop");

  rc = m_robot.stop();

  if( rc == LAE_OK )
  {
    ROS_INFO("Stopped.");
    return true;
  }
  else
  {
    ROS_ERROR("%s failed. %s(rc=%d).", svc, getStrError(rc), rc);
    return false;
  }
}

bool LaelapsControl::writeGpio(WriteGpio::Request  &req,
                                 WriteGpio::Response &rsp)
{
  const char *svc = "write_gpio";
  int         rc;

  ROS_DEBUG("%s", svc);

  // RDK rc = m_robot.x();

  if( rc == LAE_OK )
  {
    ROS_INFO("Wrote GPIO.");
    return true;
  }
  else
  {
    ROS_ERROR("%s failed. %s(rc=%d).", svc, getStrError(rc), rc);
    return false;
  }
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Topic Publishers
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

void LaelapsControl::advertisePublishers(int nQueueDepth)
{
  string  strPub;

  strPub = "dynamics";
  m_publishers[strPub] =
    m_nh.advertise<Dynamics>(strPub, nQueueDepth);

  strPub = "robot_status";
  m_publishers[strPub] =
    m_nh.advertise<industrial_msgs::RobotStatus>(strPub, nQueueDepth);

  strPub = "robot_status_ex";
  m_publishers[strPub] =
    m_nh.advertise<RobotStatusExtended>(strPub, nQueueDepth);
}

void LaelapsControl::publish()
{
  publishDynamics();
  publishRobotStatus();
}

void LaelapsControl::publishDynamics()
{
  LaeRptDynamics  dynamics;

  // get robot's extended joint state.
  m_robot.getDynamics(dynamics);
  
  // update joint state message
  updateDynamicsMsg(dynamics, m_msgDynamics);

  // publish joint state messages
  m_publishers["dynamics"].publish(m_msgDynamics);
}

void LaelapsControl::publishRobotStatus()
{
  LaeRptRobotStatus status;   // really status 

  // get robot's extended status.
  m_robot.getRobotStatus(status);

  // update robot status message
  updateRobotStatusMsg(status, m_msgRobotStatus);

  // publish robot status message
  m_publishers["robot_status"].publish(m_msgRobotStatus);

  // update extended robot status message
  updateExtendedRobotStatusMsg(status, m_msgRobotStatusEx);

  // publish extened robot status message
  m_publishers["robot_status_ex"].publish(m_msgRobotStatusEx);
}

void LaelapsControl::updateDynamicsMsg(LaeRptDynamics &dynamics,
                                       Dynamics       &msg)
{
#if 0 // RDK
  //
  // Clear previous joint state data.
  //
  msg.name.clear();
  msg.position.clear();
  msg.velocity.clear();
  msg.effort.clear();

  //
  // Set joint state header.
  //
  msg.header.stamp    = ros::Time::now();
  msg.header.frame_id = "0";
  msg.header.seq++;

  //
  // Set joint state state values;
  //
  for(int n=0; n<state.getNumPoints(); ++n)
  {
    // joint state
    msg.name.push_back(state[n].m_strName);
    msg.position.push_back(state[n].m_fPosition);
    msg.velocity.push_back(state[n].m_fVelocity);
    msg.effort.push_back(state[n].m_fEffort);
  }
#endif // RDK
}

void LaelapsControl::updateRobotStatusMsg(LaeRptRobotStatus &status,
                                          industrial_msgs::RobotStatus &msg)
{
  //
  // Set robot status header.
  //
  msg.header.stamp    = ros::Time::now();
  msg.header.frame_id = "0";
  msg.header.seq++;

  //
  // Set industrial message compliant robot status values.
  //
  msg.mode.val            = status.m_eRobotMode;
  msg.e_stopped.val       = status.m_eIsEStopped;
  msg.drives_powered.val  = status.m_eAreDrivesPowered;
  msg.motion_possible.val = status.m_eIsMotionPossible;
  msg.in_motion.val       = status.m_eIsInMotion;
  msg.in_error.val        = status.m_eIsInError;
  msg.error_code          = status.m_nErrorCode;

}

void LaelapsControl::updateExtendedRobotStatusMsg(LaeRptRobotStatus &status,
                                                  RobotStatusExtended &msg)
{
#if 0 // RDK
  ServoHealth sh;
  int         i;

  //
  // Set extended robot status header.
  //
  msg.header.stamp    = ros::Time::now();
  msg.header.frame_id = "0";
  msg.header.seq++;

  //
  // Set laelaps message extended robot status values.
  //
  msg.mode.val            = status.m_eRobotMode;
  msg.e_stopped.val       = status.m_eIsEStopped;
  msg.drives_powered.val  = status.m_eAreDrivesPowered;
  msg.motion_possible.val = status.m_eIsMotionPossible;
  msg.in_motion.val       = status.m_eIsInMotion;
  msg.in_error.val        = status.m_eIsInError;
  msg.error_code          = status.m_nErrorCode;
  msg.is_calibrated.val   = status.m_eIsCalibrated;

  // clear previous data
  msg.servo_health.clear();

  for(i=0; i<status.m_vecServoHealth.size(); ++i)
  {
    sh.servo_id = (s8_t)status.m_vecServoHealth[i].m_nServoId;
    sh.temp     = status.m_vecServoHealth[i].m_fTemperature;
    sh.voltage  = status.m_vecServoHealth[i].m_fVoltage;
    sh.alarm    = (u8_t)status.m_vecServoHealth[i].m_uAlarms;

    msg.servo_health.push_back(sh);
  }
#endif // RDK
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Subscribed Topics
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

void LaelapsControl::subscribeToTopics(int nQueueDepth)
{
  string  strSub;

  strSub = "set_velocities";
  m_subscriptions[strSub] = m_nh.subscribe(strSub, nQueueDepth,
                                          &LaelapsControl::execSetVelocities,
                                          &(*this));
}

void LaelapsControl::execSetVelocities(const laelaps_control::Velocity &msgVel)
{
  LaeMapVelocity  vel;

  ROS_INFO("Set velocities");

  // load trajectory point
  for(int i=0; i<msgVel.names.size(); ++i)
  {
    vel[msgVel.names[i]] = msgVel.velocities[i];
    ROS_INFO(" %-12s: vel=%7.3lfm/s",
        msgVel.names[i].c_str(), msgVel.velocities[i]);
  }

  m_robot.go(vel);
}
