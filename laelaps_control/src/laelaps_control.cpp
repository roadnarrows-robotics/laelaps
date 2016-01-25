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
 * (C) 2015-2016  RoadNarrows
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

#include <limits>
#include <string>
#include <vector>
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
// ROS generated core and industrial messages.
//
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "industrial_msgs/TriState.h"
#include "industrial_msgs/RobotStatus.h"
#include "sensor_msgs/Illuminance.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Range.h"

//
// ROS generated Laelaps messages.
//
#include "laelaps_control/Alarms.h"
#include "laelaps_control/AuxPwr.h"
#include "laelaps_control/Caps.h"
#include "laelaps_control/Dimensions.h"
#include "laelaps_control/Dynamics.h"
#include "laelaps_control/Gpio.h"
#include "laelaps_control/IlluminanceState.h"
#include "laelaps_control/ImuAlt.h"
#include "laelaps_control/ImuCaps.h"
#include "laelaps_control/MotorCtlrHealth.h"
#include "laelaps_control/MotorHealth.h"
#include "laelaps_control/Path2D.h"
#include "laelaps_control/Pose2DStamped.h"
#include "laelaps_control/PowertrainCaps.h"
#include "laelaps_control/ProductInfo.h"
#include "laelaps_control/RangeState.h"
#include "laelaps_control/RobotStatusExtended.h"
#include "laelaps_control/RobotTrajectory2D.h"
#include "laelaps_control/RobotTrajectoryPoint2D.h"
#include "laelaps_control/ToFSensorCaps.h"
#include "laelaps_control/Velocity.h"

//
// ROS generatated Laelaps services.
//
#include "laelaps_control/ConfigGpio.h"
#include "laelaps_control/EStop.h"
#include "laelaps_control/Freeze.h"
#include "laelaps_control/GetCaps.h"
#include "laelaps_control/GetImu.h"
#include "laelaps_control/GetImuAlt.h"
#include "laelaps_control/GetProductInfo.h"
#include "laelaps_control/GetRange.h"
#include "laelaps_control/IsAlarmed.h"
#include "laelaps_control/IsDescLoaded.h"
#include "laelaps_control/ReadGpio.h"
#include "laelaps_control/ReadIlluminance.h"
#include "laelaps_control/Release.h"
#include "laelaps_control/ReloadConfig.h"
#include "laelaps_control/ResetEStop.h"
#include "laelaps_control/SetAuxPwr.h"
#include "laelaps_control/SetRobotMode.h"
#include "laelaps_control/SetVelocities.h"
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
#include "Laelaps/laeUtils.h"
#include "Laelaps/laeXmlCfg.h"
#include "Laelaps/laeTraj.h"
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
       m_robot.getLaelapsDesc().getProdBrief().c_str());
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

#if 0 // FUTURE MAYBE
  //strSvc = "calibrate";
  //m_services[strSvc] = m_nh.advertiseService(strSvc,
  //                                        &LaelapsControl::calibrate,
  //                                        &(*this));
#endif // FUTURE

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

#if 0 // FUTURE
  strSvc = "get_caps";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &LaelapsControl::getCaps,
                                          &(*this));
#endif // FUTURE

#if 0 // FUTURE
  strSvc = "get_imu";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &LaelapsControl::getImu,
                                          &(*this));
#endif // FUTURE

  strSvc = "get_imu_alt";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &LaelapsControl::getImuAlt,
                                          this);

  strSvc = "get_range";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &LaelapsControl::getRange,
                                          &(*this));

  strSvc = "get_product_info";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &LaelapsControl::getProductInfo,
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

  strSvc = "read_illuminance";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &LaelapsControl::readIlluminance,
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

  strSvc = "set_aux_power";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &LaelapsControl::setAuxPower,
                                          &(*this));

  strSvc = "set_robot_mode";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &LaelapsControl::setRobotMode,
                                          &(*this));

  strSvc = "set_velocities";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &LaelapsControl::setVelocities,
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

  ROS_DEBUG("%s/%s", m_nh.getNamespace().c_str(), svc);

  for(size_t i = 0; i < req.gpio.size(); ++i)
  {
    rc = m_robot.configDigitalPin(req.gpio[i].pin, req.gpio[i].state);

    if( rc == LAE_OK )
    {
      ROS_INFO("Configured GPIO pin %d as %s.",
          req.gpio[i].pin, (req.gpio[i].state? "output": "input"));
    }
    else
    {
      ROS_ERROR("Service %s failed on pin %d: %s(rc=%d).",
          svc, req.gpio[i].pin, getStrError(rc), rc);
      return false;
    }
  }

  return true;
}

bool LaelapsControl::estop(EStop::Request  &req,
                           EStop::Response &rsp)
{
  const char *svc = "estop";

  ROS_DEBUG("%s/%s", m_nh.getNamespace().c_str(), svc);

  m_robot.estop();

  ROS_INFO("ESTOPPED! You must issue a \"reset_estop\" to continue.");

  return true;
}

bool LaelapsControl::freeze(Freeze::Request  &req,
                              Freeze::Response &rsp)
{
  const char *svc = "freeze";

  ROS_DEBUG("%s/%s", m_nh.getNamespace().c_str(), svc);

  m_robot.freeze();

  ROS_INFO("Robot position frozen.");

  return true;
}

bool LaelapsControl::getCaps(GetCaps::Request  &req,
                             GetCaps::Response &rsp)
{
  const char *svc = "get_caps";
  int         rc;

  ROS_DEBUG("%s/%s", m_nh.getNamespace().c_str(), svc);

  if( !m_robot.isDescribed() )
  {
    ROS_ERROR("Service %s failed: "
              "Robot description not loaded - unable to determine info.",
              svc);
    return false;
  }

  // RDK TODO
  // RDK rc = m_robot.getCaps();
  rc = LAE_OK;
 
  if( rc == LAE_OK )
  {
    ROS_INFO("Robot capabilities retrieved.");
    return true;
  }
  else
  {
    ROS_ERROR("Service %s failed: %s(rc=%d).",
          svc, getStrError(rc), rc);
    return false;
  }
}


bool LaelapsControl::readIlluminance(ReadIlluminance::Request  &req,
                                     ReadIlluminance::Response &rsp)
{
  const char *svc = "get_illuminance";

  double      fIlluminance;
  int         rc;

  ROS_DEBUG("%s/%s", m_nh.getNamespace().c_str(), svc);

  rc = m_robot.readAmbientLight(req.name, fIlluminance);
 
  if( rc == LAE_OK )
  {
    stampHeader(rsp.sensor.header, 0);

    rsp.sensor.illuminance = fIlluminance;
    rsp.sensor.variance    = 0;

    ROS_INFO("Ambient light sensor %s = %.1lf lux.",
        req.name.c_str(), rsp.sensor.illuminance);

    return true;
  }
  else
  {
    ROS_ERROR("Service %s failed on sensor %s: %s(rc=%d).",
          svc, req.name.c_str(), getStrError(rc), rc);
    return false;
  }
}

bool LaelapsControl::getImuAlt(GetImuAlt::Request  &req,
                               GetImuAlt::Response &rsp)
{
  const char *svc = "get_imu_alt";
  double      accel[ImuAlt::NUM_AXES];
  double      gyro[ImuAlt::NUM_AXES];
  double      rpy[ImuAlt::NUM_AXES];
  int         rc;

  ROS_DEBUG("%s/%s", m_nh.getNamespace().c_str(), svc);

  rc = m_robot.getImu(accel, gyro, rpy);

  if( rc == LAE_OK )
  {
    for(int i = 0; i < ImuAlt::NUM_AXES; ++i)
    {
      rsp.imu.accel[i] = accel[i];
      rsp.imu.gyro[i]  = gyro[i];
      rsp.imu.rpy[i]   = rpy[i];
    }

    stampHeader(rsp.imu.header, 0);

    ROS_INFO("IMU %s data.", req.name.c_str());
    return true;
  }
  else
  {
    ROS_ERROR("Service %s failed on IMU %s: %s(rc=%d).",
          svc, req.name.c_str(), getStrError(rc), rc);
    return false;
  }
}

bool LaelapsControl::getProductInfo(GetProductInfo::Request  &req,
                                    GetProductInfo::Response &rsp)
{
  const char *svc = "get_product_info";

  int   nMajor, nMinor, nRev;
  char  s[128];

  ROS_DEBUG("%s/%s", m_nh.getNamespace().c_str(), svc);

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
  rsp.i.version_string  = m_robot.getVersionString();
  rsp.i.product_id      = m_robot.getProdId();
  rsp.i.product_name    = m_robot.getProdName();
  rsp.i.desc            = m_robot.getFullProdBrief();

  if( gethostname(s, sizeof(s)) < 0 )
  {
    strcpy(s, "laelaps");
  }
  s[sizeof(s)-1] = 0;

  rsp.i.hostname  = s;

  ROS_INFO("Laelaps %d.%d.%d version.", nMajor, nMinor, nRev);

  return true;
}

bool LaelapsControl::getRange(GetRange::Request  &req,
                              GetRange::Response &rsp)
{
  const char *svc = "get_range";
  string      strRadType;         // sensor radiation type
  double      fFoV;               // sensor field of view (radians)
  double      fBeamDir;           // sensor beam center (radians)
  double      fMin, fMax;         // sensor min and max range (meters)
  double      fRange;             // measured object range (meters)
  int         rc;

  ROS_DEBUG("%s/%s", m_nh.getNamespace().c_str(), svc);

  rc = m_robot.getRangeSensorProps(req.name, strRadType, fFoV, fBeamDir,
                                      fMin, fMax);

  if( rc = LAE_OK )
  {
    rc = m_robot.getRange(req.name, fRange);
  }
 
  if( rc == LAE_OK )
  {
    if( strRadType == "infrared" )
    {
      rsp.sensor.radiation_type = sensor_msgs::Range::INFRARED;
    }
    else if( strRadType == "ultrasound" )
    {
      rsp.sensor.radiation_type = sensor_msgs::Range::ULTRASOUND;
    }
    else
    {
      rsp.sensor.radiation_type = 0xff;
    }

    rsp.sensor.field_of_view  = fFoV;
    rsp.sensor.min_range      = fMin;
    rsp.sensor.max_range      = fMax;

    if( (fRange < fMin) || (fRange > fMax) )
    {
      rsp.sensor.range = std::numeric_limits<float>::infinity();
    }

    ROS_INFO("Range sensor %s = %.3lf meters.",
        req.name.c_str(), rsp.sensor.range);
    return true;
  }
  else
  {
    ROS_ERROR("Service %s failed on sensor %s: %s(rc=%d).",
          svc, req.name.c_str(), getStrError(rc), rc);
    return false;
  }
}

bool LaelapsControl::isAlarmed(IsAlarmed::Request  &req,
                               IsAlarmed::Response &rsp)
{
  const char *svc = "is_alarmed";

  ROS_DEBUG("%s/%s", m_nh.getNamespace().c_str(), svc);

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

  ROS_DEBUG("%s/%s", m_nh.getNamespace().c_str(), svc);

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

  ROS_DEBUG("%s/%s", m_nh.getNamespace().c_str(), svc);

  rsp.gpio.clear();

  for(size_t i = 0; i < req.pin.size(); ++i)
  {
    Gpio    gpio;
    uint_t  val;

    gpio.pin = req.pin[i];

    rc = m_robot.readDigitalPin(gpio.pin, val);

    if( rc == LAE_OK )
    {
      gpio.state = (byte_t)val;
      rsp.gpio.push_back(gpio);

      ROS_INFO("GPIO pin %d = %d.",
          rsp.gpio[i].pin, rsp.gpio[i].state);
    }
    else
    {
      ROS_ERROR("Service %s failed on pin %d: %s(rc=%d).",
          svc, gpio.pin, getStrError(rc), rc);
      return false;
    }
  }

  return true;
}

bool LaelapsControl::release(Release::Request  &req,
                             Release::Response &rsp)
{
  const char *svc = "release";

  ROS_DEBUG("%s/%s", m_nh.getNamespace().c_str(), svc);

  m_robot.release();

  ROS_INFO("Robot released, motors are undriven.");

  return true;
}

bool LaelapsControl::reloadConfig(Release::Request  &req,
                                  Release::Response &rsp)
{
  const char *svc = "reload_config";

  ROS_DEBUG("%s/%s", m_nh.getNamespace().c_str(), svc);

  m_robot.reload();

  ROS_INFO("Robot configuration reloaded.");

  return true;
}

bool LaelapsControl::resetEStop(ResetEStop::Request  &req,
                                ResetEStop::Response &rsp)
{
  const char *svc = "reset_estop";

  ROS_DEBUG("%s/%s", m_nh.getNamespace().c_str(), svc);

  m_robot.resetEStop();

  ROS_INFO("EStop reset.");

  return true;
}

bool LaelapsControl::setAuxPower(SetAuxPwr::Request  &req,
                                 SetAuxPwr::Response &rsp)
{
  const char *svc = "set_aux_power";

  ROS_DEBUG("%s/%s", m_nh.getNamespace().c_str(), svc);

  for(size_t i = 0; i < req.aux.size(); ++i)
  {
    m_robot.setAuxPower(req.aux[i].name, (LaeTriState)req.aux[i].state.val);
    ROS_INFO("Aux. %s power %s.", req.aux[i].name.c_str(),
        (req.aux[i].state.val == industrial_msgs::TriState::ENABLED?
         "enabled": "disabled"));

  }

  return true;
}

bool LaelapsControl::setRobotMode(SetRobotMode::Request  &req,
                                  SetRobotMode::Response &rsp)
{
  const char *svc = "set_robot_mode";

  ROS_DEBUG("%s/%s", m_nh.getNamespace().c_str(), svc);

  m_robot.setRobotMode((LaeRobotMode)req.mode.val);

  ROS_INFO("Robot mode set to %d.", req.mode.val);

  return true;
}

bool LaelapsControl::setVelocities(SetVelocities::Request  &req,
                                   SetVelocities::Response &rsp)
{
  const char     *svc = "set_velocities";
  static u32_t    seq = 0;
  LaeMapVelocity  vel;
  int             rc;

  ROS_DEBUG("%s/%s", m_nh.getNamespace().c_str(), svc);

  for(size_t i = 0; i < req.goal.names.size(); ++i)
  {
    vel[req.goal.names[i]] = req.goal.velocities[i];
  }

  rc = m_robot.move(vel);
 
  if( rc == LAE_OK )
  {
    stampHeader(rsp.actual.header, seq++);
    
    // RDK TODO get actual goal, not just copy target goal
    rsp.actual = req.goal;

    ROS_INFO("Robot velocities set.");
    for(size_t i = 0; i < rsp.actual.names.size(); ++i)
    {
      ROS_INFO(" %-12s: vel=%7.2lfdeg/s",
          rsp.actual.names[i].c_str(),
          radToDeg(rsp.actual.velocities[i]));
    }
    return true;
  }
  else
  {
    ROS_ERROR("Service %s failed: %s(rc=%d).",
          svc, getStrError(rc), rc);
    return false;
  }
}

bool LaelapsControl::stop(Stop::Request  &req,
                          Stop::Response &rsp)
{
  const char *svc = "stop";
  int         rc;

  ROS_DEBUG("%s/%s", m_nh.getNamespace().c_str(), svc);

  rc = m_robot.stop();

  if( rc == LAE_OK )
  {
    ROS_INFO("Stopped.");
    return true;
  }
  else
  {
    ROS_ERROR("Service %s failed: %s(rc=%d).", svc, getStrError(rc), rc);
    return false;
  }
}

bool LaelapsControl::writeGpio(WriteGpio::Request  &req,
                               WriteGpio::Response &rsp)
{
  const char *svc = "write_gpio";
  int         rc;

  ROS_DEBUG("%s/%s", m_nh.getNamespace().c_str(), svc);

  for(size_t i = 0; i < req.gpio.size(); ++i)
  {
    rc = m_robot.writeDigitalPin(req.gpio[i].pin, req.gpio[i].state);

    if( rc == LAE_OK )
    {
      ROS_INFO("Write GPIO pin %d = %d.",
          req.gpio[i].pin, req.gpio[i].state);
    }
    else
    {
      ROS_ERROR("Service %s failed on pin %d: %s(rc=%d).",
          svc, req.gpio[i].pin, getStrError(rc), rc);
      return false;
    }
  }

  return true;
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

#if 0 // FUTURE
  strPub = "illuminance_state";
  m_publishers[strPub] =
    m_nh.advertise<IlluminanceState>(strPub, nQueueDepth);
#endif // FUTURE

  strPub = "imu_state_alt";
  m_publishers[strPub] =
    m_nh.advertise<ImuAlt>(strPub, nQueueDepth);

#if 0 // FUTURE
  strPub = "joint_state";
  m_publishers[strPub] =
    m_nh.advertise<sensor_msgs::JointState>(strPub, nQueueDepth);
#endif // FUTURE

  strPub = "range_state";
  m_publishers[strPub] =
    m_nh.advertise<RangeState>(strPub, nQueueDepth);

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
  publishSensorStates();
}

void LaelapsControl::publishDynamics()
{
  // update dynamics message
  updateDynamicsMsg(m_msgDynamics);

  // publish dynamics messages
  m_publishers["dynamics"].publish(m_msgDynamics);

#if 0 // FUTURE
  updateJointStateMsg(..., m_msgJointState);

  // publish message
  m_publishers["joint_state"].publish(m_msgJointState);
#endif // FUTURE
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

void LaelapsControl::publishSensorStates()
{
  updateImuAltMsg(m_msgImuAlt);

  m_publishers["imu_state_alt"].publish(m_msgImuAlt);

  updateRangeStateMsg(m_msgRangeState);

  m_publishers["range_state"].publish(m_msgRangeState);
}

void LaelapsControl::updateDynamicsMsg(Dynamics &msg)
{
  LaeRptDynamics  dynamics;

  // get robot's dynamics
  m_robot.getDynamics(dynamics);
  
  //
  // Clear previous state data.
  //
  msg.name.clear();
  msg.wheel_odometer.clear();
  msg.wheel_linear_velocity.clear();
  msg.wheel_angular_velocity.clear();
  msg.motor_encoder.clear();
  msg.motor_speed.clear();
  msg.motor_power_elec.clear();

  //
  // Set header.
  //
  stampHeader(msg.header, msg.header.seq+1);

  //
  // Set robot values.
  //
  // RDK msg.robot_odometer = ???
  msg.robot_pose.x      =  dynamics.m_pose.m_x;
  msg.robot_pose.y      =  dynamics.m_pose.m_y;
  msg.robot_pose.theta  =  dynamics.m_pose.m_theta;
  msg.robot_velocity    =  dynamics.m_fVelocity;

  //
  // Set powertrain values.
  //
  for(size_t i = 0; i < dynamics.m_vecPowertrain.size(); ++i)
  {
    LaePowertrain &pt = dynamics.m_vecPowertrain[i];

    msg.name.push_back(pt.m_strName);
    msg.wheel_odometer.push_back(pt.m_state.m_fOdometer);
    msg.wheel_linear_velocity.push_back(pt.m_state.m_fVelocity);
    msg.wheel_angular_velocity.push_back(pt.m_state.m_fVelAng);
    msg.motor_encoder.push_back(pt.m_state.m_nEncoder);
    msg.motor_speed.push_back(pt.m_state.m_nSpeed);
    msg.motor_power_elec.push_back(pt.m_state.m_fPe);
  }
}

void LaelapsControl::updateRobotStatusMsg(LaeRptRobotStatus &status,
                                          industrial_msgs::RobotStatus &msg)
{
  //
  // Set header.
  //
  stampHeader(msg.header, msg.header.seq+1);

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
  int         i;

  // clear previous data
  msg.motor_ctlr_health.clear();
  msg.motor_health.clear();

  //
  // Set header.
  //
  stampHeader(msg.header, msg.header.seq+1);

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

  //
  // Set extended robot status values.
  //
  msg.battery         = status.m_fBatterySoC;
  msg.is_charging.val = status.m_bIsCharging;
  msg.current         = status.m_fCurrent;
  msg.voltage         = status.m_fVoltage;
  msg.temp            = status.m_fTemperature;
  msg.aux_batt_en.val = status.m_eAuxBattEn;
  msg.aux_5v_en.val   = status.m_eAux5VEn;

  msg.alarms.is_critical  = status.m_alarms.m_bIsCritical;
  msg.alarms.alarms       = status.m_alarms.m_uAlarms;
  msg.alarms.warnings     = status.m_alarms.m_uWarnings;

  for(size_t i = 0; i < status.m_vecCtlrHealth.size(); ++i)
  {
    LaeRptMotorCtlrHealth  &h = status.m_vecCtlrHealth[i];
    MotorCtlrHealth         health;

    health.name     = h.m_strName;
    health.temp     = h.m_fTemperature;
    health.voltage  = h.m_fVoltage;

    health.alarms.is_critical = h.m_alarms.m_bIsCritical;
    health.alarms.alarms      = h.m_alarms.m_uAlarms;
    health.alarms.warnings    = h.m_alarms.m_uWarnings;

    msg.motor_ctlr_health.push_back(health);
  }

  for(size_t i = 0; i < status.m_vecMotorHealth.size(); ++i)
  {
    LaeRptMotorHealth  &h = status.m_vecMotorHealth[i];
    MotorHealth         health;

    health.name     = h.m_strName;
    health.temp     = h.m_fTemperature;
    health.voltage  = h.m_fVoltage;
    health.current  = h.m_fCurrent;

    health.alarms.is_critical = h.m_alarms.m_bIsCritical;
    health.alarms.alarms      = h.m_alarms.m_uAlarms;
    health.alarms.warnings    = h.m_alarms.m_uWarnings;

    msg.motor_health.push_back(health);
  }
}

void LaelapsControl::updateImuAltMsg(ImuAlt &msg)
{
  double      accel[ImuAlt::NUM_AXES];
  double      gyro[ImuAlt::NUM_AXES];
  double      rpy[ImuAlt::NUM_AXES];
  int         rc;

  rc = m_robot.getImu(accel, gyro, rpy);

  if( rc == LAE_OK )
  {
    for(int i = 0; i < ImuAlt::NUM_AXES; ++i)
    {
      msg.accel[i] = accel[i];
      msg.gyro[i]  = gyro[i];
      msg.rpy[i]   = rpy[i];
    }

    stampHeader(msg.header, msg.header.seq+1);
  }
}

void LaelapsControl::updateRangeStateMsg(RangeState &msg)
{
  vector<string>  names;
  vector<double>  ranges;
  int             rc;

  rc = m_robot.getRange(names, ranges);

  if( rc == LAE_OK )
  {
    stampHeader(msg.header, msg.header.seq+1);

    for(size_t i = 0; i < names.size(); ++i )
    {
      msg.name.push_back(names[i]);
      msg.range.push_back(ranges[i]);
    }
  }
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
  const char     *topic = "set_velocities";
  LaeMapVelocity  vel;

  ROS_DEBUG("%s/%s", m_nh.getNamespace().c_str(), topic);

  // set velocities
  ROS_INFO("%s", topic);
  for(int i=0; i<msgVel.names.size(); ++i)
  {
    vel[msgVel.names[i]] = msgVel.velocities[i];
    ROS_INFO(" %-12s: vel=%7.2lfdeg/s",
        msgVel.names[i].c_str(), radToDeg(msgVel.velocities[i]));
  }

  m_robot.move(vel);
}

void LaelapsControl::stampHeader(std_msgs::Header &header,
                                 u32_t             nSeqNum,
                                 const string     &strFrameId)
{
  header.seq      = nSeqNum;
  header.stamp    = ros::Time::now();
  header.frame_id = strFrameId;
}
