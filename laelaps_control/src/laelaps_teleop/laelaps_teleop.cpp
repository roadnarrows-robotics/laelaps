////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Laelaps Robot Package
//
// Link:      https://github.com/roadnarrows-robotics/laelaps
//
// ROS Node:  laelaps_teleop
//
// File:      laelaps_teleop.cpp
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief The ROS laelaps_teleop node class implementation.
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
#include <math.h>

#include <string>
#include <map>

//
// Boost
//
#include "boost/assign.hpp"

//
// ROS
//
#include "ros/ros.h"

//
// RoadNarrows
//
#include "rnr/rnrconfig.h"
#include "rnr/hid/HIDXbox360.h"

//
// ROS generated industrial messages.
//
#include "industrial_msgs/RobotStatus.h"        // subscribe

//
// ROS generated Laelaps messages.
//
#include "laelaps_control/ProductInfo.h"        // service
#include "laelaps_control/Velocity.h"           // publish

//
// ROS generatated Laelaps services.
//
#include "laelaps_control/EStop.h"
#include "laelaps_control/Freeze.h"
#include "laelaps_control/GetProductInfo.h"
#include "laelaps_control/IsAlarmed.h"
#include "laelaps_control/IsDescLoaded.h"
#include "laelaps_control/Release.h"
#include "laelaps_control/ResetEStop.h"
#include "laelaps_control/SetRobotMode.h"
#include "laelaps_control/Stop.h"

//
// ROS generated HID messages.
//
#include "hid/ConnStatus.h"           // subscribe
#include "hid/Controller360State.h"   // subscribe
#include "hid/LEDPattern.h"           // service
#include "hid/RumbleCmd.h"            // publish

//
// ROS generatated HID services.
//
#include "hid/SetLED.h"
#include "hid/SetRumble.h"

//
// RoadNarrows embedded laelaps library.
//
#include "Laelaps/laelaps.h"
#include "Laelaps/laeUtils.h"
#include "Laelaps/laeMotor.h"
#include "Laelaps/laeDesc.h"

//
// Node headers.
//
#include "laelaps_teleop.h"

using namespace std;
using namespace boost::assign;
using namespace hid;
using namespace laelaps_control;
using namespace laelaps;


//------------------------------------------------------------------------------
// LaelapsTeleop Class
//------------------------------------------------------------------------------

LaelapsTeleop::LaelapsTeleop(ros::NodeHandle &nh, double hz) :
      m_nh(nh), m_hz(hz)
{
  m_eState          = TeleopStateUninit;
  m_bHasXboxComm    = false;
  m_nWdXboxCounter  = 0;
  m_nWdXboxTimeout  = countsPerSecond(3.0);
  m_bHasRobotComm   = false;
  m_nWdRobotCounter = 0;
  m_nWdRobotTimeout = countsPerSecond(5.0);
  m_bHasFullComm    = false;

  m_maxRadiansPerSec = (double)LaeMotorRatedMaxRpm / 60.0 * M_TAU;
  m_fGovernor        = 1.0;

  m_buttonState = map_list_of
      (ButtonIdEStop,   0)
      (ButtonIdGovUp,   0)
      (ButtonIdGovDown, 0)
      //(ButtonIdBrake,   0)
      (ButtonIdPause,   0)
      (ButtonIdStart,   0)
      (ButtonIdMoveLin, 0)
      (ButtonIdMoveAng, 0);
}

LaelapsTeleop::~LaelapsTeleop()
{
}


//..............................................................................
// Server Services
//..............................................................................

void LaelapsTeleop::advertiseServices()
{
  // No services
}


//..............................................................................
// Client Services
//..............................................................................

void LaelapsTeleop::clientServices()
{
  string  strSvc;

  strSvc = "/xbox_360/set_led";
  m_clientServices[strSvc] = m_nh.serviceClient<hid::SetLED>(strSvc);

  strSvc = "/xbox_360/set_rumble";
  m_clientServices[strSvc] = m_nh.serviceClient<hid::SetRumble>(strSvc);

  strSvc = "/laelaps_control/estop";
  m_clientServices[strSvc] = m_nh.serviceClient<laelaps_control::EStop>(strSvc);

  strSvc = "/laelaps_control/freeze";
  m_clientServices[strSvc] =
      m_nh.serviceClient<laelaps_control::Freeze>(strSvc);

  strSvc = "/laelaps_control/release";
  m_clientServices[strSvc] =
      m_nh.serviceClient<laelaps_control::Release>(strSvc);

  strSvc = "/laelaps_control/reset_estop";
  m_clientServices[strSvc] =
      m_nh.serviceClient<laelaps_control::ResetEStop>(strSvc);

  strSvc = "/laelaps_control/set_robot_mode";
  m_clientServices[strSvc] =
    m_nh.serviceClient<laelaps_control::SetRobotMode>(strSvc);
}

void LaelapsTeleop::setLED(int pattern)
{
  hid::SetLED svc;

  svc.request.led_pattern.val = pattern;

  if( m_clientServices["/xbox_360/set_led"].call(svc) )
  {
    ROS_DEBUG("Xbox360 LED set to pattern to %d", pattern);
  }
  else
  {
    ROS_ERROR("Failed to set Xbox360 LED.");
  }
}

void LaelapsTeleop::setRumble(int motorLeft, int motorRight)
{
  hid::SetRumble svc;

  svc.request.left_rumble  = motorLeft;
  svc.request.right_rumble = motorRight;

  if( m_clientServices["/xbox_360/set_rumble"].call(svc) )
  {
    ROS_DEBUG("Xbox360 rumble motors set to %d, %d", motorLeft, motorRight);
  }
  else
  {
    ROS_ERROR("Failed to set Xbox360 rumble motors.");
  }
}

void LaelapsTeleop::estop()
{
  laelaps_control::EStop svc;

  if( m_clientServices["/laelaps_control/estop"].call(svc) )
  {
    ROS_INFO("Laelaps emergency stopped.");
  }
  else
  {
    ROS_ERROR("Failed to estop Laelaps.");
  }
}

void LaelapsTeleop::freeze()
{
  laelaps_control::Freeze svc;

  if( m_clientServices["/laelaps_control/freeze"].call(svc) )
  {
    ROS_INFO("Laelaps frozen.");
  }
  else
  {
    ROS_ERROR("Failed to freeze Laelaps.");
  }
}

void LaelapsTeleop::release()
{
  laelaps_control::Release svc;

  if( m_clientServices["/laelaps_control/release"].call(svc) )
  {
    ROS_INFO("Laelaps released.");
  }
  else
  {
    ROS_ERROR("Failed to release Laelaps.");
  }
}

void LaelapsTeleop::resetEStop()
{
  laelaps_control::ResetEStop svc;

  if( m_clientServices["/laelaps_control/reset_estop"].call(svc) )
  {
    ROS_INFO("Laelaps emergency stopped has been reset.");
  }
  else
  {
    ROS_ERROR("Failed to reset estop.");
  }
}

void LaelapsTeleop::setRobotMode(int mode)
{
  laelaps_control::SetRobotMode svc;

  svc.request.mode.val = mode;

  if( m_clientServices["/laelaps_control/set_robot_mode"].call(svc) )
  {
    ROS_DEBUG("Laelaps mode set to %d.", svc.request.mode.val);
  }
  else
  {
    ROS_ERROR("Failed to set robot mode.");
  }
}


//..............................................................................
// Topic Publishers
//..............................................................................

void LaelapsTeleop::advertisePublishers(int nQueueDepth)
{
  string  strPub;

  strPub = "/laelaps_control/set_velocities";
  m_publishers[strPub] =
    m_nh.advertise<laelaps_control::Velocity>(strPub, nQueueDepth);

  strPub = "/xbox_360/rumble_command";
  m_publishers[strPub] =
    m_nh.advertise<hid::RumbleCmd>(strPub, nQueueDepth);
}

void LaelapsTeleop::publishVelocities(double speedLeft, double speedRight)
{
  double    fVelLeft, fVelRight;
  Velocity  msg;

  fVelLeft  = speedLeft  * m_maxRadiansPerSec;
  fVelRight = speedRight * m_maxRadiansPerSec;

  // stampHeader RDK
  msg.names.push_back(LaeKeyLeftFront);
  msg.velocities.push_back(fVelLeft);
  msg.names.push_back(LaeKeyLeftRear);
  msg.velocities.push_back(fVelLeft);

  msg.names.push_back(LaeKeyRightFront);
  msg.velocities.push_back(fVelRight);
  msg.names.push_back(LaeKeyRightRear);
  msg.velocities.push_back(fVelRight);

  // publish
  m_publishers["/laelaps_control/set_velocities"].publish(msg);

  // RDK
  ROS_INFO("Speed = %6.1lf%%, %6.1lf%%.", speedLeft*100.0, speedRight*100.0);
}

void LaelapsTeleop::publishRumbleCmd(int motorLeft, int motorRight)
{
  RumbleCmd msg;
  
  msg.left_rumble  = motorLeft;
  msg.right_rumble = motorRight;

  // publish
  m_publishers["/xbox_360/rumble_command"].publish(msg);
}


//..............................................................................
// Subscribed Topics
//..............................................................................

void LaelapsTeleop::subscribeToTopics(int nQueueDepth)
{
  string  strSub;

  strSub = "/laelaps_control/robot_status";
  m_subscriptions[strSub] = m_nh.subscribe(strSub, nQueueDepth,
                                          &LaelapsTeleop::cbRobotStatus,
                                          &(*this));

  strSub = "/xbox_360/conn_status";
  m_subscriptions[strSub] = m_nh.subscribe(strSub, nQueueDepth,
                                          &LaelapsTeleop::cbXboxConnStatus,
                                          &(*this));

  strSub = "/xbox_360/controller_360_state";
  m_subscriptions[strSub] = m_nh.subscribe(strSub, nQueueDepth,
                                          &LaelapsTeleop::cbXboxBttnState,
                                          &(*this));
}

void LaelapsTeleop::cbRobotStatus(const industrial_msgs::RobotStatus &msg)
{
  ROS_DEBUG("Received robot status.");

  m_bHasRobotComm   = true;
  m_nWdRobotCounter = 0;

  m_msgRobotStatus = msg;
}

void LaelapsTeleop::cbXboxConnStatus(const hid::ConnStatus &msg)
{
  ROS_DEBUG("Received Xbox360 connectivity status.");

  m_bHasXboxComm    = msg.is_connected && msg.is_linked;
  m_nWdXboxCounter  = 0;

  m_msgConnStatus = msg;
}

void LaelapsTeleop::cbXboxBttnState(const hid::Controller360State &msg)
{
  ButtonState buttonState;

  ROS_DEBUG("Received Xbox360 button state.");

  if( m_bHasFullComm )
  {
    msgToState(msg, buttonState);

    switch( m_eState )
    {
      case TeleopStateReady:
        execAllButtonActions(buttonState);
        break;
      case TeleopStatePaused:
        buttonStart(buttonState); // only button active in pause state
        break;
      case TeleopStateUninit:
      default:
        pause();
        break;
    }
  }

  m_buttonState = buttonState;
}


//..............................................................................
// Sanity
//..............................................................................

void LaelapsTeleop::commCheck()
{
  if( m_bHasXboxComm )
  {
    if( ++m_nWdXboxCounter >= m_nWdXboxTimeout )
    {
      m_bHasXboxComm = false;
    }
  }

  if( m_bHasRobotComm )
  {
    if( ++m_nWdRobotCounter >= m_nWdRobotTimeout )
    {
      m_bHasRobotComm = false;
    }
  }

  bool hasComm  = m_bHasXboxComm && m_bHasRobotComm;

  // had communitcation, but no more
  if( m_bHasFullComm && !hasComm )
  {
    ROS_INFO("Lost communication with Xbox360 and/or Laelaps.");
    putRobotInSafeMode(m_msgConnStatus.is_connected);
  }

  m_bHasFullComm = hasComm;

  // not really a communication check function, but convenient.
  if( m_eState == TeleopStatePaused )
  {
    driveLEDsFigure8Pattern();
  }
}

void LaelapsTeleop::putRobotInSafeMode(bool bHard)
{
  // stop robot with 'parking' brake at full
  freeze();

  // set robot mode
  setRobotMode(LaeRobotModeAuto);
 
  m_eState = TeleopStateUninit;

  setLED(LEDPatOn);
}


//..............................................................................
// Xbox Actions
//..............................................................................

void LaelapsTeleop::msgToState(const hid::Controller360State &msg,
                            ButtonState                   &buttonState)
{
  buttonState[ButtonIdEStop]    = msg.b_button;
  buttonState[ButtonIdGovUp]    = msg.dpad_up;
  buttonState[ButtonIdGovDown]  = msg.dpad_down;
  buttonState[ButtonIdPause]    = msg.back_button;
  buttonState[ButtonIdStart]    = msg.start_button;
  buttonState[ButtonIdMoveLin]  = msg.left_joy_y;
  buttonState[ButtonIdMoveAng]  = msg.right_joy_x;
}

void LaelapsTeleop::execAllButtonActions(ButtonState &buttonState)
{
  // emergency stop
  buttonEStop(buttonState);

  //
  // Teleoperation state.
  //
  if( m_eState == TeleopStateReady )
  {
    buttonPause(buttonState);
  }
  else if( m_eState == TeleopStatePaused )
  {
    buttonStart(buttonState);
  }

  //
  // Moves.
  //
  if( canMove() )
  {
    buttonGovernorUp(buttonState);
    buttonGovernorDown(buttonState);
    buttonSpeed(buttonState);
    buttonBrake(buttonState);
  }
}

void LaelapsTeleop::buttonStart(ButtonState &buttonState)
{
  if( buttonOffToOn(ButtonIdStart, buttonState) )
  {
    ROS_INFO("Manual operation active, auto mode disabled.");

    if( m_msgRobotStatus.e_stopped.val == industrial_msgs::TriState::TRUE )
    {
      resetEStop();
    }

    setRobotMode(LaeRobotModeManual);

    ready();
  }
}

void LaelapsTeleop::buttonPause(ButtonState &buttonState)
{
  if( buttonOffToOn(ButtonIdPause, buttonState) )
  {
    ROS_INFO("Manual operation paused, auto mode enabled.");

    setRobotMode(LaeRobotModeAuto);

    pause();
  }
}

void LaelapsTeleop::buttonEStop(ButtonState &buttonState)
{
  static int  clicks        = 0;            // number of button clicks
  static int  intvlCounter  = 0;            // intra-click interval counter
  static int  intvlTimeout  = countsPerSecond(0.3); // intra-click timeout

  //
  // Robot is estopped. This can come from a different node source. Make sure
  // counters are cleared.
  //
  if( m_msgRobotStatus.e_stopped.val == industrial_msgs::TriState::TRUE )
  {
    clicks = 0;
    intvlCounter = 0;
    return;
  }

  // button off to on
  if( buttonOffToOn(ButtonIdEStop, buttonState) )
  {
    ++clicks;
  }

  switch( clicks )
  {
    case 0:     // no click
      break;
    case 1:     // single click
      if( intvlCounter > intvlTimeout )
      {
        clicks = 0;
        intvlCounter = 0;
      }
      break;
    case 2:     // double click
      if( intvlCounter <= intvlTimeout )
      {
        estop();
        pause();
      }
      clicks = 0;
      intvlCounter = 0;
      break;
    default:    // multiple clicks
      clicks = 0;
      intvlCounter = 0;
      break;
  }
}

void LaelapsTeleop::buttonGovernorUp(ButtonState &buttonState)
{
  if( buttonOffToOn(ButtonIdGovUp, buttonState) )
  {
    if( m_fGovernor <= 0.8 )
    {
      m_fGovernor += 0.2;
    }
  }
}

void LaelapsTeleop::buttonGovernorDown(ButtonState &buttonState)
{
  if( buttonOffToOn(ButtonIdGovDown, buttonState) )
  {
    if( m_fGovernor >= 0.4 )
    {
      m_fGovernor -= 0.2;
    }
  }
}

void LaelapsTeleop::buttonBrake(ButtonState &buttonState)
{
#if 0 // RDK
  float   brake;

  if( buttonDiff(ButtonIdBrake, buttonState) )
  {
    brake = (float)buttonState[ButtonIdBrake]/(float)XBOX360_TRIGGER_MAX;

    publishBrakeCmd(brake);
  }
#endif // RDK
}

void LaelapsTeleop::buttonSpeed(ButtonState &buttonState)
{
  double  joy_x;
  double  joy_y;
  double  linear;
  double  angular;
  double  div;
  double  speedLeft;
  double  speedRight;

  // joy button state [-32k, 32k]
  joy_x = (double)buttonState[ButtonIdMoveLin];
  joy_y = (double)buttonState[ButtonIdMoveAng];

  //
  // Note: laelaps_teleop has watchdog on this subscribed speed message. It
  // will timout and stop the robot if not sent frequently. So always send if
  // different or not zero.
  //
  if( !buttonDiff(ButtonIdMoveLin, buttonState) &&
      !buttonDiff(ButtonIdMoveAng, buttonState) &&
      (joy_x == 0.0) && (joy_y == 0.0) )
  {
    return;
  }

  // normalized linear and angular velocity components [-1.0, 1.0]
  linear  = joy_x / (double)XBOX360_JOY_MAX;
  angular = joy_y / (double)XBOX360_JOY_MAX;

  // mix [-2.0, 2.0]
  speedLeft  = linear + angular;
  speedRight = linear - angular;

  // apply software govenor
  speedLeft  *= m_fGovernor;
  speedRight *= m_fGovernor;

  // calculate divider
  if( fabs(speedLeft) > 1.0 )
  {
    div = fabs(speedLeft);
  }
  else if( fabs(speedRight) > 1.0 )
  {
    div = fabs(speedRight);
  }
  else
  {
    div = 1.0;
  }

  // normalize speed [-1.0, 1.0]
  speedLeft  = fcap(speedLeft/div,  -1.0, 1.0);
  speedRight = fcap(speedRight/div, -1.0, 1.0);

  // mix throttle x and y values
  //speedLeft   = (joy_x + joy_y) / (double)XBOX360_JOY_MAX;
  //speedRight  = (joy_y - joy_x) / (double)XBOX360_JOY_MAX;
  
  // cap
  //speedLeft   = fcap(speedLeft, -1.0, 1.0);
  //speedRight  = fcap(speedRight, -1.0, 1.0);

  publishVelocities(speedLeft, speedRight);
}


//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Support
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

void LaelapsTeleop::pause()
{
  m_eState = TeleopStatePaused;

  setLED(LEDPatPaused);
}

void LaelapsTeleop::ready()
{
  m_eState = TeleopStateReady;

  setLED(LEDPatReady);
}

void LaelapsTeleop::driveLEDsFigure8Pattern()
{
  static int nLEDTimeout = -1;
  static int nLEDCounter = 0;
  static int iLED = 0;
  static int LEDPat[] =
  {
    XBOX360_LED_PAT_1_ON, XBOX360_LED_PAT_2_ON,
    XBOX360_LED_PAT_3_ON, XBOX360_LED_PAT_4_ON
  };

  // lazy init
  if( nLEDTimeout < 0 )
  {
    nLEDTimeout = countsPerSecond(0.50);
  }

  // switch pattern
  if( nLEDCounter++ >= nLEDTimeout )
  {
    iLED = (iLED + 1) % arraysize(LEDPat);
    setLED(LEDPat[iLED]);
    nLEDCounter = 0;
  }
}

