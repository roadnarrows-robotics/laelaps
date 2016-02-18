////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Laelaps Robot Package
//
// Link:      https://github.com/roadnarrows-robotics/laelaps
//
// ROS Node:  laelaps_teleop
//
// File:      laelaps_teleop.h
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief The ROS laelaps_teleop node class interface.
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

#ifndef _LAELAPS_TELEOP_H
#define _LAELAPS_TELEOP_H

#include <string>
#include <map>

//
// Includes for boost libraries
//
#include <boost/bind.hpp>

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
#include "laelaps_control/DutyCycle.h"          // publish
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
#include "hid/Controller360State.h"   // subscribe
#include "hid/ConnStatus.h"           // subscribe
#include "hid/LEDPattern.h"           // service
#include "hid/RumbleCmd.h"            // publish

//
// ROS generatated Laelaps services.
//
#include "hid/SetLED.h"
#include "hid/SetRumble.h"


namespace laelaps_control
{
  /*!
   * \brief The class embodiment of the laelaps_teleop ROS node.
   */
  class LaelapsTeleop
  {
  public:
    /*! map of ROS server services type */
    typedef std::map<std::string, ros::ServiceServer> MapServices;

    /*! map of ROS client services type */
    typedef std::map<std::string, ros::ServiceClient> MapClientServices;

    /*! map of ROS publishers type */
    typedef std::map<std::string, ros::Publisher> MapPublishers;

    /*! map of ROS subscriptions type */
    typedef std::map<std::string, ros::Subscriber> MapSubscriptions;

    /*!
     * \brief Teleoperation state.
     */
    enum TeleopState
    {
      TeleopStateUninit,    ///< not initialized
      TeleopStatePaused,    ///< paused
      TeleopStateReady,     ///< ready and running
    };

    /*!
     * \brief Xbox360 button map ids.
     */
    enum ButtonId
    {
      ButtonIdEStop       = rnr::Xbox360FeatIdBButton,  ///< emergency stop
      ButtonIdGovUp       = rnr::Xbox360FeatIdPadUp,    ///< governor speed up
      ButtonIdGovDown     = rnr::Xbox360FeatIdPadDown,  ///< governor speed down
      ButtonIdMoveModeDec = rnr::Xbox360FeatIdPadLeft,  ///< move mode -
      ButtonIdMoveModeInc = rnr::Xbox360FeatIdPadRight, ///< move mode +
      ButtonIdPause       = rnr::Xbox360FeatIdBack,     ///< pause teleop
      ButtonIdStart       = rnr::Xbox360FeatIdStart,    ///< start teleop
      ButtonIdMoveLin     = rnr::Xbox360FeatIdLeftJoyY, ///< move fwd/bwd
      ButtonIdMoveAng     = rnr::Xbox360FeatIdRightJoyX,  ///< turn left/right
    //ButtonIdBrake     = rnr::Xbox360FeatIdLeftTrigger,  ///< ease brake
    };

    /*! teleop button state type */
    typedef std::map<int, int> ButtonState;

    /*!
     * \brief Xbox360 LED patterns.
     */
    enum LEDPat
    {
      LEDPatOff    = XBOX360_LED_PAT_ALL_OFF,     ///< all off
      LEDPatOn     = XBOX360_LED_PAT_ALL_BLINK,   ///< default xbox on pattern
      LEDPatPaused = XBOX360_LED_PAT_4_ON,        ///< pause teleop
      LEDPatReady  = XBOX360_LED_PAT_ALL_SPIN     ///< ready to teleop
    };

    /*!
     * \brief Move modes.
     */
    enum MoveMode
    {
      MoveModeVelocity,   ///< move by specifying angular velocites (vel PID)
      MoveModeDutyCycle,  ///< move by specifying motor duty cycles (open loop)
    //MoveModeTist,       ///< move by specifying linear and angular velocities
      MoveModeNumOf       ///< number of move modes
    };

    /*!
     * \brief Default initialization constructor.
     *
     * \param nh  Bound node handle.
     * \param hz  Application nominal loop rate in Hertz.
     */
    LaelapsTeleop(ros::NodeHandle &nh, double hz);

    /*!
     * \brief Destructor.
     */
    virtual ~LaelapsTeleop();

    /*!
     * \brief Advertise all server services.
     */
    virtual void advertiseServices();

    /*!
     * \brief Initialize client services.
     */
    virtual void clientServices();

    /*!
     * \brief Advertise all publishers.
     *
     * \param nQueueDepth   Maximum queue depth.
     */
    virtual void advertisePublishers(int nQueueDepth=10);

    /*!
     * \brief Subscribe to all topics.
     *
     * \param nQueueDepth   Maximum queue depth.
     */
    virtual void subscribeToTopics(int nQueueDepth=10);

    /*!
     * \brief Publish.
     *
     * Call in main loop.
     */
    virtual void publish()
    {
      // No periodic publishing. All event driven.
    }

    /*!
     * \brief Check communications.
     *
     * Call in main loop.
     */
    virtual void commCheck();

    /*!
     * \brief Put robot into safe mode.
     *
     * \param bHard   Harden safe mode. When teleop node dies or xbox is 
     *                physically disconnected, robot is set to known defaults.
     */
    void putRobotInSafeMode(bool bHard);

    /*!
     * \brief Test if robot is allowed to move.
     *
     * \return Returns true or false.
     */
    bool canMove()
    {
      if((m_eState == TeleopStateReady) &&
         (m_msgRobotStatus.e_stopped.val == industrial_msgs::TriState::FALSE) &&
         (m_msgRobotStatus.in_error.val == industrial_msgs::TriState::FALSE))
      {
        return true;
      }
      else
      {
        return false;
      }
    }

    /*!
     * \brief Get bound node handle.
     *
     * \return Node handle.
     */
    ros::NodeHandle &getNodeHandle()
    {
      return m_nh;
    }

    /*!
     * \brief Convert seconds to loop counts.
     *
     * \param seconds Seconds.
     *
     * \return Count.
     */
    int countsPerSecond(double seconds)
    {
      return (int)(seconds * m_hz);
    }

  protected:
    // ROS
    ros::NodeHandle  &m_nh;       ///< the node handler bound to this instance
    double            m_hz;       ///< application nominal loop rate

    // ROS services, publishers, subscriptions.
    MapServices       m_services;       ///< laelaps teleop as server services
    MapClientServices m_clientServices; ///< laelaps teleop as client services
    MapPublishers     m_publishers;     ///< laelaps teleop publishers
    MapSubscriptions  m_subscriptions;  ///< laelaps teleop subscriptions

    // state
    TeleopState       m_eState;           ///< teleoperation state
    bool              m_bHasXboxComm;     ///< Xbox communications is [not] good
    int               m_nWdXboxCounter;   ///< Xbox watchdog counter
    int               m_nWdXboxTimeout;   ///< Xbox watchdog timeout
    bool              m_bHasRobotComm;    ///< communications is [not] good
    int               m_nWdRobotCounter;  ///< watchdog counter
    int               m_nWdRobotTimeout;  ///< watchdog timeout
    bool              m_bHasFullComm;     ///< good full communications
    ButtonState       m_buttonState;      ///< saved button state

    // working data
    double    m_fMaxRadiansPerSec;///< max wheel-shaft radians/second
    double    m_fGovernor;        ///< software velocity governor
    MoveMode  m_eMoveMode;        ///< move mode
    int       m_iLedPattern;      ///< current Xbox LED pattern
    int       m_iLedTempPattern;  ///< temporary Xbox LED pattern
    int       m_nLedTempCounter;  ///< temporary LED pattern counter

    // messages
    industrial_msgs::RobotStatus m_msgRobotStatus;
                                                ///< saved last robot status 
    hid::ConnStatus           m_msgConnStatus;  ///< saved last conn status 


    //..........................................................................
    // Server Service callbacks
    //..........................................................................

    // none


    //..........................................................................
    // Client Services
    //..........................................................................

    /*!
     * \brief Set Xbox360 LED pattern client service request.
     *
     * \param pattern   LED pattern.
     */
    void setLed(int pattern);

    /*!
     * \brief Set Xbox360 left and right rumble motors client service request.
     *
     * \param motorLeft   Left motor speed.
     * \param motorRight  Right motor speed.
     */
    void setRumble(int motorLeft, int motorRight);

    /*!
     * \brief Emergency stop robot client service request.
     */
    void estop();

    /*!
     * \brief Freeze (stop) the robot with full brake applied client service
     * request.
     */
    void freeze();

    /*!
     * \brief Release (neutral) the robot with no brake applied client service
     * request.
     */
    void release();

    /*!
     * \brief Reset emergency stop condition.
     */
    void resetEStop();

    /*!
     * \brief Set robot's speed limiting governor.
     *
     * \param governor  New governor setting.
     */
    //void setGovernor(float governor);

    /*!
     * \brief Set robot's operation mode.
     *
     * \param mode    New rebot mode: auto or manual.
     */
    void setRobotMode(int mode);


    //..........................................................................
    // Topic Publishers
    //..........................................................................

    /*!
     * \brief Publish motor duty cycles command.
     *
     * \param dutyLeft    Duty cycle of left motors [-1.0, 1.0].
     * \param dutyRight   Duty cycle of right motors [-1.0, 1.0].
     */
    void publishDutyCycles(double dutyLeft, double dutyRight);

    /*!
     * \brief Publish speed command.
     *
     * \param speedLeft   Speed of left motors [-1.0, 1.0].
     * \param speedRight  Speed of right motors [-1.0, 1.0].
     */
    void publishVelocities(double speedLeft, double speedRight);

    /*!
     * \brief Publish Xbox360 rumble command.
     *
     * \param motorLeft   Left rumble motor speed.
     * \param motorRight  Right rumble motor speed.
     */
    void publishRumbleCmd(int motorLeft, int motorRight);


    //..........................................................................
    // Subscribed Topic Callbacks
    //..........................................................................

    /*!
     * \brief Robot status callback.
     *
     * \param msg Received subscribed topic.
     */
    void cbRobotStatus(const industrial_msgs::RobotStatus &msg);

    /*!
     * \brief Xbox360 HID connectivity status callback.
     *
     * \param msg Received subscribed topic.
     */
    void cbXboxConnStatus(const hid::ConnStatus &msg);

    /*!
     * \brief Xbox360 HID button state callback.
     *
     * \param msg Received subscribed topic.
     */
    void cbXboxBttnState(const hid::Controller360State &msg);

    //..........................................................................
    // Xbox Actions
    //..........................................................................

    /*!
     * \brief Convert ROS Xbox360 message to button state.
     *
     * \param msg                 Message.
     * \param [out] buttonState   Button statue.
     */
    void msgToState(const hid::Controller360State &msg,
                    ButtonState                   &buttonState);

    /*!
     * \brief Test if button toggle from off to on.
     *
     * \param id            Button id.
     * \param buttonState   Button state array.
     *
     * \return Returns true or false.
     */
    bool buttonOffToOn(int id, ButtonState &buttonState)
    {
      return (m_buttonState[id] == 0) && (buttonState[id] == 1);
    }

    /*!
     * \brief Test if button state has changed.
     *
     * \param id            Button id.
     * \param buttonState   Button state array.
     *
     * \return Returns true or false.
     */
    bool buttonDiff(int id, ButtonState &buttonState)
    {
      return m_buttonState[id] != buttonState[id];
    }

    /*!
     * \brief Execute all new button initiated actions.
     *
     * \param buttonState   Button state array.
     */
    void execAllButtonActions(ButtonState &buttonState);

    /*!
     * \brief Execute start button action.
     *
     * \param buttonState   Button state array.
     */
    void buttonStart(ButtonState &buttonState);

    /*!
     * \brief Execute pause button action.
     *
     * \param buttonState   Button state array.
     */
    void buttonPause(ButtonState &buttonState);

    /*!
     * \brief Execute emergency stop button action.
     *
     * \param buttonState   Button state array.
     */
    void buttonEStop(ButtonState &buttonState);

    /*!
     * \brief Execute software governor up button action.
     *
     * \param buttonState   Button state array.
     */
    void buttonGovernorUp(ButtonState &buttonState);

    /*!
     * \brief Execute software governor down button action.
     *
     * \param buttonState   Button state array.
     */
    void buttonGovernorDown(ButtonState &buttonState);

    /*!
     * \brief Execute software move mode up button action.
     *
     * \param buttonState   Button state array.
     */
    void buttonMoveModeDec(ButtonState &buttonState);

    /*!
     * \brief Execute software move mode down button action.
     *
     * \param buttonState   Button state array.
     */
    void buttonMoveModeInc(ButtonState &buttonState);

    /*!
     * \brief Execute brake button action.
     *
     * \param buttonState   Button state array.
     */
    void buttonBrake(ButtonState &buttonState);

    /*!
     * \brief Execute speed buttons action.
     *
     * \param buttonState   Button state array.
     */
    void buttonSpeed(ButtonState &buttonState);


    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
    // Support 
    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

    /*!
     * \brief Go to pause teleoperation state.
     */
    void pause();

    /*!
     * \brief Go to ready to teleoperate state.
     */
    void ready();

    /*!
     * \brief Drive Xbox360 LEDs into a figure 8 pattern.
     */
    void driveLedFigure8Pattern();

    /*!
     * \brief Drive Xbox360 temporary LED pattern.
     *
     * \param pattern   LED pattern.
     */
    void driveLedTempPattern();

    /*!
     * \brief Restore LED patteern given the current teleoperation state.
     */
    void restoreLedPattern();

    /*!
     * \brief Set Xbox360 LED pattern for the current governor setting.
     */
    void setLedGovernorPattern();

    /*!
     * \brief Set Xbox360 LED pattern for the current move mode.
     */
    void setLedMoveModePattern();

    /*!
     * \brief Set LED temporary pattern for the given seconds.
     *
     * The previous pattern is restored after timeout.
     *
     * \param pattern   LED temporary pattern.
     * \param seconds   Seconds to keep patttern.
     */
    void setTempLed(int pattern, double seconds);

  };

} // namespace laelaps_control


#endif // _LAELAPS_TELEOP_H
