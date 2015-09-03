////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Laelaps Mobile Robot ROS Package
//
// Link:      https://github.com/roadnarrows-robotics/laelaps
//
// ROS Node:  laelaps_control
//
// File:      laelaps_control.h
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief The ROS laelaps_control node class interface.
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

#ifndef _LAELAPS_CONTROL_H
#define _LAELAPS_CONTROL_H

//
// System
//
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
// ROS generated core and industrial messages.
//
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "industrial_msgs/RobotStatus.h"
#include "sensor_msgs/Illuminance.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Range.h"

//
// ROS generated Laelaps messages.
//
#include "laelaps_control/Caps.h"
#include "laelaps_control/Dimensions.h"
#include "laelaps_control/Dynamics.h"
#include "laelaps_control/Gpio.h"
#include "laelaps_control/IlluminanceState.h"
#include "laelaps_control/ImuCaps.h"
#include "laelaps_control/MotorCtlrHealth.h"
#include "laelaps_control/MotorHealth.h"
#include "laelaps_control/Path2D.h"
#include "laelaps_control/Pose2DStamped.h"
#include "laelaps_control/PowertrainCaps.h"
#include "laelaps_control/ProductInfo.h"
#include "laelaps_control/RangeState.h"
#include "laelaps_control/RobotStatusExtended.h"
#include "laelaps_control/ToFSensorCaps.h"
#include "laelaps_control/Velocity.h"

//
// ROS generatated Laelaps services.
//
#include "laelaps_control/ConfigGpio.h"
#include "laelaps_control/EStop.h"
#include "laelaps_control/Freeze.h"
#include "laelaps_control/GetCaps.h"
#include "laelaps_control/GetIlluminance.h"
#include "laelaps_control/GetImu.h"
#include "laelaps_control/GetProductInfo.h"
#include "laelaps_control/GetRange.h"
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
#include "Laelaps/laeRobot.h"

//
// Node headers.
//
#include "laelaps_control.h"


namespace laelaps_control
{
  /*!
   * \brief The class embodiment of the laelaps_control ROS node.
   */
  class LaelapsControl
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
     * \brief Default initialization constructor.
     *
     * \param nh  Bound node handle.
     * \param hz  Application nominal loop rate in Hertz.
     */
    LaelapsControl(ros::NodeHandle &nh, double hz);

    /*!
     * \brief Destructor.
     */
    virtual ~LaelapsControl();

    /*!
     * \brief Configure Laelaps product specifics.
     *
     * \param strCfgFile    XML configuration file name.
     *
     * \return Returns LAE_OK of success, \h_lt 0 on failure.
     */
    virtual int configure(const std::string &strCfgFile);

    /*!
     * \brief Connect to Laelaps hardware.
     *
     * \param strDevMotorCtrls     Motor controllers serial device name.
     *
     * \return Returns LAE_OK of success, \h_lt 0 on failure.
     */
    int connect(const std::string &strDevMotorCtlrs)
    {
      return strDevMotorCtlrs.empty()?  m_robot.connect():
                                        m_robot.connect(strDevMotorCtlrs);
    }

    /*!
     * \brief Disconnect from Laelaps.
     *
     * \return Returns LAE_OK of success, \h_lt 0 on failure.
     */
    int disconnect()
    {
      return m_robot.disconnect();
    }

    /*!
     * \brief Advertise all server services.
     */
    virtual void advertiseServices();

    /*!
     * \brief Initialize client services.
     */
    virtual void clientServices()
    {
      // No client services
    }

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
    virtual void publish();

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
     * \brief Get bound embedded robot instance.
     *
     * \return Robot instance.
     */
    laelaps::LaeRobot &getRobot()
    {
      return m_robot;
    }

    /*!
     * \brief Update robot dynamics state message from current robot dynamics.
     *
     * \param [in]  dynamics  Robot joint state.
     * \param [out] msg       Dynamics state message.
     */
    void updateDynamicsMsg(laelaps::LaeRptDynamics &dynamics,
                           Dynamics                &msg);

    /*!
     * \brief Update robot status message from current robot status.
     *
     * \param [in] status Robot status.
     * \param [out] msg   Robot status message.
     */
    void updateRobotStatusMsg(laelaps::LaeRptRobotStatus   &status,
                              industrial_msgs::RobotStatus &msg);

    /*!
     * \brief Update extended robot status message from current robot status.
     *
     * \param [in] status Robot status.
     * \param [out] msg   Extended roobt status message.
     */
    void updateExtendedRobotStatusMsg(laelaps::LaeRptRobotStatus &status,
                                      RobotStatusExtended        &msg);

  protected:
    ros::NodeHandle    &m_nh;     ///< the node handler bound to this instance
    double              m_hz;     ///< application nominal loop rate
    laelaps::LaeRobot   m_robot;  ///< real-time, Laelaps robotic arm

    // ROS services, publishers, subscriptions.
    MapServices       m_services;       ///< Laelaps control server services
    MapClientServices m_clientServices; ///< Laelaps control client services
    MapPublishers     m_publishers;     ///< Laelaps control publishers
    MapSubscriptions  m_subscriptions;  ///< Laelaps control subscriptions

    // Messages for published data.
    Dynamics                      m_msgDynamics;    ///< robot dynamics
    IlluminanceState              m_msgIlluminanceState;
                                        ///< ambient light sensors state message
    sensor_msgs::Imu              m_msgImu;         ///< IMU state message
    sensor_msgs::JointState       m_msgJointState;  ///< joint state message
    RangeState                    m_msgRangeState;
                                        ///< range sensors state message
    industrial_msgs::RobotStatus  m_msgRobotStatus; ///< robot status message
    RobotStatusExtended           m_msgRobotStatusEx;
                                        ///< extended robot status message

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Service callbacks
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Configure GPIO pins.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool configGpio(laelaps_control::ConfigGpio::Request  &req,
                    laelaps_control::ConfigGpio::Response &rsp);

    /*!
     * \brief Emergency stop robot service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool estop(laelaps_control::EStop::Request  &req,
               laelaps_control::EStop::Response &rsp);

    /*!
     * \brief Freeze (stop) robot service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool freeze(laelaps_control::Freeze::Request  &req,
                laelaps_control::Freeze::Response &rsp);

    /*!
     * \brief Get robot base capabilities service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool getCaps(laelaps_control::GetCaps::Request  &req,
                 laelaps_control::GetCaps::Response &rsp);

    /*!
     * \brief Get ambient light sensor's latest sensed data service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool getIlluminance(laelaps_control::GetIlluminance::Request  &req,
                        laelaps_control::GetIlluminance::Response &rsp);

    /*!
     * \brief Get inertia measurement unit's latest sensed data service
     * callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool getImu(laelaps_control::GetImu::Request  &req,
                laelaps_control::GetImu::Response &rsp);

    /*!
     * \brief Get robot product information service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool getProductInfo(laelaps_control::GetProductInfo::Request  &req,
                        laelaps_control::GetProductInfo::Response &rsp);

    /*!
     * \brief Get time-of-flight sensor's latest sensed data service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool getRange(laelaps_control::GetRange::Request  &req,
                  laelaps_control::GetRange::Response &rsp);

    /*!
     * \brief Go service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool go(laelaps_control::Go::Request  &req,
            laelaps_control::Go::Response &rsp);

    /*!
     * \brief Test if robot is alarmed service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool isAlarmed(laelaps_control::IsAlarmed::Request  &req,
                   laelaps_control::IsAlarmed::Response &rsp);

    /*!
     * \brief Test if robot description has been loaded service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool isDescLoaded(laelaps_control::IsDescLoaded::Request  &req,
                      laelaps_control::IsDescLoaded::Response &rsp);

    /*!
     * \brief Read GPIO pin states.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool readGpio(laelaps_control::ReadGpio::Request  &req,
                  laelaps_control::ReadGpio::Response &rsp);

    /*!
     * \brief Release drive power to robot motors service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool release(laelaps_control::Release::Request  &req,
                 laelaps_control::Release::Response &rsp);

    /*!
     * \brief Reload reloadable configuration and reset operational parameters
     * service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool reloadConfig(Release::Request  &req,
                      Release::Response &rsp);

    /*!
     * \brief Release robot's emergency stop condition service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool resetEStop(laelaps_control::ResetEStop::Request  &req,
                    laelaps_control::ResetEStop::Response &rsp);

    /*!
     * \brief Set robot's manual/auto mode service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool setRobotMode(laelaps_control::SetRobotMode::Request  &req,
                      laelaps_control::SetRobotMode::Response &rsp);

    /*!
     * \brief Stop a set of joints robot service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool stop(laelaps_control::Stop::Request  &req,
              laelaps_control::Stop::Response &rsp);

    /*!
     * \brief Write GPIO pin states.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool writeGpio(laelaps_control::WriteGpio::Request  &req,
                   laelaps_control::WriteGpio::Response &rsp);


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Topic Publishers
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Publish dynamics and joint state topics.
     */
    void publishDynamics();

    /*!
     * \brief Publish robot status and extended robot status topics.
     */
    void publishRobotStatus();

    /*!
     * \brief Publish sensor state topics.
     */
    void publishSensorStates();


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Subscribed Topic Callbacks
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Execute set velocities subscibed topic callback.
     *
     * \param msgVel  Velocity message.
     */
    void execSetVelocities(const laelaps_control::Velocity &msgVel);


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Utilities
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Fill in ROS standard message header.
     *
     * \param [out] header    Message header.
     * \param nSeqNum         Sequence number.
     * \param strFrameId      Frame id. No frame = "0", global frame = "1".
     */
    void stampHeader(std_msgs::Header  &header,
                     u32_t             nSeqNum = 0,
                     const std::string &strFrameId = "0");
  };

} // namespace laelaps_control


#endif // _LAELAPS_CONTROL_H
