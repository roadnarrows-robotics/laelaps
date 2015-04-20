#include <string>
#include "ros/ros.h"

using namespace std;

#if 0
bool SetSpeed(mot::SetSpeed::Request &req,
              mot::SetSpeed::Response &rsp)
{
  ROS_INFO("Setting speed"):
    pMot->setSpeed(req.devName, req.mot_id, req.speed);
    fprintf(stderr, "Speed = %f \n", req,speed);
  retrun true;
}
#endif

int main(int argc, char* argv[])
{ 
#if 0
  LOG_SET_THRESHOLD(LOG_LEVEL_DIAG3);
  ros::init(argc, argv, "laelaps_control");
  ros::NodeHandle n("laelaps_control");

  pMot=new MotRoboteqSmall;

  string devName1="/dev/ttyACM0";
  string devName2="/dev/ttyACM1";
  int baudRate=115200;
  if(pMot->open(devName, baudRate)!=0)
  {
    fprintf(stderr, "jnt failed to open\n");
    return -1;
  }
  
  //Services
  ros::ServiceServer set_speed_ser = n.advertiseService("set_speed", 
                                                       SetSpeed);

  //Subscriptions
  ros::Subscriber speed_command_sub = n.subscribe("speed_command", 1,
                                                 speed_commandCB);

  ros::spin();

  if(pMot->close()!=0)
  {
    fprintf(stderr, "jnt failed to close\n");
    return -1;
  }
#endif

  return 0;
}
