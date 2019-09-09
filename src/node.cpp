#include <ros/ros.h>
#include <dh_hand_driver/hand_controller.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hand_controller");
  ros::NodeHandle n;
  HandController controller(n,"actuate_hand");
  ros::spin();

  return 0;
}

