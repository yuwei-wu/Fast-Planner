#include "ros/ros.h"

#include "plan_manage/Bspline2PosCmd.h"
#include <kr_tracker_msgs/BsplineTrackerAction.h>
#include "kr_mav_msgs/PositionCommand.h"


bool bspline_to_pos_cmd(plan_manage::Bspline2PosCmd::Request &req, plan_manage::Bspline2PosCmd::Response &res) {
    return true;
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "bspline2poscmd_service_node");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("bspline2poscmd", bspline_to_pos_cmd);

  ros::spin();

  return 0;
}