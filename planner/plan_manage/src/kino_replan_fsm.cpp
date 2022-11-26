/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/


#include <plan_manage/kino_replan_fsm.h>
#include <kr_tracker_msgs/Transition.h>
#include <kr_tracker_msgs/BsplineTrackerAction.h>
#include <std_srvs/Trigger.h>
namespace fast_planner {

void KinoReplanFSM::init(ros::NodeHandle& nh) {
  current_wp_  = 0;
  exec_state_  = FSM_EXEC_STATE::INIT;
  have_target_ = false;
  have_odom_   = false;

  /*  fsm param  */
  nh.param("fsm/flight_type", target_type_, -1);
  nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
  nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
  nh.param("srv_name", srv_name_, std::string(" "));


  if (target_type_ == TARGET_TYPE::PRESET_TARGET){
    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++) {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }
  }


  /* initialize main modules */
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh);
  visualization_.reset(new PlanningVisualization(nh));

  /* callback */
  exec_timer_   = nh.createTimer(ros::Duration(0.01), &KinoReplanFSM::execFSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &KinoReplanFSM::checkCollisionCallback, this);

  waypoint_sub_ =  nh.subscribe("/ddk/goal_point", 1, &KinoReplanFSM::waypointCallback, this);
  
  
  odom_sub_ = nh.subscribe("odom", 1, &KinoReplanFSM::odometryCallback, this);


  traj_goal_pub_ = nh.advertise<kr_tracker_msgs::BsplineTrackerActionGoal>("tracker_cmd", 10);

}

void KinoReplanFSM::waypointCallback(const geometry_msgs::PoseStampedPtr& msg) {

  cout << "Triggered!" << endl;
  trigger_ = true;

  if (target_type_ == TARGET_TYPE::MANUAL_TARGET) {
    waypoint_num_ = 1;
    waypoints_[0][0] = msg->pose.position.x;
    waypoints_[0][1] = msg->pose.position.y;
    waypoints_[0][2] = 1.0;

    Eigen::Quaterniond q;

    q.w() = msg->pose.orientation.w;
    q.x() = msg->pose.orientation.x;
    q.y() = msg->pose.orientation.y;
    q.z() = msg->pose.orientation.z;

    end_yaw_ = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
  }

  current_wp_ = 0;

  setGoal();
  have_target_ = true;
}


void KinoReplanFSM::setGoal(){

  if(current_wp_ >=  waypoint_num_){
    have_target_ = false;
    return;
  }

  end_pt_(0)  = waypoints_[current_wp_][0];
  end_pt_(1)  = waypoints_[current_wp_][1];
  end_pt_(2)  = waypoints_[current_wp_][2];
  current_wp_ += 1;

  ROS_INFO_STREAM("current_wp_  is : " << current_wp_);

  visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
  end_vel_.setZero();
  
  if (exec_state_ == WAIT_TARGET)
    changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
  else if (exec_state_ == EXEC_TRAJ)
    changeFSMExecState(REPLAN_TRAJ, "TRIG");

}

void KinoReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;

  Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
  odom_yaw_ = atan2(rot_x(1), rot_x(0));

  have_odom_ = true;
  odom_time_ = msg->header.stamp;
}

void KinoReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call) {
  string state_str[7] = { "INIT", "WAIT_TARGET", "INIT_ROTATE", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
  int    pre_s        = int(exec_state_);
  exec_state_         = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

void KinoReplanFSM::printFSMExecState() {
  string state_str[7] = { "INIT", "WAIT_TARGET", "INIT_ROTATE", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP" };

  cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
}

void KinoReplanFSM::execFSMCallback(const ros::TimerEvent& e) {
  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100) {
    printFSMExecState();
    if (!have_odom_) cout << "no odom." << endl;
    if (!trigger_) cout << "wait for goal." << endl;
    fsm_num = 0;
  }

  switch (exec_state_) {
    case INIT: {
      if (!have_odom_) {
        return;
      }
      if (!trigger_) {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET: {
      if (!have_target_)
        return;
      else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }    

    case GEN_NEW_TRAJ: {
      start_pt_  = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      start_yaw_(1) = start_yaw_(2) = 0.0;


      plan_stime_ = odom_time_ + ros::Duration(0.01);


      bool success = callKinodynamicReplan();
      if (success) {
        fail_cnt_= 0;
                flag_escape_emergency_ = true;
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } 
      else if (fail_cnt_ >= 10)
      {
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
      }else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        fail_cnt_ += 1;
      }
      break;
    }

    case EXEC_TRAJ: {
      /* determine if need to replan */
      LocalTrajData* info     = &planner_manager_->local_data_;
      ros::Time      time_now = ros::Time::now();
      double         t_cur    = (time_now - info->start_time_).toSec();
      t_cur                   = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2) {
        //have_target_ = false;
        setGoal();
        changeFSMExecState(WAIT_TARGET, "FSM");
        return;

      } else if ((end_pt_ - pos).norm() < no_replan_thresh_) {
        //cout << "near end" << endl;
        return;

      } else if ((info->start_pos_ - pos).norm() < replan_thresh_) {
        //cout << "near start" << endl;
        return;

      } else if (t_cur > 2.0)
      {
        cout << "[FSM] from exec to replan t_cur " << t_cur << endl;
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ: {
      LocalTrajData* info     = &planner_manager_->local_data_;
      // ros::Time      time_now = ros::Time::now();
      // double         t_cur    = std::min((time_now - info->start_time_).toSec(), info->duration_);

      plan_stime_ = ros::Time::now() + ros::Duration(0.01);
      double         t_cur    = std::min((plan_stime_ - info->start_time_).toSec(), info->duration_);

      start_pt_  = info->position_traj_.evaluateDeBoorT(t_cur);

      if ((start_pt_ - odom_pos_).norm() > 1.0) {
        //cout << "near end" << endl;
        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
      }

      start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
      start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

      start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_cur)[0];
      start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_cur)[0];
      start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_cur)[0];


      kr_tracker_msgs::BsplineTrackerActionGoal replan_msg;
      replan_msg.goal.status = 1;
      traj_goal_pub_.publish(replan_msg);

      std_srvs::Trigger trg;
      ros::service::call(srv_name_, trg);

      bool success = callKinodynamicReplan();
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop();
      }
      else
      {
        if (odom_vel_.norm() < 0.1)
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      flag_escape_emergency_ = false;
      break;
    }


  }
}

void KinoReplanFSM::checkCollisionCallback(const ros::TimerEvent& e) {
  LocalTrajData* info = &planner_manager_->local_data_;

  if (have_target_) {
    auto edt_env = planner_manager_->edt_environment_;

    double dist = planner_manager_->pp_.dynamic_ ?
        edt_env->evaluateCoarseEDT(end_pt_, /* time to program start + */ info->duration_) :
        edt_env->evaluateCoarseEDT(end_pt_, -1.0);
     

    if (dist <= 0.4 || !edt_env->sdf_map_->isInMap(end_pt_)) {
      /* try to find a max distance goal around */
      bool            new_goal = false;
      const double    dr = 0.5, dtheta = 30, dz = 0.3;
      double          new_x, new_y, new_z, max_dist = -1.0;
      Eigen::Vector3d goal;

      for (double r = dr; r <= 5 * dr + 1e-3; r += dr) {
        for (double theta = -90; theta <= 270; theta += dtheta) {
          for (double nz = 1 * dz; nz >= -1 * dz; nz -= dz) {

            new_x = end_pt_(0) + r * cos(theta / 57.3);
            new_y = end_pt_(1) + r * sin(theta / 57.3);
            new_z = end_pt_(2) + nz;

            Eigen::Vector3d new_pt(new_x, new_y, new_z);

            if (!edt_env->sdf_map_->isInMap(new_pt))
            {
              continue;
            }


            dist = planner_manager_->pp_.dynamic_ ?
                edt_env->evaluateCoarseEDT(new_pt, /* time to program start+ */ info->duration_) :
                edt_env->evaluateCoarseEDT(new_pt, -1.0);

            if (dist > max_dist) {
              /* reset end_pt_ */
              goal(0)  = new_x;
              goal(1)  = new_y;
              goal(2)  = new_z;
              max_dist = dist;
            }
          }
        }
      }

      if (max_dist > 0.4) {
        cout << "change goal, replan." << endl;
        end_pt_      = goal;
        have_target_ = true;
        end_vel_.setZero();

        if (exec_state_ == EXEC_TRAJ) {
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        }

        visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
      } else {
        // have_target_ = false;
        // cout << "Goal near collision, stop." << endl;
        // changeFSMExecState(WAIT_TARGET, "SAFETY");
        cout << "goal near collision, keep retry" << endl;
        changeFSMExecState(REPLAN_TRAJ, "FSM");

        kr_tracker_msgs::BsplineTrackerActionGoal replan_msg;
        replan_msg.goal.status = 1;
        traj_goal_pub_.publish(replan_msg);

        std_srvs::Trigger trg;
        ros::service::call(srv_name_, trg);

      }
    }
  }

  /* ---------- check trajectory ---------- */
  if (exec_state_ == FSM_EXEC_STATE::EXEC_TRAJ) {
    double dist;
    bool   safe = planner_manager_->checkTrajCollision(dist);

    if (!safe) {
      // cout << "current traj in collision." << endl;
      ROS_WARN("current traj in collision.");
      if (dist < 0.5)
      {
        changeFSMExecState(EMERGENCY_STOP, "SAFETY");
      }else{
        changeFSMExecState(REPLAN_TRAJ, "SAFETY");
      }
    }
  }
}


  bool KinoReplanFSM::callEmergencyStop()
  {

    planner_manager_->kinodynamicReplan(start_pt_, start_vel_, start_acc_, start_pt_, Eigen::Vector3d(0.0, 0.0, 0.0));

    auto info = &planner_manager_->local_data_;


    planner_manager_->planYaw(start_yaw_);
    LocalTrajData *locdat = &planner_manager_->local_data_;

    locdat->start_time_ = plan_stime_;

    /* publish traj */
    kr_tracker_msgs::BsplineTrackerActionGoal bspline_msg;

    bspline_msg.goal.order      = 3;
    bspline_msg.goal.start_time = locdat->start_time_;
    bspline_msg.goal.traj_id    = locdat->traj_id_;

    Eigen::MatrixXd pos_pts = locdat->position_traj_.getControlPoint();

    std::cout << " pos_pts is   " <<  pos_pts << std::endl;
    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline_msg.goal.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = locdat->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline_msg.goal.knots.push_back(knots(i));
    }

    Eigen::MatrixXd yaw_pts = locdat->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);

     //// cout << "yaw is " << yaw << endl;
      bspline_msg.goal.yaw_pts.push_back(yaw);
    }
    bspline_msg.goal.yaw_dt = locdat->yaw_traj_.getInterval();


    cout << "!!!!!!!!!!! send the command" << endl;
    bspline_msg.goal.status = 2;
    traj_goal_pub_.publish(bspline_msg);

    std_srvs::Trigger trg;
    ros::service::call(srv_name_, trg);

    /* visulization */
    auto plan_data = &planner_manager_->plan_data_;
    visualization_->drawGeometricPath(plan_data->kino_path_, 0.075, Eigen::Vector4d(1, 1, 0, 0.4));
    visualization_->drawBspline(locdat->position_traj_, 0.1, Eigen::Vector4d(1.0, 0, 0.0, 1), true, 0.2,
                                Eigen::Vector4d(1, 0, 0, 1));


    return true;
  }




bool KinoReplanFSM::callKinodynamicReplan() {
  bool plan_success =
      planner_manager_->kinodynamicReplan(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_);

  if (plan_success) {

    // if((start_pt_- end_pt_ ).norm() < 2.0)
    // {
    //   planner_manager_->planYaw(start_yaw_, end_yaw_);

    // }else{
    //   planner_manager_->planYaw(start_yaw_);

    // }

    planner_manager_->planYaw(start_yaw_);
    LocalTrajData *locdat = &planner_manager_->local_data_;

    locdat->start_time_ = plan_stime_;

    /* publish traj */
    kr_tracker_msgs::BsplineTrackerActionGoal bspline_msg;

    bspline_msg.goal.order      = 3;
    bspline_msg.goal.start_time = locdat->start_time_;
    bspline_msg.goal.traj_id    = locdat->traj_id_;

    Eigen::MatrixXd pos_pts = locdat->position_traj_.getControlPoint();

    /////std::cout << " pos_pts is   " <<  pos_pts << std::endl;
    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline_msg.goal.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = locdat->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline_msg.goal.knots.push_back(knots(i));
    }

    Eigen::MatrixXd yaw_pts = locdat->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);

     //// cout << "yaw is " << yaw << endl;
      bspline_msg.goal.yaw_pts.push_back(yaw);
    }
    bspline_msg.goal.yaw_dt = locdat->yaw_traj_.getInterval();

    cout << "!!!!!!!!!!! send the command" << endl;
    bspline_msg.goal.status = 2;
    traj_goal_pub_.publish(bspline_msg);

    std_srvs::Trigger trg;
    ros::service::call(srv_name_, trg);

    /* visulization */
    auto plan_data = &planner_manager_->plan_data_;
    visualization_->drawGeometricPath(plan_data->kino_path_, 0.075, Eigen::Vector4d(1, 1, 0, 0.4));
    visualization_->drawBspline(locdat->position_traj_, 0.1, Eigen::Vector4d(1.0, 0, 0.0, 1), true, 0.2,
                                Eigen::Vector4d(1, 0, 0, 1));

    return true;

  } else {
    cout << "generate new traj fail." << endl;
    return false;
  }
}

// KinoReplanFSM::
}  // namespace fast_planner
