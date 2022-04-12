#include <kr_tracker_msgs/TrackerStatus.h>
#include <kr_trackers_manager/Tracker.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <kr_tracker_msgs/TrackerStatus.h>
#include <kr_tracker_msgs/BsplineTrackerAction.h>
#include "bspline/non_uniform_bspline.h"
#include <Eigen/Eigen>
#include <actionlib/server/simple_action_server.h>
#include <kr_trackers/initial_conditions.h>

using fast_planner::NonUniformBspline;


class BsplineTracker : public kr_trackers_manager::Tracker
{
 public:
  BsplineTracker(void);

  void Initialize(const ros::NodeHandle &nh);
  bool Activate(const kr_mav_msgs::PositionCommand::ConstPtr &cmd);
  void Deactivate(void);

  kr_mav_msgs::PositionCommand::ConstPtr update(const nav_msgs::Odometry::ConstPtr &msg);
  uint8_t status() const;

 private:
  

  void goal_callback();
  void preempt_callback();

  kr_mav_msgs::PositionCommand position_cmd_;

  double last_t_;

  /*** current odom***/
  double cur_yaw_;
  Eigen::Vector3d cur_pos_, last_goal_;
  bool have_last_goal_ = false;

  int traj_id_;

  typedef actionlib::SimpleActionServer<kr_tracker_msgs::BsplineTrackerAction> ServerType;

  // Action server that takes a goal.
  // Must be a pointer because plugin does not support a constructor with inputs, but an action server must be
  // initialized with a Nodehandle.
  std::unique_ptr<ServerType> tracker_server_;


  bool pos_set_, goal_set_, goal_reached_, active_;
  bool traj_set_ = false;

  ros::Time start_time_;
  std::vector<NonUniformBspline> traj_;
  double traj_dur_;

  // for intial rotation
  double init_final_yaw_, init_dyaw_;
  double init_yaw_time_;

  // yaw control
  double last_yaw_ = 0.0, last_yawdot_ = 0.0;
  ros::Time time_last_;
  double time_forward_ = 1.5;

};


BsplineTracker::BsplineTracker(void) : pos_set_(false), goal_set_(false), goal_reached_(false), active_(false) {}


void BsplineTracker::Initialize(const ros::NodeHandle &nh)
{
  ros::NodeHandle priv_nh(nh, "bspline_tracker");

  // Set up the action server.
  tracker_server_.reset(new ServerType(priv_nh, "BsplineTracker", false));
  tracker_server_->registerGoalCallback(boost::bind(&BsplineTracker::goal_callback, this));
  tracker_server_->registerPreemptCallback(boost::bind(&BsplineTracker::preempt_callback, this));

  tracker_server_->start();

}

bool BsplineTracker::Activate(const kr_mav_msgs::PositionCommand::ConstPtr &cmd)
{
  // Only allow activation if a goal has been set
  if(pos_set_)
  {
    if(!tracker_server_->isActive())
    {
      ROS_WARN("TrajectoryTracker::Activate: goal_set_ is true but action server has no active goal - not activating.");
      active_ = false;
      return false;
    }
    active_ = true;
    ROS_WARN("TrajectoryTracker::Activate: !");
  }
  return active_;
}



void BsplineTracker::Deactivate(void)
{
  if(tracker_server_->isActive())
  {
    ROS_WARN("BsplineTracker::Deactivate: deactivated tracker while still tracking the goal.");
    tracker_server_->setAborted();
  }
  
  goal_set_ = false;
  active_ = false;
}


kr_mav_msgs::PositionCommand::ConstPtr BsplineTracker::update(const nav_msgs::Odometry::ConstPtr &msg)
{
  pos_set_ = true;

  cur_pos_(0) = msg->pose.pose.position.x;
  cur_pos_(1) = msg->pose.pose.position.y;
  cur_pos_(2) = msg->pose.pose.position.z;
  cur_yaw_ = tf::getYaw(msg->pose.pose.orientation);

  ros::Time time_now = ros::Time::now();

  if(!active_){
    time_last_ = time_now;
    return kr_mav_msgs::PositionCommand::Ptr();
  }
    
  double t_cur = (time_now - start_time_).toSec();

  Eigen::Vector3d pos(0.0, 0.0, 0.0), vel(0.0, 0.0, 0.0), acc(0.0, 0.0, 0.0), pos_f(0.0, 0.0, 0.0), jerk(0.0, 0.0, 0.0);
  double yaw = 0, yawdot = 0;

  if (t_cur < traj_dur_ && t_cur >= 0.0) {

    pos  = traj_[0].evaluateDeBoorT(t_cur);
    vel  = traj_[1].evaluateDeBoorT(t_cur);
    acc  = traj_[2].evaluateDeBoorT(t_cur);
    jerk = traj_[3].evaluateDeBoorT(t_cur);
    yaw  = traj_[4].evaluateDeBoorT(t_cur)[0];
    yawdot = traj_[5].evaluateDeBoorT(t_cur)[0];

    double tf = min(traj_dur_, t_cur + 2.0);
    pos_f = traj_[0].evaluateDeBoorT(tf);

  } else if (t_cur >= traj_dur_) {
    /* hover when finish traj_ */
    pos = traj_[0].evaluateDeBoorT(traj_dur_);
    vel.setZero();
    acc.setZero();
    yaw = traj_[4].evaluateDeBoorT(traj_dur_)[0];
    yawdot = 0;

    pos_f = pos;

  } else {
    cout << "[Traj server]: invalid time." << endl;
    return kr_mav_msgs::PositionCommand::Ptr();
  }

  //publish the command
  position_cmd_.header.frame_id = msg->header.frame_id;
  position_cmd_.header.stamp = time_now;
  //std::cout << " publish the command cur_pos_ " << cur_pos_ << std::endl;
  position_cmd_.position.x       = pos(0);
  position_cmd_.position.y       = pos(1);
  position_cmd_.position.z       = pos(2);
  position_cmd_.velocity.x       = vel(0);
  position_cmd_.velocity.y       = vel(1);
  position_cmd_.velocity.z       = vel(2);
  position_cmd_.acceleration.x   = acc(0);
  position_cmd_.acceleration.y   = acc(1);
  position_cmd_.acceleration.z   = acc(2);
  position_cmd_.jerk.x = jerk(0);
  position_cmd_.jerk.y = jerk(1);
  position_cmd_.jerk.z = jerk(2);

  position_cmd_.yaw = yaw;
  position_cmd_.yaw_dot = yawdot;

  time_last_ = time_now;
  

  return kr_mav_msgs::PositionCommand::ConstPtr(new kr_mav_msgs::PositionCommand(position_cmd_));
}


void BsplineTracker::goal_callback()
{
  // If another goal is already active, cancel that goal and track this one instead.
  if(tracker_server_->isActive())
  {
    ROS_INFO("BsplineTracker goal aborted");
    tracker_server_->setAborted();
  }

  // Pointer to the recieved goal.
  const auto msg = tracker_server_->acceptNewGoal();


  // If preempt has been requested, then set this goal to preempted and make no changes to the tracker state.
  if(tracker_server_->isPreemptRequested())
  {
    ROS_INFO("BsplineTracker preempted");
    tracker_server_->setPreempted();
    return;
  }
    
  
  switch (msg->status)
  {
    case 0:
    {

      break;
    }

    case 1:
    {
      // replan
      /* reset duration */
      const double time_out = 0.01;
      ros::Time time_now = ros::Time::now();
      double t_stop = (time_now - start_time_).toSec() + time_out;
      traj_dur_ = min(t_stop, traj_dur_);
    
      break;
    }

    case 2:
    {
      ROS_INFO("BsplineTracker : receive the traj!");
      Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);

      Eigen::VectorXd knots(msg->knots.size());
      for (long unsigned int i = 0; i < msg->knots.size(); ++i) {
        knots(i) = msg->knots[i];
      }

      for (long unsigned int i = 0; i < msg->pos_pts.size(); ++i) {
        pos_pts(i, 0) = msg->pos_pts[i].x;
        pos_pts(i, 1) = msg->pos_pts[i].y;
        pos_pts(i, 2) = msg->pos_pts[i].z;
      }

      NonUniformBspline pos_traj(pos_pts, msg->order, 0.1);
      pos_traj.setKnot(knots);

      // parse yaw traj

      Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
      for (long unsigned int i = 0; i < msg->yaw_pts.size(); ++i) {
        yaw_pts(i, 0) = msg->yaw_pts[i];
      }

      NonUniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

      /* Store data */
      start_time_ = msg->start_time;
      traj_id_ = msg->traj_id;

      traj_.clear();
      traj_.push_back(pos_traj);
      traj_.push_back(traj_[0].getDerivative());
      traj_.push_back(traj_[1].getDerivative());
      traj_.push_back(traj_[2].getDerivative());
      traj_.push_back(yaw_traj);
      traj_.push_back(yaw_traj.getDerivative());

      traj_dur_ = traj_[0].getTimeSum();

      goal_set_ = true;
      goal_reached_ = false;
      traj_set_ = true;

    }

  }
  return;
}

void BsplineTracker::preempt_callback()
{
  if(tracker_server_->isActive())
  {
    ROS_INFO("BsplineTracker aborted");
    tracker_server_->setAborted();
  }
  else
  {
    ROS_INFO("BsplineTracker preempted");
    tracker_server_->setPreempted();
  }

  goal_set_ = false;
  goal_reached_ = true;
}


uint8_t BsplineTracker::status() const
{
  return tracker_server_->isActive() ? static_cast<uint8_t>(kr_tracker_msgs::TrackerStatus::ACTIVE) :
                                       static_cast<uint8_t>(kr_tracker_msgs::TrackerStatus::SUCCEEDED);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(BsplineTracker, kr_trackers_manager::Tracker)
