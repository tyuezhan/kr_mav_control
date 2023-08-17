#include <kr_tracker_msgs/TrackerStatus.h>
#include <kr_tracker_msgs/Transition.h>
#include <kr_trackers_manager/Tracker.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

class TrackersManager : public nodelet::Nodelet
{
 public:
  TrackersManager(void);
  ~TrackersManager(void);

  void onInit(void);

 private:
  void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
  bool transition_callback(kr_tracker_msgs::Transition::Request &req, kr_tracker_msgs::Transition::Response &res);
  // initialize last_cmd_ to be null 
  kr_mav_msgs::PositionCommand last_cmd_;
  bool last_cmd_initialized_ = false;
  //(new kr_mav_msgs::PositionCommand);
  bool critical_safety_entered_ = false;
  bool no_stopping_policy_ = false;
  

  ros::Subscriber sub_odom_;
  ros::Publisher pub_cmd_, pub_status_;
  ros::ServiceServer srv_tracker_;
  pluginlib::ClassLoader<kr_trackers_manager::Tracker> tracker_loader_;
  kr_trackers_manager::Tracker *active_tracker_;
  std::map<std::string, kr_trackers_manager::Tracker *> tracker_map_;
  kr_mav_msgs::PositionCommand::ConstPtr cmd_;
};

TrackersManager::TrackersManager(void)
    : tracker_loader_("kr_trackers_manager", "kr_trackers_manager::Tracker"), active_tracker_(NULL)
{
}

TrackersManager::~TrackersManager(void)
{
  for(std::map<std::string, kr_trackers_manager::Tracker *>::iterator it = tracker_map_.begin();
      it != tracker_map_.end(); it++)
  {
    delete it->second;
#if ROS_VERSION_MINIMUM(1, 8, 0)
    try
    {
      tracker_loader_.unloadLibraryForClass(it->first);
    }
    catch(pluginlib::LibraryUnloadException &e)
    {
      NODELET_ERROR_STREAM("Could not unload library for the tracker " << it->first << ": " << e.what());
    }
#endif
  }
}

void TrackersManager::onInit(void)
{
  ros::NodeHandle priv_nh(getPrivateNodeHandle());

  XmlRpc::XmlRpcValue tracker_list;
  priv_nh.getParam("trackers", tracker_list);
  ROS_ASSERT(tracker_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for(int i = 0; i < tracker_list.size(); i++)
  {
    ROS_ASSERT(tracker_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    const std::string tracker_name = static_cast<const std::string>(tracker_list[i]);
    try
    {
#if ROS_VERSION_MINIMUM(1, 8, 0)
      kr_trackers_manager::Tracker *c = tracker_loader_.createUnmanagedInstance(tracker_name);
#else
      kr_trackers_manager::Tracker *c = tracker_loader_.createClassInstance(tracker_name);
#endif
      c->Initialize(priv_nh);
      tracker_map_.insert(std::make_pair(tracker_name, c));
    }
    catch(pluginlib::LibraryLoadException &e)
    {
      NODELET_ERROR_STREAM("Could not load library for the tracker " << tracker_name << ": " << e.what());
    }
    catch(pluginlib::CreateClassException &e)
    {
      NODELET_ERROR_STREAM("Could not create an instance of the tracker " << tracker_name << ": " << e.what());
    }
  }

  pub_cmd_ = priv_nh.advertise<kr_mav_msgs::PositionCommand>("cmd", 10);
  pub_status_ = priv_nh.advertise<kr_tracker_msgs::TrackerStatus>("status", 10);

  sub_odom_ = priv_nh.subscribe("odom", 10, &TrackersManager::odom_callback, this, ros::TransportHints().tcpNoDelay());

  srv_tracker_ = priv_nh.advertiseService("transition", &TrackersManager::transition_callback, this);
}

void TrackersManager::odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  if (no_stopping_policy_){
    // initialize with empty command
    kr_mav_msgs::PositionCommand new_cmd_temp = kr_mav_msgs::PositionCommand();
    ROS_ERROR_THROTTLE(0.1,"+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
    ROS_ERROR_THROTTLE(0.1,"VERY UNSAFE SCENARIO, EVEN STOPPING POLICY FAIL TO OUTPUT SAFE COMMAND, PLEASE MANUALLY TAKE OVER!!!");
    ROS_ERROR_THROTTLE(0.1,"VERY UNSAFE SCENARIO, EVEN STOPPING POLICY FAIL TO OUTPUT SAFE COMMAND, PLEASE MANUALLY TAKE OVER!!!");
    ROS_ERROR_THROTTLE(0.1,"VERY UNSAFE SCENARIO, EVEN STOPPING POLICY FAIL TO OUTPUT SAFE COMMAND, PLEASE MANUALLY TAKE OVER!!!");
    ROS_ERROR_STREAM_THROTTLE(0.1,"no_stopping_policy_: " << no_stopping_policy_);
    ROS_ERROR_THROTTLE(0.1,"+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
    new_cmd_temp.position.x = msg->pose.pose.position.x;
    new_cmd_temp.position.y = msg->pose.pose.position.y;
    new_cmd_temp.position.z = msg->pose.pose.position.z;
    new_cmd_temp.velocity.x = 0.0;
    new_cmd_temp.velocity.y = 0.0;
    new_cmd_temp.velocity.z = 0.0;
    new_cmd_temp.acceleration.x = 0.0;
    new_cmd_temp.acceleration.y = 0.0;
    new_cmd_temp.acceleration.z = 0.0;

    new_cmd_temp.header.frame_id = cmd_->header.frame_id;
    new_cmd_temp.header.stamp = cmd_->header.stamp;
    if(last_cmd_initialized_){
      new_cmd_temp.yaw = last_cmd_.yaw;
    } else {
      new_cmd_temp.yaw = 0.0;
    }
    new_cmd_temp.yaw_dot = 0.0;
    
    pub_cmd_.publish(new_cmd_temp);
    // copy cmd_ to last_cmd_
    last_cmd_initialized_ = true;
    last_cmd_ = new_cmd_temp;
    return;
  }

  std::map<std::string, kr_trackers_manager::Tracker *>::iterator it;
  
  for(it = tracker_map_.begin(); it != tracker_map_.end(); it++)
  {
    if(it->second == active_tracker_)
    {
      cmd_ = it->second->update(msg);
      if (cmd_ != NULL){
        // if cmd position is too different from msg position, if yes, print error msg and set cmd to be the odometry position with 0 velocity
        double distance_squared = std::pow((cmd_->position.x - msg->pose.pose.position.x),2) + std::pow((cmd_->position.y - msg->pose.pose.position.y),2) + std::pow((cmd_->position.z - msg->pose.pose.position.z),2);
        if (last_cmd_initialized_ && distance_squared > 1.0){
          ROS_ERROR("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
          ROS_ERROR_STREAM("UNSAFE!!! CALLING STOPPING POLICY! cmd position is too different from odom position, distance_squared: " << distance_squared);
          ROS_ERROR_STREAM("UNSAFE!!! CALLING STOPPING POLICY! cmd position is too different from odom position, distance_squared: " << distance_squared);
          ROS_ERROR_STREAM("UNSAFE!!! CALLING STOPPING POLICY! cmd position is too different from odom position, distance_squared: " << distance_squared);
          ROS_ERROR_STREAM("UNSAFE!!! CALLING STOPPING POLICY! cmd position is too different from odom position, distance_squared: " << distance_squared);
          ROS_ERROR("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
          if (!critical_safety_entered_){
            critical_safety_entered_ = true;
            // transit to stopping policy with position from odometry and velocity and accelearation from last_cmd 
            
            const std::map<std::string, kr_trackers_manager::Tracker *>::iterator it2 = tracker_map_.find("kr_trackers/StoppingPolicy");
            if(it2 == tracker_map_.end()) {
              ROS_ERROR("Failed to find stopping policy!!!!!!!!");
              ROS_ERROR("Failed to find stopping policy!!!!!!!!");
              ROS_ERROR("Failed to find stopping policy!!!!!!!!");
              no_stopping_policy_ = true;
              continue;
            }
            // Compose new command
            
            kr_mav_msgs::PositionCommand new_cmd = kr_mav_msgs::PositionCommand();
            new_cmd.position.x = msg->pose.pose.position.x;
            new_cmd.position.y = msg->pose.pose.position.y;
            new_cmd.position.z = msg->pose.pose.position.z;
            new_cmd.velocity.x = last_cmd_.velocity.x;
            new_cmd.velocity.y = last_cmd_.velocity.y;
            new_cmd.velocity.z = last_cmd_.velocity.z;
            new_cmd.acceleration.x = last_cmd_.acceleration.x;
            new_cmd.acceleration.y = last_cmd_.acceleration.y;
            new_cmd.acceleration.z = last_cmd_.acceleration.z;
            new_cmd.yaw = last_cmd_.yaw;
            new_cmd.yaw_dot = last_cmd_.yaw_dot;
            
            new_cmd.header.frame_id = cmd_->header.frame_id;
            new_cmd.header.stamp = cmd_->header.stamp;
            // make new_cmd constptr for activate function
            kr_mav_msgs::PositionCommand::ConstPtr new_cmd_ptr = boost::make_shared<kr_mav_msgs::PositionCommand>(new_cmd);
            if(!it2->second->Activate(new_cmd_ptr)) {
              no_stopping_policy_ = true;
              ROS_ERROR("Failed to activate stopping policy!!!!!!!!");
              ROS_ERROR("Failed to activate stopping policy!!!!!!!!");
              ROS_ERROR("Failed to activate stopping policy!!!!!!!!");
              continue;
            } else {
              if(active_tracker_ != NULL)
              {
                active_tracker_->Deactivate();
              }
              active_tracker_ = it2->second;
              ROS_ERROR("Stopping Policy Triggered Under Critical Safety Condition!!!!!!!!");
              ROS_ERROR("Stopping Policy Triggered Under Critical Safety Condition!!!!!!!!");
              ROS_ERROR("Stopping Policy Triggered Under Critical Safety Condition!!!!!!!!");
            }

          } else {
            // initialize with empty command
            kr_mav_msgs::PositionCommand new_cmd_temp = kr_mav_msgs::PositionCommand();
            ROS_ERROR_THROTTLE(0.1,"+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
            ROS_ERROR_THROTTLE(0.1,"VERY UNSAFE SCENARIO, EVEN STOPPING POLICY FAIL TO OUTPUT SAFE COMMAND, PLEASE MANUALLY TAKE OVER!!!");
            ROS_ERROR_THROTTLE(0.1,"VERY UNSAFE SCENARIO, EVEN STOPPING POLICY FAIL TO OUTPUT SAFE COMMAND, PLEASE MANUALLY TAKE OVER!!!");
            ROS_ERROR_THROTTLE(0.1,"VERY UNSAFE SCENARIO, EVEN STOPPING POLICY FAIL TO OUTPUT SAFE COMMAND, PLEASE MANUALLY TAKE OVER!!!");
            ROS_ERROR_STREAM_THROTTLE(0.1,"critical_safety_entered_: " << no_stopping_policy_);
            ROS_ERROR_THROTTLE(0.1,"+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
            new_cmd_temp.position.x = msg->pose.pose.position.x;
            new_cmd_temp.position.y = msg->pose.pose.position.y;
            new_cmd_temp.position.z = msg->pose.pose.position.z;
            new_cmd_temp.velocity.x = 0.0;
            new_cmd_temp.velocity.y = 0.0;
            new_cmd_temp.velocity.z = 0.0;
            new_cmd_temp.acceleration.x = 0.0;
            new_cmd_temp.acceleration.y = 0.0;
            new_cmd_temp.acceleration.z = 0.0;

            new_cmd_temp.header.frame_id = cmd_->header.frame_id;
            new_cmd_temp.header.stamp = cmd_->header.stamp;
            new_cmd_temp.yaw = last_cmd_.yaw;

            new_cmd_temp.yaw_dot = 0.0;
            
            pub_cmd_.publish(new_cmd_temp);
            // copy cmd_ to last_cmd_
            last_cmd_initialized_ = true;
            last_cmd_ = new_cmd_temp;
          }
        } else {
          pub_cmd_.publish(cmd_);
          // copy cmd_ to last_cmd_
          last_cmd_initialized_ = true;
          last_cmd_ = *cmd_;
        }
      }

      kr_tracker_msgs::TrackerStatus::Ptr status_msg(new kr_tracker_msgs::TrackerStatus);
      status_msg->header.stamp = msg->header.stamp;
      status_msg->tracker = it->first;
      status_msg->status = it->second->status();
      pub_status_.publish(status_msg);
    }
    else
    {
      it->second->update(msg);
    }
  }
}

bool TrackersManager::transition_callback(kr_tracker_msgs::Transition::Request &req,
                                          kr_tracker_msgs::Transition::Response &res)
{
  if (critical_safety_entered_){
    ROS_ERROR("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
    ROS_ERROR("CRITICAL SAFETY ENTERED, CANNOT TRANSITION, PLEASE MANUALLY TAKE OVER!!!");
    ROS_ERROR("CRITICAL SAFETY ENTERED, CANNOT TRANSITION, PLEASE MANUALLY TAKE OVER!!!");
    ROS_ERROR("CRITICAL SAFETY ENTERED, CANNOT TRANSITION, PLEASE MANUALLY TAKE OVER!!!");
    ROS_ERROR("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
    res.success = false;
    res.message = std::string("Critical safety entered, cannot transition");
    NODELET_WARN_STREAM(res.message);
    return true;
  }

  const std::map<std::string, kr_trackers_manager::Tracker *>::iterator it = tracker_map_.find(req.tracker);
  if(it == tracker_map_.end())
  {
    res.success = false;
    res.message = std::string("Cannot find tracker ") + req.tracker + std::string(", cannot transition");
    NODELET_WARN_STREAM(res.message);
    return true;
  }
  if(active_tracker_ == it->second)
  {
    res.success = true;
    res.message = std::string("Tracker ") + req.tracker + std::string(" already active");
    NODELET_INFO_STREAM(res.message);
    return true;
  }

  if(!it->second->Activate(cmd_))
  {
    res.success = false;
    res.message = std::string("Failed to activate tracker ") + req.tracker + std::string(", cannot transition");
    NODELET_WARN_STREAM(res.message);
    return true;
  }

  if(active_tracker_ != NULL)
  {
    active_tracker_->Deactivate();
  }

  active_tracker_ = it->second;
  res.success = true;
  res.message = std::string("Successfully activated tracker ") + req.tracker;
  return true;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(TrackersManager, nodelet::Nodelet);
