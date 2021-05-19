#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <configuration_msgs/StartConfiguration.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit_planning_helper/manage_trajectories.h>
#include <simple_touch_controller_msgs/simpleTouchAction.h>






#define GET_AND_RETURN( nh, param, value )\
  if (!nh.getParam(param,value) )\
  {\
    ROS_ERROR("The param '%s/%s' is not defined", nh.getNamespace().c_str(), std::string( param ).c_str() );\
    return false;\
  }



#define GET_AND_DEFAULT( nh, param, value, def )\
  if (!nh.getParam(param,value) )\
  {\
    ROS_WARN("The param '%s/%s' is not defined", nh.getNamespace().c_str(), std::string( param ).c_str() );\
    ROS_WARN("Default value '%s' superimposed. " );\
    value=def;\
  }
  
  
namespace action_type{

enum ACTION_TYPE
{
     none                  = 0
    ,FollowJointTrajectory = 1
    ,simpleTouch           = 2
};
}
const std::map<std::string, action_type::ACTION_TYPE> action_map_ = {
    {"none"                   ,action_type::none                 }
   ,{""                       ,action_type::none                 }
   ,{"traj"                   ,action_type::FollowJointTrajectory}
   ,{"trajectory"             ,action_type::FollowJointTrajectory}
   ,{"followjointtrajectory"  ,action_type::FollowJointTrajectory}
   ,{"follow_joint_trajectory",action_type::FollowJointTrajectory}
   ,{"touch"                  ,action_type::simpleTouch          }
   ,{"simple_touch"           ,action_type::simpleTouch          }
   ,{"simpletouch"            ,action_type::simpleTouch          }
   ,{"velocity_touch"         ,action_type::simpleTouch          }
}; 

  
  
  
struct Task 
{
  const std::string id;
  
  control_msgs::FollowJointTrajectoryGoal       goal_trajectory;
  simple_touch_controller_msgs::simpleTouchGoal goal_touch;
//   cartesian_motion_msgs::cartMotionGoal         goal_cart;
  
  std::string traj_id;
  std::string action_name;
  std::string configuration;
  std::string target_frame;
  std::string goal_twist_frame;
  
  double assesment_time;
  
  std::vector<double> goal_twist      ;
  std::vector<double> target_wrench   ;
  std::vector<double> target_delta_pos;
  std::vector<double> goal_toll       ;
  std::vector<double> wrench_deadband ;
  std::vector<double> displacement    ;
  
  double linear_speed      ;
  double angular_speed     ;
  double linear_tollerance ;
  double angular_tollerance;
  
  bool plan_correction = true;
  bool attach          = false;
  bool detach          = false;
  bool replan          = false;
  
  action_type::ACTION_TYPE action_type;
  
  Task(std::string id): id(id) {};
  
  inline bool getGoalTrajectory(ros::NodeHandle nh)
  {
    std::string path = "trajectories_planned/"+traj_id;
    trajectory_msgs::JointTrajectory traj;
    if(! trajectory_processing::getTrajectoryFromParam(nh, path, traj))
    {
      ROS_ERROR("%s not found ros param, nh ns: [%s]",traj_id.c_str(), nh.getNamespace().c_str());
      return false;
    }
    goal_trajectory.trajectory = traj;
    return true;
  }
  
  inline bool getGoalTouch()
  {
     goal_touch.goal_twist          = goal_twist;
     goal_touch.target_wrench       = target_wrench;
     goal_touch.wrench_toll         = goal_toll;
     goal_touch.wrench_deadband     = wrench_deadband;
     goal_touch.target_wrench_frame = target_frame;
     goal_touch.goal_twist_frame    = goal_twist_frame;

    return true;
  }
  
  inline bool getGoalCart()
  {
//     goal_cart.linear_speed        = linear_speed      ;
//     goal_cart.angular_speed       = angular_speed     ;
//     goal_cart.linear_tollerance   = linear_tollerance ;
//     goal_cart.angular_tollerance  = angular_tollerance;
//     goal_cart.reference_frame     = target_frame      ;
//     
//     Eigen::Quaterniond q;
//     q = Eigen::AngleAxisd(target_delta_pos[3], Eigen::Vector3d::UnitZ())
//       * Eigen::AngleAxisd(target_delta_pos[4], Eigen::Vector3d::UnitY())
//       * Eigen::AngleAxisd(target_delta_pos[5], Eigen::Vector3d::UnitX());
//     
//     goal_cart.displacement = {target_delta_pos[0]
//                             , target_delta_pos[1]
//                             , target_delta_pos[2]
//                             , q.x()
//                             , q.y()
//                             , q.z()
//                             , q.w()
//     };
    
    
    return true;
  }
  
};

bool getParams(ros::NodeHandle nh_, Task& task)
{

  ROS_INFO( "Getting Params of task: %s", task.id.c_str());
  GET_AND_RETURN(nh_,task.id+"/configuration",task.configuration);
  
  std::string ac_type;
  GET_AND_RETURN(nh_,task.id+"/action_type", ac_type);
  std::transform(ac_type.begin(), ac_type.end(), ac_type.begin(), ::tolower);
  if(!action_map_.count(ac_type))
  {
    ROS_ERROR("[%s] does not exist. return",ac_type.c_str());
    return false;
  }
  else
  {
    task.action_type = action_map_.at(ac_type);
  }
  GET_AND_RETURN (nh_,task.id+"/action_name", task.action_name );
  GET_AND_DEFAULT(nh_,task.id+"/replan"     , task.replan, false);
    
  std::vector<double> zero_def  = {0,0,0,0,0,0};
  std::vector<double> zero5_def = {0.5,0.5,0.5,0.5,0.5,0.5};
  std::vector<double> zero_2    = {0,0};
  
  switch(task.action_type)
  {    
    case(action_type::none):
      break;
    case(action_type::FollowJointTrajectory):
    {
      GET_AND_RETURN (nh_,task.id+"/traj_id"        , task.traj_id             );
      GET_AND_DEFAULT(nh_,task.id+"/plan_correction", task.plan_correction,true);
      break;
    }
    case(action_type::simpleTouch):
    {
      GET_AND_DEFAULT(nh_,task.id+"/goal_twist"         , task.goal_twist      , zero_def);
      GET_AND_DEFAULT(nh_,task.id+"/target_wrench"      , task.target_wrench   , zero_def);
      GET_AND_DEFAULT(nh_,task.id+"/wrench_toll"        , task.goal_toll       , zero_def);
      GET_AND_DEFAULT(nh_,task.id+"/wrench_deadband"    , task.wrench_deadband , zero_def);
      GET_AND_DEFAULT(nh_,task.id+"/target_wrench_frame", task.target_frame    , "base"  );
      GET_AND_DEFAULT(nh_,task.id+"/goal_twist_frame"   , task.goal_twist_frame, "base"  );
      
      break;
    }    
//    case(action_type::cartesianMotion):
//    {
//      GET_AND_DEFAULT(nh_,task.id+"/linear_speed"    , task.linear_speed      , 0       );
//      GET_AND_DEFAULT(nh_,task.id+"/angular_speed"   , task.angular_speed     , 0       );
//      GET_AND_DEFAULT(nh_,task.id+"/linear_toll"     , task.linear_tollerance , 0       );
//      GET_AND_DEFAULT(nh_,task.id+"/angular_toll"    , task.angular_tollerance, 0       );
//      GET_AND_DEFAULT(nh_,task.id+"/target_delta_pos", task.target_delta_pos  , zero_def);
//      GET_AND_DEFAULT(nh_,task.id+"/reference_frame" , task.target_frame      , "base"  );
//      break;
//    }
  }
  
  GET_AND_DEFAULT(nh_,task.id+"/attach", task.attach,false);
  GET_AND_DEFAULT(nh_,task.id+"/detach", task.detach,false);
  
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller_switch");
  ros::NodeHandle nh;
  
  ros::ServiceClient configuration_srv= nh.serviceClient<configuration_msgs::StartConfiguration>("/configuration_manager/start_configuration");
  configuration_srv.waitForExistence();
  configuration_msgs::StartConfiguration start;
  
  std::vector<std::string> tasks;
  nh.getParam("tasks",tasks);

  for (size_t i;i<tasks.size();i++)
      ROS_FATAL_STREAM(tasks[i]);


  for (auto t : tasks)
  {

  // oggetto appositamente definito per gestire i parametri relativi al task corrente
    Task task = Task(t);


  // funzione appositamente definita per recuperare i parametri appartenenti al task corrente
    getParams(nh,task);

  // il robot viene messo nella configurazione richiesta dal task corrente
    start.request.start_configuration = task.configuration;
    start.request.strictness = 1;
    configuration_srv.call(start);

  // eseguo la action relativa al task corrente
    switch(task.action_type)
    {
      case(action_type::none):
      {      
        break;
      }
      case(action_type::FollowJointTrajectory):
      {
        task.getGoalTrajectory(nh);                                                                                             // recupero la traiettoria precalcolata
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> execute_trajectory ( task.action_name, true ); // creo action client 
        execute_trajectory.sendGoal ( task.goal_trajectory );                                                                   // eseguo la traiettoria
        execute_trajectory.waitForResult();                                                                                     // aspetto che sia finita
      }
      case(action_type::simpleTouch):
      {
         task.getGoalTouch();
         actionlib::SimpleActionClient<simple_touch_controller_msgs::simpleTouchAction> touch_action ( task.action_name, true );
         touch_action.sendGoal ( task.goal_touch);
         touch_action.waitForResult();
      }
//      case(action_type::cartesianMotion):
//      {
////         task.getGoalCart();
////         actionlib::SimpleActionClient<cartesian_motion_msgs::cartMotionAction> cart_action ( task.action_name, true );
////         cart_action.sendGoal ( task.goal_cart);
////         cart_action.waitForResult();
//      }
    }
  }






  return 0;
}
