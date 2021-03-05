/* copyright[2019] <msl>
**************************************************************************
  File Name    : setpoint_publisher.cpp
  Author       : Kunal Shah, Jun En Low, Alexander Koufos
                 Multi-Robot Systems Lab (MSL), Stanford University
  Contact      : k2shah@stanford.edu
  Create Time  : Feb 9, 2021.
  Description  : px4 setpoint publisher class. Isolates and ensures mavros
                 requirements are satisfied at all times.
**************************************************************************/

#include<mslquad/setpoint_publisher.h>
#include<cmath>

SetpointPublisher::SetpointPublisher() {
  // TO THE MOON

  // resolve parameters
  ros::param::get("~m_namespace", m_namespace);
  ROS_INFO_STREAM(m_namespace << " initalizing");
  ros::param::get("~takeoff_height", m_takeoffHeight);
  ros::param::get("~max_vel", m_maxVel);
  ros::param::get("~control_freq", m_controlRate);

  // init state machine
  m_state = State::INIT;

  // PUBLISHERS
  pub_px4SetPose = m_nh.advertise<geometry_msgs::PoseStamped>(
        m_namespace +"mavros/setpoint_position/local", 1);
  
  pub_px4SetVel = m_nh.advertise<geometry_msgs::Twist>(
        m_namespace +"mavros/setpoint_velocity/cmd_vel_unstamped", 1);

  // SUBSCRIBERS
  //px4 subscribers
  sub_px4GetState = m_nh.subscribe<mavros_msgs::State>(
    m_namespace + "mavros/state", 1,
    &Pilot::sub_px4GetStateCB,this);

  sub_px4GetPose = m_nh.subscribe<geometry_msgs::PoseStamped>(
    m_namespace + "mavros/local_position/pose", 1,
    &Pilot::sub_px4GetPoseCB, this);
  
  sub_px4GetVel = m_nh.subscribe<geometry_msgs::TwistStamped>(
    m_namespace + "mavros/local_position/velocity", 1,
    &Pilot::sub_px4GetVelCB, this);

  // flight room
  sub_vrpn = m_nh.subscribe<geometry_msgs::PoseStamped>(
    m_namespace + "mavros/vision_pose/pose", 1,
    &Pilot::sub_vrpnCB, this);
  
  // user commands 
  sub_cmdPose = m_nh.subscribe<geometry_msgs::PoseStamped>(
    m_namespace + "command/pose", 1,
    &Pilot::sub_cmdPoseCB, this);
  
  sub_cmdVel = m_nh.subscribe<geometry_msgs::TwistStamped>(
    m_namespace + "command/velocity", 1,
    &Pilot::sub_cmdVelCB, this);
  
  // SERVICES
  landClient = m_nh.serviceClient<mavros_msgs::CommandTOL>(
    m_namespace + "mavros/cmd/land");

  // spawn timers 
  m_controlLoop = m_nh.createTimer(
        ros::Duration(1.0/controlRate),
        &Pilot::controlLoopCB, this);

  m_statusLoop = m_nh.createTimer(
      ros::Duration(.1),
      &Pilot::statusLoopCB, this);

  // run pre-flight checks
  // check for mavros
  preFlight();
}


// callback methods
void SetpointPublisher::controlLoopCB(const ros::TimerEvent& event) {
    switch (State) {
      case INIT:
        ROS_WARN("Initaliztion Failure")

        break
      case STANDBY:
        break;
      case TAKEOFF: takeoff();
        break;

      case HOVER:

      case FLIGHT:

      case LAND: land();
        break;

      case FAILSAFE:
        break

      default:
       ROS_WARN("UNDEFINED STATE")
       m_state = State::FAILSAFE;
    }
}

void SetpointPublisher::statusLoopCB(void) {
    // pose time delay check
    this -> poseDelay();
    // pose drift 
    this -> poseDrift(localPose.pose, vrpnPose.pose);
}

void SetpointPublisher::sub_px4GetStateCB(
  const mavros_msgs::State::ConstPtr& msg) {
    // store the current pose
    m_mavrosState = *msg;
}


void SetpointPublisher::sub_px4GetPoseCB(
  const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // store the current pose
    m_localPose = *msg;
}

void SetpointPublisher::sub_px4GetVelCB(
  const geometry_msgs::TwistStamped::ConstPtr& msg) {
    // store the current velocity
    m_localVel = *msg;
}

void SetpointPublisher::sub_vrpnCB(
  const geometry_msgs::PoseStamped::ConstPtr& msg){
    // store the VRPN pose
    vrpnPose = *msg
}

void SetpointPublisher::preFlight(void){
  ROS_INFO_STREAM(m_namespace << " waiting for internal pose.");
  while (ros::ok() && m_localPose.header.seq < 100) {
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }
  ROS_INFO("Inital Pose: (%f, %f, %f)",
            m_localPose.position.x,
            m_localPose.position.y,
            m_localPose.position.z);
  m_initPose = m_localPose; 


  if (m_takeoffHeight > 0){
    auto
  }
}

void SetpointPublisher::takeoff(void){

}

void SetpointPublisher::land(void){
  if (land_client.call(srv_land) && srv_land.response.success)
    {
      ROS_INFO("Land Sent %d", srv_land.response.success);
    }
  while
}


bool SetpointPublisher::landServiceHandle(
        mslquad::Land::Request &req,
        mslquad::Land::Response &res) {
    m_landPose = req.landpos;
    ROS_WARN("LANDING");
    ROS_INFO("Pose: (%f, %f, %f)",
            landPose.position.x,
            landPose.position.y,
            landPose.position.z);
    m_state = State::LAND
    res.success = true;
    return true;
}
void SetpointPublisher::poseDelay(void){
  // find delay since last pose 
    double delay = ros::Time::now().toSec() - localPose.header.stamp.toSec();
    if (delay > 0.5) {
        ROS_ERROR("Critical Pose DELAY Detected");
        State = LAND;
    } else if (poseTimeDiff_.toSec() > 0.15) {
        ROS_WARN("Moderate Pose DELAY Detected");
    }
}

void SetpointPublisher::poseDrift(Pose p1, Pose p2){
  dx = p1.position.x - p1.position.x
  dy = p1.position.y - p1.position.y
  dz = p1.position.z - p1.position.z
  double drift = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2))
  if (drift > .01 ){
    ROS_ERROR("Critical Pose DRIFT Detected");
        State = LAND;
  } 

}

// void SetpointPublisher::passback(const geometry_msgs::PoseStamped::ConstPtr& msg,
//                 geometry_msgs::PoseStamped& receipt) {
//   receipt = *msg;
// }

SetpointPublisher::~SetpointPublisher() {
  ROS_WARN("Terminating Controller");
}

int main(int argc, char const *argv[]) {
    SetpointPublisher sp;
    ROS_INFO(sp.m_namespace+" initialized")
    ros::spin();
    return 0;
  }
