#ifndef _OPTITRACK_LISTENER_H_
#define _OPTITRACK_LISTENER_H_

#include "ros/ros.h"
//#include "geometry_msgs/PoseStamped.h"

class Optitrack{
private:
    ros::NodeHandle _nh;
    ros::Subscriber _sub_pose;
    
    void pose_callback(const geometry_msgs::PoseStampedConstPtr &ps);

public:
    geometry_msgs::PoseStamped pose_stamped; // latest pose read from Optitrack
    tf::Quaternion Q;
    float pos[3];
    float rpy[3]; // rpy[0]: roll, rpy[1]: pitch, rpy[2]: yaw

    void update_pos(void); // update pos[3] from pose_stamped
    void update_rpy(void);
  

    Optitrack();
    ~Optitrack();
};

#endif
