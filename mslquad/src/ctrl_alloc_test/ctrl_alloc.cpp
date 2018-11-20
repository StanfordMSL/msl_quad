
#include "ctrl_alloc.h"

ctrl_alloc::ctrl_alloc(int quad_count, float arm_length)
{
  // Setup the swarm's configuration (quad and motor positions).
  float base_angle = 2 * M_PI / quad_count;  

  for (int i = 0; i < quad_count; i++)
  {
    quad.push_back(profile_struct());
    quad[i].address = i;
    quad[i].x_offset = arm_length * cos(base_angle * i);
    quad[i].y_offset = arm_length * sin(base_angle * i);

    quad[i].rot_offset = 0;
    quad[i].arm_length = 0.2;
    
    quad[i].motor_prof[0] = 4.67636;
    quad[i].motor_prof[1] = 1.68915;
    quad[i].motor_prof[2] = -0.05628;
    quad[i].motor_prof[3] = 1.0/60.0;

    for (int j = 0; j < 2; j++)
    {
      float angle;

      angle = (base_angle * i) - (M_PI / 4) + (j * M_PI / 2);

      quad[i].motor_pos[2 * j][0] = quad[i].x_offset + (quad[i].arm_length * cos(angle));
      quad[i].motor_pos[2 * j][1] = quad[i].y_offset + (quad[i].arm_length * sin(angle));

      angle += M_PI;
      quad[i].motor_pos[(2 * j) + 1][0] = quad[i].x_offset + (quad[i].arm_length * cos(angle));
      quad[i].motor_pos[(2 * j) + 1][1] = quad[i].y_offset + (quad[i].arm_length * sin(angle));
    }
  }

  // Setup the variables for the math we'll be doing.
  A.resize(4, quad_count * 4);
  x_ln.resize(quad_count,1);

  Eigen::Vector4f feeder;
  for (int i = 0; i < quad_count; i++)
  {
    for (int j = 0; j < 4; j++)
    {

      feeder(0, 0) = 1;
      feeder(1, 0) = quad[i].motor_pos[j][1];
      feeder(2, 0) = -quad[i].motor_pos[j][0];
      if (j <= 1)
      {
        feeder(3, 0) = -quad[i].motor_prof[3];
      }
      else
      {
        feeder(3, 0) = quad[i].motor_prof[3];
      }
      A.col(4 * i + j) = feeder;
    }
  }
}

ctrl_alloc::~ctrl_alloc()
{
}

void ctrl_alloc::se3Callback(const std_msgs::Float32MultiArray::ConstPtr &command)
{    
  y(0,0) = command->data[0];
  y(1,0) = command->data[1];
  y(2,0) = command->data[2];
  y(3,0) = command->data[3];

  x_ln = A.transpose() * (A * A.transpose()).inverse() * y;
  y_tilde = A*x_ln;

  std::string sep = "\n----------------------------------------\n";
  std::cout << y_tilde << sep;

  //ROS_INFO("I received: [%f] [%f] [%f] [%f]", output.data[0], output.data[1], output.data[2], output.data[3]);
}

int main(int argc, char **argv)
{
  int quad_count = 4;
  float arm_length = 1.0;

  ctrl_alloc swarm(quad_count,arm_length);

  ros::init(argc, argv, "ctrl_alloc");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("se3_command", 1000,&ctrl_alloc::se3Callback, &swarm);

  ros::spin();


  return 0;
}