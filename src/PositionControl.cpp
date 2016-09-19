#include "ros/ros.h"
#include "std_msgs/String.h"
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>


class ControlNodeMaster
{
public:
  ControlNodeMaster()
  {

    pub_ = n_.advertise<trajectory_msgs::JointTrajectoryPoint>("/master/joint_traj_pt_cmd", 1000);
    sub_ = n_.subscribe("/slave/joint_states", 1000, &ControlNodeMaster::masterCallback, this);

  }

  void masterCallback(const sensor_msgs::JointState& msg)
  {
      ROS_INFO("commandig joint positions to master robot:");
      trajectory_msgs::JointTrajectoryPoint msg_out;
      msg_out.positions = msg.position;
      pub_.publish(msg_out);
  }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

};

class ControlNodeSlave
{
public:
  ControlNodeSlave()
  {

    pub_ = n_.advertise<trajectory_msgs::JointTrajectoryPoint>("/slave/joint_traj_pt_cmd", 1000);
    sub_ = n_.subscribe("/master/joint_states", 1000, &ControlNodeSlave::slaveCallback, this);

  }

  void slaveCallback(const sensor_msgs::JointState& msg)
  {
      ROS_INFO("commandig joint positions to slave robot:");
      trajectory_msgs::JointTrajectoryPoint msg_out;
      msg_out.positions = msg.position;
      pub_.publish(msg_out);
  }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

};

int Usage (int argc, char *argv[])
{

    int val;  //return value, 0 == master, 1 == slave

    if (argc ==1){

      printf("\n Usage: \n"
          "\t[-master] : If you want to use for master node\n"
          "\t[-slave] : If you want to use for slave node\n"
          "\n");
      exit(1);
    }

    while (--argc > 0) {
        argv++;

        if (!strncmp(*argv, "-master", 7)) {
           val = 0;

        }  else if (!strncmp(*argv, "-slave", 6)){
           val = 1;

        }  else {
            printf("\n Usage: \n"
                "\t[-master] : If you want to use for master node\n"
                "\t[-slave] : If you want to use for slave node\n"
                "\n");
            exit(1);
        }

        argc--; argv++;
    }

    if (val == 0)
        printf("\nStarting controller for master node:\n");

    if (val == 1)
        printf("\nStarting controller for slave node:\n");

    return val;
}

int main(int argc, char **argv)
{
  int node;
  ros::init(argc, argv, "ControlNodeMaster");
  node = Usage(argc, argv);

if (node == 0) {

  ControlNodeMaster controlNodeMasterObj;
  ros::spin();
  return 0;

} else if (node == 1){

  ControlNodeSlave controlNodeSlaveObj;
  ros::spin();
  return 0;

} else {

  return -1;
}

}
