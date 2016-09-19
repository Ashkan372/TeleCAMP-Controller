 #include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Wrench.h>
#include <cartesian_impedance_msgs/SetCartesianImpedance.h>
#include <cmath>

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

// on master side

class ControlNodeMaster
{
public:
  ControlNodeMaster(double proportional, double differntial): kp(proportional), kd(differntial)
  {

    pub_ =  n_.advertise<cartesian_impedance_msgs::SetCartesianImpedance>("/master/set_cartesian_impedance_params", 1000);
    sub_slave_wrench = n_.subscribe("/slave/wrench_r", 1000, &ControlNodeMaster::slaveWrenchCallback, this);
    sub_master_wrench = n_.subscribe("/master/wrench_r", 1000, &ControlNodeMaster::masterWrenchCallback, this);

    masterForceX = 0;
    masterForceY = 0;
    masterForceZ = 0;

     maxStiffness = 1000;
     minStiffness = 50;
     maxForce = 60;
     minForce = 4;

    small_increment = 0.06;
    medium_increment = 0.06;
    big_increment = 0.3;

    currentStiffnessX = 50;
    currentStiffnessY = 50;
    currentStiffnessZ = 50;

    oldErrorZ = 0;
    oldErrorY = 0;
    oldErrorX = 0;

  }


      void masterWrenchCallback(const geometry_msgs::Wrench& msg)
      {

      mutex.lock();

      masterForceX = std::abs(msg.force.x);
      masterForceY = std::abs(msg.force.y);
      masterForceZ = std::abs(msg.force.z);

      mutex.unlock();


      }

      void slaveWrenchCallback(const geometry_msgs::Wrench& msg)
      {

      int  slaveForceX = std::abs(msg.force.x);
      int  slaveForceY = std::abs(msg.force.y);
      int  slaveForceZ = std::abs(msg.force.z);

      mutex.lock();

      double errorX = slaveForceX - masterForceX;
      double errorY = slaveForceY - masterForceY;
      double errorZ = slaveForceZ - masterForceZ;

      mutex.unlock();

      cartesian_impedance_msgs::SetCartesianImpedance cart_vals;

    //Z

          if ((slaveForceZ < minForce)){

            currentStiffnessZ = currentStiffnessZ - small_increment;
          }


           if ( ( currentStiffnessZ >= 50) && (currentStiffnessZ < 100) ){
             currentStiffnessZ = currentStiffnessZ + (errorZ * kp) + (errorZ - oldErrorZ) * kd;
           } else {
             currentStiffnessZ = currentStiffnessZ + (errorZ * kp) + (errorZ - oldErrorZ) * kd;
           }



      // Check stiffness limits
      if (currentStiffnessZ < minStiffness) currentStiffnessZ = minStiffness;
      if (currentStiffnessZ > maxStiffness) currentStiffnessZ = maxStiffness;

      ROS_INFO("ErrorZ: %f", errorZ);
      ROS_INFO("current StiffnessZ: %f, kp = %f, kd = %f", currentStiffnessZ, kp, kd);


      // creating the message
      cart_vals.stiffness.translational.z = currentStiffnessZ ;


      // Y

            if ((slaveForceY < minForce)){

              currentStiffnessY = currentStiffnessY - small_increment;
            }


             if ( ( currentStiffnessY >= 50) && (currentStiffnessY < 100) ){
               currentStiffnessY = currentStiffnessY + errorY * kp + (errorY - oldErrorY) * kd;
             } else {
               currentStiffnessY = currentStiffnessY + errorY * kp + (errorY - oldErrorY) * kd;
             }


      // Check stiffness limits
      if (currentStiffnessY < minStiffness) currentStiffnessY = minStiffness;
      if (currentStiffnessY > maxStiffness) currentStiffnessY = maxStiffness;

      ROS_INFO("ErrorY: %f", errorY);
      ROS_INFO("current StiffnessY: %f", currentStiffnessY);


      // creating the message
      cart_vals.stiffness.translational.y = currentStiffnessY ;



      // X

            if ((slaveForceX < minForce)){

              currentStiffnessX = currentStiffnessX - small_increment;
            }


             if ( ( currentStiffnessX >= 50) && (currentStiffnessX < 100) ){
               currentStiffnessX = currentStiffnessX + errorX * kp + (errorX - oldErrorX) * kd;
             } else {
               currentStiffnessX = currentStiffnessX + errorX * kp + (errorX - oldErrorX) * kd;
             }



      // Check stiffness limits
      if (currentStiffnessX < minStiffness) currentStiffnessX = minStiffness;
      if (currentStiffnessX > maxStiffness) currentStiffnessX = maxStiffness;

      ROS_INFO("ErrorX: %f", errorX);
      ROS_INFO("current StiffnessX: %f", currentStiffnessX);


      // creating the message
      cart_vals.stiffness.translational.x = currentStiffnessX ;



                			//stiffness Rotational
                			cart_vals.stiffness.rotational.x = 200;
                			cart_vals.stiffness.rotational.y = 200;
                			cart_vals.stiffness.rotational.z = 200;

                			//damping Ttranslational
                			cart_vals.damping.translational.x = 0.7;
                			cart_vals.damping.translational.y = 0.7;
                			cart_vals.damping.translational.z = 0.7;
                			//damping Rotational
                			cart_vals.damping.rotational.x = 0.7;
                			cart_vals.damping.rotational.y = 0.7;
                			cart_vals.damping.rotational.z = 0.7;

                			//Maximum Cartesian Velocity Linear
                			cart_vals.max_cart_vel.set.linear.x = 1000;
                			cart_vals.max_cart_vel.set.linear.y = 1000;
                			cart_vals.max_cart_vel.set.linear.z = 1000;
                			//Maximum Cartesian Velocity Angular
                			cart_vals.max_cart_vel.set.angular.x = 6.3;
                			cart_vals.max_cart_vel.set.angular.y = 6.3;
                			cart_vals.max_cart_vel.set.angular.z = 6.3;


                			//Maximum Control Force Linear
                			cart_vals.max_ctrl_force.set.force.x = 200;
                			cart_vals.max_ctrl_force.set.force.y = 200;
                			cart_vals.max_ctrl_force.set.force.z = 200;
                			//Maximum Control Force Angular
                			cart_vals.max_ctrl_force.set.torque.x = 200;
                			cart_vals.max_ctrl_force.set.torque.y = 200;
                			cart_vals.max_ctrl_force.set.torque.z = 200;

                			//Maximum Cartesian Path Deviation Translation
                			cart_vals.max_path_deviation.translation.x = 1000;
                			cart_vals.max_path_deviation.translation.y = 1000;
                			cart_vals.max_path_deviation.translation.z = 1000;
                			//Maximum Cartesian Path Deviation Rotation
                			cart_vals.max_path_deviation.rotation.x = 500;
                			cart_vals.max_path_deviation.rotation.y = 500;
                			cart_vals.max_path_deviation.rotation.z = 500;

                			//NullSpace reduntant joint parameters
                			cart_vals.null_space_params.stiffness.push_back(100);
                		  cart_vals.null_space_params.damping.push_back(0.7);

                      pub_.publish(cart_vals);

                      oldErrorZ = errorZ;
                      oldErrorY = errorY;
                      oldErrorX = errorX;
      }


private:

  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_slave_wrench;
  ros::Subscriber sub_master_wrench;

  double masterForceX;
  double masterForceY;
  double masterForceZ;


   int maxStiffness;
   int minStiffness;
   int maxForce;
   int minForce;

  double small_increment;
  double medium_increment;
  double big_increment;

  double kp;
  double kd;

  double currentStiffnessX;
  double currentStiffnessY;
  double currentStiffnessZ;

  double oldErrorZ;
  double oldErrorY;
  double oldErrorX;

  boost::mutex mutex;

};

void Usage (int argc, char *argv[], double* kp, double* kd)
{

    *kp = 0.006;
    *kd = 0.010;
    if (argc ==1){
      printf("\nUsage: \n"
          "\t[-kp <proportional coef value>] : kp coefficient in pd controller\n"
          "\t[-kd <differentional coef value>] : kd coefficient in pd controller\n"
          "\n");
      printf("\nStarting with default values:\n");
    }

    while (--argc > 0) {
        argv++;

        if (!strncmp(*argv, "-kp", 3)) {
            sscanf(argv[1], "%lf", kp);
            argc--; argv++;

        } else if (!strncmp(*argv, "-kd", 3)) {
            sscanf(argv[1], "%lf", kd);
            argc--; argv++;

        } else {
            printf("\nUsage: \n"
                "\t[-kp <proportional coef value>] : kp coefficient in pd controller\n"
                "\t[-kd <differentional coef value>] : kd coefficient in pd controller\n"
                "\n");
            exit(1);
        }
    }

    printf("\nStarting execution with the following settings:\n"
        "\t kp = %lf\n"
        "\t kd = %lf\n"
        "\n",
        *kp, *kd);
}

int main(int argc, char **argv)
{

  // default values
  double k_p;
  double k_d;

  ros::init(argc, argv, "PDControlNodeMaster");

  Usage(argc, argv, &k_p, &k_d);

  ControlNodeMaster controlNodeMasterObj(k_p, k_d);



   ros::Rate loop_rate(100);

   while (ros::ok())
   {

   ros::spinOnce();
   loop_rate.sleep();

   }

  return 0;
}
