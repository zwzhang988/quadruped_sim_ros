#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <math.h>
#include <cmath>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <sstream>

#define PI M_PI

#include <eigen3/Eigen/Dense>

#include <my_legged_robot_msgs/FiveParametersOfTheFourLegs.h>


enum tag
{
    STOP,
    WALK_FORWARD,
    LEFT,
    RIGHT,
    WALK_BACKWARD,
    JUMP
};

tag robdog_motion_mode = STOP;

struct Parameter
{
    double Phase;
    double Skew;
    double Amp;
    double Ampback;
    double Freq;

    Parameter(double phase, double skew, double amp, double ampback, double freq)
        : Phase(phase), Skew(skew), Amp(amp), Ampback(ampback), Freq(freq) {}
};

class controllerNode
{
    ros::NodeHandle nh;

    ros::Publisher commands;
    ros::Subscriber listener;
    ros::Subscriber keyboard_listener; // 添加订阅键盘输入的订阅者
    ros::Timer timer;

    // 添加发布器
    ros::Publisher five_parameters_pub;

    Eigen::Vector3d x;     // current position of the UAV's c.o.m. in the world frame
    Eigen::Vector3d v;     // current velocity of the UAV's c.o.m. in the world frame
    Eigen::Matrix3d R;     // current orientation of the UAV
    Eigen::Vector3d omega; // current angular velocity of the UAV's c.o.m. in the *body* frame

    Eigen::Vector3d xd;    // desired position of the UAV's c.o.m. in the world frame
    Eigen::Vector3d vd;    // desired velocity of the UAV's c.o.m. in the world frame
    Eigen::Vector3d ad;    // desired acceleration of the UAV's c.o.m. in the world frame
    double yawd;           // desired yaw angle

    Parameter parameters = Parameter(0, 0, 0, 0, 0);

    double hz; // frequency of the main control loop

    bool jump_mode = false; // 标志是否处于 JUMP 模式

public:
    controllerNode() : hz(1000.0)
    {
        commands = nh.advertise<mav_msgs::Actuators>("commands", 1);
        listener = nh.subscribe("cmd_vel", 1000, &controllerNode::commandCallback, this);
        keyboard_listener = nh.subscribe("/keyboard_input", 1000, &controllerNode::keyboardCallback, this); // 订阅键盘输入
        timer = nh.createTimer(ros::Rate(hz), &controllerNode::controlLoop, this);

        // 初始化发布器
        five_parameters_pub = nh.advertise<my_legged_robot_msgs::FiveParametersOfTheFourLegs>("/my_legged_robot/five_parameters_four_legs_status", 10);
    }

    void commandCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        if (jump_mode)
        {
            // 如果处于 JUMP 模式，cmd_vel 话题的消息不再起作用
            return;
        }

        float anti_vel_linear_threshold = 0.2;
        float anti_vel_angular_z_threshold = 0.2;

        float vel_linear_x = msg->linear.x;
        float vel_angular_z = msg->angular.z;

        if (vel_linear_x == 0 && vel_angular_z == 0)
        {
            robdog_motion_mode = STOP;
        }
        else if ((vel_linear_x > 0.1) && (std::abs(vel_angular_z) < 0.3))
        {
            robdog_motion_mode = WALK_FORWARD;
        }
        else if (((vel_angular_z > 0) && (std::abs(vel_linear_x) < 0.1)) || ((std::abs(vel_linear_x) > 0.1) && (vel_angular_z > 0.3)))
        {
            robdog_motion_mode = LEFT;
        }
        else if (((vel_angular_z < 0) && (std::abs(vel_linear_x) < 0.1)) || ((std::abs(vel_linear_x) > 0.1) && (vel_angular_z < -0.3)))
        {
            robdog_motion_mode = RIGHT;
        }
        else if ((vel_linear_x < 0))
        {
            robdog_motion_mode = WALK_BACKWARD;
        }
        else
        {
            robdog_motion_mode = STOP;
        }

        std::stringstream ss;

        switch (robdog_motion_mode)
        {
        case STOP:
            this->parameters = Parameter(0, 0, 0, 0, 0);
            ss << "STOP - linear_velocity_x: " << vel_linear_x << ", angular_velocity: " << vel_angular_z;
            ROS_INFO("%s", ss.str().c_str());
            break;

        case WALK_FORWARD:
            this->parameters = Parameter(0, 90, 0, 0, 12);
            ss << "WALK_FORWARD - linear_velocity_x: " << vel_linear_x << ", angular_velocity: " << vel_angular_z;
            ROS_INFO("%s", ss.str().c_str());
            break;

        case LEFT:
            this->parameters = Parameter(0, -45, 0, 0, 7);
            ss << "LEFT - linear_velocity_x: " << vel_linear_x << ", angular_velocity: " << vel_angular_z;
            ROS_INFO("%s", ss.str().c_str());
            break;

        case RIGHT:
            this->parameters = Parameter(0, 45, 0, 0, 7);
            ss << "RIGHT - linear_velocity_x: " << vel_linear_x << ", angular_velocity: " << vel_angular_z;
            ROS_INFO("%s", ss.str().c_str());
            break;

        case WALK_BACKWARD:
            this->parameters = Parameter(0, 90, 0, 0, 3);
            ss << "WALK_BACKWARD - linear_velocity_x: " << vel_linear_x << ", angular_velocity: " << vel_angular_z;
            ROS_INFO("%s", ss.str().c_str());
            break;

        default:
            this->parameters = Parameter(0, 0, 0, 0, 0);
            ss << "NOTHING - linear_velocity_x: " << vel_linear_x << ", angular_velocity: " << vel_angular_z;
            ROS_INFO("%s", ss.str().c_str());
            break;
        }

        // 发布新的五参数消息
        my_legged_robot_msgs::FiveParametersOfTheFourLegs param_msg;
        param_msg.first_parameter = this->parameters.Phase;
        param_msg.second_parameter = this->parameters.Skew;
        param_msg.third_parameter = this->parameters.Amp;
        param_msg.fourth_parameter = this->parameters.Ampback;
        param_msg.fifth_parameter = this->parameters.Freq;
        five_parameters_pub.publish(param_msg);
    }

    void keyboardCallback(const std_msgs::String::ConstPtr &msg)
    {
        std::string input = msg->data;

        if (input == "JUMP")
        {
            jump_mode = true;
            this->parameters = Parameter(37, 0, 37, 13, 7);
            ROS_INFO("Entering JUMP mode");
        }
        else if (input == "NORMAL")
        {
            jump_mode = false;
            // 清空参数，等待下一次 cmd_vel 消息来设置新的参数
            this->parameters = Parameter(0, 0, 0, 0, 0);
            ROS_INFO("Exiting JUMP mode");
        }

        // 发布新的五参数消息
        my_legged_robot_msgs::FiveParametersOfTheFourLegs param_msg;
        param_msg.first_parameter = this->parameters.Phase;
        param_msg.second_parameter = this->parameters.Skew;
        param_msg.third_parameter = this->parameters.Amp;
        param_msg.fourth_parameter = this->parameters.Ampback;
        param_msg.fifth_parameter = this->parameters.Freq;
        five_parameters_pub.publish(param_msg);
    }

    void controlLoop(const ros::TimerEvent &t)
    {
        ros::Rate rate(10.0);

        int count = 3;

        mav_msgs::Actuators msg;

        msg.angular_velocities.resize(5);
        msg.angular_velocities[0] = this->parameters.Phase;   // Phase between front and back legs (in degree)
        msg.angular_velocities[1] = this->parameters.Skew;    // Phase between front left + back right legs and front right and left back legs
        msg.angular_velocities[2] = this->parameters.Amp;     // Amplitude change of all legs
        msg.angular_velocities[3] = this->parameters.Ampback; // Amplitude change of back legs (added to angular_velocities[2])
        msg.angular_velocities[4] = this->parameters.Freq;    // Frequency of legs

        for (int i = 0; i < count; i++)
        {
            commands.publish(msg);
            rate.sleep();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_node");
    ROS_INFO_NAMED("controller", "Controller started!");
    controllerNode n;
    ros::spin();
}





// #include <ros/ros.h>

// #include <ros/console.h>

// #include <tf/transform_datatypes.h>
// #include <tf_conversions/tf_eigen.h>
// #include <eigen_conversions/eigen_msg.h>
// #include <mav_msgs/Actuators.h>
// #include <nav_msgs/Odometry.h>
// #include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
// #include <math.h>
// #include <cmath>
// #include <std_msgs/Float64.h>
// #include <geometry_msgs/Twist.h>
// #include <sstream> 

// #define PI M_PI


// #include <eigen3/Eigen/Dense>

// // If you choose to use Eigen, tf provides useful functions to convert tf 
// // messages to eigen types and vice versa, have a look to the documentation:
// // http://docs.ros.org/melodic/api/eigen_conversions/html/namespacetf.html
// #include <eigen_conversions/eigen_msg.h>


// enum tag
// {
//     STOP,
//     WALK_FORWARD,
//     LEFT,
//     RIGHT,
//     WALK_BACKWARD,
//     JUMP
// };

// tag robdog_motion_mode = STOP;




// struct Parameter
// { 
//   double Phase;
//   double Skew;
//   double Amp;
//   double Ampback;
//   double Freq;

//   Parameter(double phase, double skew, double amp, double ampback, double freq) 
//   : Phase(phase), Skew(skew), Amp(amp), Ampback(ampback), Freq(freq) {}
// };

// class controllerNode
// {
//   ros::NodeHandle nh;

//   ros::Publisher commands;
//   ros::Subscriber listener;
//   ros::Timer timer;

//   // Controller internals (you will have to set them below)
//   // Current state
//   Eigen::Vector3d x;     // current position of the UAV's c.o.m. in the world frame
//   Eigen::Vector3d v;     // current velocity of the UAV's c.o.m. in the world frame
//   Eigen::Matrix3d R;     // current orientation of the UAV
//   Eigen::Vector3d omega; // current angular velocity of the UAV's c.o.m. in the *body* frame

//   // Desired state
//   Eigen::Vector3d xd;    // desired position of the UAV's c.o.m. in the world frame
//   Eigen::Vector3d vd;    // desired velocity of the UAV's c.o.m. in the world frame
//   Eigen::Vector3d ad;    // desired acceleration of the UAV's c.o.m. in the world frame
//   double yawd;           // desired yaw angle

//   // Parameters
//   Parameter parameters = Parameter(0,0,0,0,0);


//   double hz;             // frequency of the main control loop

// public:
//   controllerNode():hz(1000.0)
//   {
      
//       commands = nh.advertise<mav_msgs::Actuators>("commands", 1);
//       listener = nh.subscribe("cmd_vel",1000, &controllerNode::commandCallback, this); 
//       timer = nh.createTimer(ros::Rate(hz), &controllerNode::controlLoop, this);
//   } // END:controllerNode():hz(1000.0)


//   void commandCallback(const geometry_msgs::Twist::ConstPtr msg)
//   {

//     float anti_vel_linear_threshold = 0.2; 
//     // To resist minor disturbances, sometimes we just need to focus on turning around. There is no need to pay attention to minor values of vel_linear_x or y.

//     float anti_vel_angular_z_threshold = 0.2;
//     // To resist minor disturbances, sometimes we just need to focus on moving forward. There is no need to pay attention to minor values of vel_angular_z.

//     float vel_linear_x = msg->linear.x;
//     float vel_linear_y = msg->linear.y;
//     float vel_angular_z = msg->angular.z;

//     if (vel_linear_x == 0 && vel_angular_z == 0)
//     {
//         robdog_motion_mode = STOP;
//     }

//     else if ((vel_linear_x > 0.1) && (std::abs(vel_angular_z) < 0.3)) // < anti_vel_angular_z_threshold
//     {
//         robdog_motion_mode = WALK_FORWARD;
//     }

//     // else if ((vel_angular_z > 0) && (std::abs(vel_angular_z) > 0.2) && (std::abs(vel_linear_x) < 0.1))
//     else if (((vel_angular_z > 0) && (std::abs(vel_linear_x) < 0.1)) || ((std::abs(vel_linear_x) > 0.1) && (vel_angular_z > 0.3)) )
//     {
//         robdog_motion_mode = LEFT;
//     }

//     // else if ((vel_angular_z < 0) && (std::abs(vel_angular_z) > 0.2) && (std::abs(vel_linear_x) < 0.1))
//     else if (((vel_angular_z < 0) && (std::abs(vel_linear_x) < 0.1)) || ((std::abs(vel_linear_x) > 0.1) && (vel_angular_z < -0.3)))
//     {
//         robdog_motion_mode = RIGHT;
//     }

//     else if ((vel_linear_x < 0))
//     {
//         robdog_motion_mode = WALK_BACKWARD;
//     }

//     else
//     {
//         robdog_motion_mode = STOP;
//     }

//     std::stringstream ss;  // 在switch外部声明

//     while(ros::ok())
//     {
//       ros::spinOnce();
//       switch(robdog_motion_mode)
//       {
//         case STOP:
//             this->parameters = Parameter(0,0,0,0,0);

//             ss.str("");  // 清空字符串流

//             ss << "STOP - linear_velocity_x: " << vel_linear_x << ", angular_velocity: " << vel_angular_z;

//             ROS_INFO("%s", ss.str().c_str());

//             break;

//         case WALK_FORWARD:
//             this->parameters = Parameter(0,90,0,0,12);

//             ss.str("");  // 清空字符串流

//             ss << "WALK_FORWARD - linear_velocity_x: " << vel_linear_x << ", angular_velocity: " << vel_angular_z;

//             ROS_INFO("%s", ss.str().c_str());

//             break;

//         case LEFT:
//             this->parameters = Parameter(0,-45,0,0,7);

//             ss.str("");  // 清空字符串流

//             ss << "LEFT - linear_velocity_x: " << vel_linear_x << ", angular_velocity: " << vel_angular_z;

//             ROS_INFO("%s", ss.str().c_str());

//             break;   

//         case RIGHT:
//             this->parameters = Parameter(0,45,0,0,7);

//             ss.str("");  // 清空字符串流

//             ss << "RIGHT - linear_velocity_x: " << vel_linear_x << ", angular_velocity: " << vel_angular_z;

//             ROS_INFO("%s", ss.str().c_str());

//             break;   
        
//         case WALK_BACKWARD:
//             this->parameters = Parameter(0,90,0,0,3);

//             ss.str("");  // 清空字符串流

//             ss << "WALK_BACKWARD - linear_velocity_x: " << vel_linear_x << ", angular_velocity: " << vel_angular_z;

//             ROS_INFO("%s", ss.str().c_str());

//             break;  

//         default:
//             this->parameters = Parameter(0,0,0,0,0);

//             ss.str("");  // 清空字符串流

//             ss << "NOTHING - linear_velocity_x: " << vel_linear_x << ", angular_velocity: " << vel_angular_z;

//             ROS_INFO("%s", ss.str().c_str());

//             break;                           
//       }

//     } //END: while(fos::ok())




    
//     // float angular_velocity = msg->angular.z; 
//     // // double linear_velocity = std::sqrt(vel_x*vel_x + vel_y*vel_y);
  	
//     // if (abs(angular_velocity) < std::pow(10, -2))
//     // {
//     //   if (vel_x < std::pow(10, -2)) 
//     //   {
//     //     this->parameters = Parameter(0,0,0,0,0);

//     //     std::stringstream ss;

//     //     ss << "Stop - vel_x: " << vel_x << ", angular_velocity: " << angular_velocity;

//     //     ROS_INFO("%s", ss.str().c_str());
//     //   }
//     //   else 
//     //   {
//     //     this->parameters = Parameter(0,90,0,0,12);
//     //     ROS_INFO("Walking");
//     //   }
//     // } 
//     // else 
//     // {

//     //   if (angular_velocity<0) 
//     //   {
//     //   this->parameters = Parameter(0,45,0,0,7);
//     //   ROS_INFO("Turning right");
//     //   } 
//     //   else if (angular_velocity>0)
//     //   {
//     //   this->parameters = Parameter(0,-45,0,0,7);
//     //   ROS_INFO("Turning left");
//     //   }

//       // std::stringstream ss;

//       // ss << "Turning - vel_x: " << vel_x << ", angular_velocity: " << angular_velocity;

//       // ROS_INFO("%s", ss.str().c_str());
//   //   }
//   } // END: void commandCallback(const geometry_msgs::Twist::ConstPtr msg)

//   void controlLoop(const ros::TimerEvent& t)
//   {

//     ros::Rate rate(10.0);
//     // 设置频率：
//     // 这行代码创建了一个ros::Rate对象rate，其频率参数为10.0赫兹（Hz）。也就是说，循环每秒钟应该执行10次。
//     // 在循环中使用：
//     // 在主循环中，每次迭代末尾调用rate.sleep()，这会根据设定的频率让循环休眠一段时间，以保持循环频率稳定在10Hz。
    
//     int count = 3;

//     mav_msgs::Actuators msg;

//     msg.angular_velocities.resize(5);
//     msg.angular_velocities[0] = this->parameters.Phase; // Phase between front and back legs (in degree)
//     msg.angular_velocities[1] = this->parameters.Skew; // Phase between front left + back right legs and front right and left back legs
//     msg.angular_velocities[2] = this->parameters.Amp; // Amplitude change of all legs
//     msg.angular_velocities[3] = this->parameters.Ampback; // Amplitude change of back legs (added to angular_velocities[2])
//     msg.angular_velocities[4] = this->parameters.Freq; // Frequency of legs

//     for (int i = 0; i < count; i++)
//     {
//         commands.publish(msg);
//         rate.sleep();
//     }

//   } // END: void controlLoop(const ros::TimerEvent& t)

// }; // END: class controllerNode


// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "controller_node");
//   ROS_INFO_NAMED("controller", "Controller started!");
//   controllerNode n;
//   ros::spin();
// }
