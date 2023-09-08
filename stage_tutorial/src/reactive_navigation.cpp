#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"


double obstacle_distance;
bool robot_stopped;
int state_ = 0;
int follow_dir = -1;

geometry_msgs::Twist velocity;


struct Section
{
  float front;
  float left;
  float right;
} section;

std::map<int, std::string> state_dict_ = {
  {0, "Find wall"},
  {1, "Turn right"},
  {2, "Follow the wall"},
  {3, "Turn left"},
  {4, "Diagonally right"},
  {5, "Diagonally left"},
  {6, "STOP"}
};

//THIS CHANGES THE STATE OF THE FOLLOW WALL MOVEMNTS AND SENDS THE INFORMATION TO CONSOLE
void change_state(int state)
{
  if (state != state_)
  {
    ROS_INFO("State of Bot - [%d] - %s", state, state_dict_[state].c_str());
    state_ = state;
  }
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    std::vector<float> laser_ranges = msg->ranges;

    if (!robot_stopped)
    {
        ROS_INFO("Received a LaserScan with %i samples", (int) laser_ranges.size());

        obstacle_distance = *std::min_element (laser_ranges.begin(), laser_ranges.end());
        ROS_INFO("minimum distance to obstacle: %f", obstacle_distance);
    }

    section.right = *std::min_element(laser_ranges.begin() , laser_ranges.begin() + 80); //80
    section.front = *std::min_element(laser_ranges.begin() + 81, laser_ranges.begin() + 160); //160
    section.left = *std::min_element(laser_ranges.begin() + 161, laser_ranges.end()); //240

    //printf("Tamanho ranges: %li", laser_ranges.size());
}


geometry_msgs::Twist follow_wall()
{

    float b = 1.1; // maximum threshold distance
    float a = 0.6; // minimum threshold distance
    //geometry_msgs::Twist velocity;
    float linear_x = 0;
    float angular_z = 0;

    ROS_INFO("follow_direction %d", follow_dir);
    //IF ALL REGIONS ONF THE LASER ARE BIGGER THEN THE max threshold IT WILL CHANGE STATE TO FIND THE WALL
    if (section.front > b && section.left > b && section.right > b)
    {
        // Reset follow_dir
        change_state(0);
        ROS_INFO("Reset Follow_dir");
    }
    else if (follow_dir == -1)  // To set the direction of wall to follow
    {
        if (section.left < b)
        {
            change_state(1);
            follow_dir = 0;
            ROS_INFO("following left wall");
        }
        else if (section.right < b)
        {
            change_state(3);
            follow_dir = 1;
            ROS_INFO("following right wall");
        }
        else
        {
            change_state(2);
            ROS_INFO("following front wall");
        }
    }
    else if(section.front < a && section.left < a && section.right < a){ //IF ALL REGIONS ARE TOO CLOSE TO THE WALL THE ROBOT STOPS
        ROS_INFO("Too Close");
        change_state(6);
        ROS_INFO("STOP");
    }else{
        ROS_INFO("Running");
    }

    if (follow_dir == 0) // Algorithm for left wall follower
    {
        //ACTIONS TO DO DEPENDING ON THE LASER RANGE
        if (section.left > b && section.front > a)
        {
            change_state(5);
        }
        else if (section.left < b && section.front > a)
        {
            change_state(2);
        }
        else if(section.left < b && section.front < a)
        {
            change_state(1);
        }
        else if(section.left < a)
        {
            change_state(4);
        }
        else
        {
            follow_dir = -1;
            ROS_INFO("Follow left wall is not running");
        }
    }
    else if (follow_dir == 1) // Algorithm for right wall follower
    {
        if (section.right > b && section.front > a)
        {
            change_state(4);
        }
        else if (section.right < b && section.front > a)
        {
            change_state(2);
        }else if (section.right < b && section.front < a){
            change_state(3);
        }else if(section.right < a){
            change_state(5);
        }else{
            follow_dir = -1;
            ROS_INFO("Follow right wall is not running");
        }
    }

    //VELOCITIES TO EACH ACTION
    if (state_ == 0)
    {
        linear_x = 0.4;
        angular_z = 0.0;
    }
    else if (state_ == 1)
    {
        linear_x = 0.0;
        angular_z = -0.5;
    }
    else if (state_ == 2)
    {
        linear_x = 0.4;
        angular_z = 0.0;
    }
    else if (state_ == 3)
    {
        linear_x = 0.0;
        angular_z = 0.5;
    }
    else if (state_ == 4)
    {
        linear_x = 0.4;
        angular_z = -0.2;
    }
    else if (state_ == 5)
    {
        linear_x = 0.4;
        angular_z = 0.2;
    }else if(state_ == 6){
        linear_x = 0.0;
        angular_z = 0.0;
    }else{
        ROS_INFO("Unknown state!");
    }

    velocity.linear.x = linear_x;
    velocity.angular.z = angular_z;

    return velocity;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reactive_navigation");

    ros::NodeHandle n;

    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);

    ros::Subscriber laser_sub = n.subscribe("base_scan", 100, laserCallback);

    robot_stopped = true;

    ros::Rate loop_rate(10);

    while(ros::ok)
    {
        follow_wall();
        cmd_vel_pub.publish(velocity);
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spinOnce();
    loop_rate.sleep();
}