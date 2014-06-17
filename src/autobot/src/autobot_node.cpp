#include <ros/ros.h>
#include <sstream>
#include <iostream>
using namespace std;

#include <boost/lexical_cast.hpp>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
    float xCoordinate = 0.0;

    if(argc != 2)
    {
      cout << "Incorrect arguments" << endl;
      return 0;
    }
    else
    {
      try {
        xCoordinate = boost::lexical_cast<float>( argv[1] );
      }
      catch( boost::bad_lexical_cast const& ) {
        cout << "Error: input string was not valid" << std::endl;
        cout << "Cannot send goal to YouBot";
        return 0;
      }
    }

    ros::init(argc, argv, "navigation_goals");
    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0)))
    {
            ROS_INFO("Waiting for the move_base action server");
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = xCoordinate;
    goal.target_pose.pose.position.y = 0;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("You have arrived to the goal position");
    else
    {
            ROS_INFO("The base failed for some reason");
    }

    return 0;
}



