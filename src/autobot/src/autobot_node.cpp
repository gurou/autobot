/*******************************************************************************
 *
 * Copyright (c) 2014
 * All rights reserved.
 *
 * Hochschule Rosenheim
 * University of Applied Sciences
 * Electrical Engineering & Information Technology Department
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File Name     : autobot_node.cpp
 * Created on    : Jun 16, 2014
 * Author        : Guru, Julian, Andreas
 * Supervised by : Prof. Dr. Wolfgang Schittenhelm
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * This software is published under MIT License.
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Hochschule Rosenheim nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the MIT License as published by the Open Source
 * Initiative.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the MIT
 * License for more details.
 *
 * You should have received a copy of the MIT License along with this program.
 *
 ******************************************************************************/
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



