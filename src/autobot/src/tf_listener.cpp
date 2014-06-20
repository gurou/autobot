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
 * File Name     : tf_listener.cpp
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
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <tf/exceptions.h>

void transformPoint(const tf::TransformListener& listener)
{
    // we'll create a point in the base_laser frame that we'd like to
    // transform to the base_link frame
    geometry_msgs::PointStamped laser_point;
    laser_point.header.frame_id = "base_laser";

    //we'll just use the most recent transform available for our simple
    // example
    laser_point.header.stamp = ros::Time();

    //just an arbitrary point in space
    laser_point.point.x = 1.0;
    laser_point.point.y = 2.0;
    laser_point.point.z = 0.0;

    geometry_msgs::PointStamped base_point;

    try {
        listener.transformPoint("base_link", laser_point, base_point);

        ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f,"
                    "%.2f, %.2f) at time %.2f", laser_point.point.x,
                    laser_point.point.y, laser_point.point.z, base_point.point.x,
                    base_point.point.y, base_point.point.z,
                    base_point.header.stamp.toSec());
    }
    catch(tf::TransformException& ex) {
        ROS_ERROR("Received an exception trying to transform a point from"
                    "\"base_laser\" to \"base_link\": %s", ex.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "youbot_tf_listener");
    ros::NodeHandle node;

    tf::TransformListener listener(ros::Duration(10));

    //we'll transform a point once every second
    ros::Timer timer = node.createTimer(ros::Duration(1.0),
                            boost::bind(&transformPoint, boost::ref(listener)));

    ros::spin();
}
