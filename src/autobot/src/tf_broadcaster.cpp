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
 * File Name     : tf_broadcaster.cpp
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
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "youbot_tf_broadcaster");
  ros::NodeHandle node;
  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;
  while (node.ok())
  {
    broadcaster.sendTransform(
        tf::StampedTransform(
            tf::Transform(
                tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.35, 0.0, 0.85)
            ), ros::Time::now(), "base_footprint", "camera_link"
        )
    );
    r.sleep();
  }
}





