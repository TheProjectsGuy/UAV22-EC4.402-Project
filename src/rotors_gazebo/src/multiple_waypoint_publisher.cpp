/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <fstream>
#include <iostream>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>


//  /pelican/odometry_sensor1/odometry

int main(int argc, char** argv) {
  ros::init(argc, argv, "multi_waypoint_publisher");
  ros::NodeHandle nh;
  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

  ROS_INFO("Started waypoint_publisher.");

  double delay;

  const float DEG_2_RAD = M_PI / 180.0;

  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();

  double x = 5.0;
  double y = 5.0;
  double z = 5.0;

  std::vector<std::vector<double>> coordinates;
  
  coordinates.push_back({1.0, 1.0, 1.0});
  //coordinates.push_back({2.0, 2.0, 2.0});
  //coordinates.push_back({3.0, 3.0, 3.0});
  //coordinates.push_back({4.0, 4.0, 4.0});
  //coordinates.push_back({5.0, 5.0, 5.0});

  for(std::vector<double> coor : coordinates)
  {
    Eigen::Vector3d desired_position(coor[0], coor[1], coor[2]);

    double desired_yaw = 0.0 * DEG_2_RAD;

    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
        desired_yaw, &trajectory_msg);

    // Wait for some time to create the ros publisher.
    ros::Duration(delay).sleep();

    while (trajectory_pub.getNumSubscribers() == 0 && ros::ok()) {
        ROS_INFO("There is no subscriber available, trying again in 1 second.");
        ros::Duration(1.0).sleep();
    }

    ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
            nh.getNamespace().c_str(),
            desired_position.x(),
            desired_position.y(),
            desired_position.z());

    trajectory_pub.publish(trajectory_msg);

    ros::spinOnce();
  }
  ros::shutdown();

  return 0;
}
