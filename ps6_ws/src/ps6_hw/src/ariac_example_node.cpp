// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// %Tag(FULLTEXT)%
// %Tag(INCLUDE_STATEMENTS)%
#include <algorithm>
#include <vector>

#include <ros/ros.h>

#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/Proximity.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <osrf_gear/ConveyorBeltControl.h> //for conveyor belt movement
#include <osrf_gear/DroneControl.h> //for drone control
#include<osrf_gear/LogicalCameraImage.h> //for camera control

//global variables for camera functionality
bool g_take_snapshot = false;
osrf_gear::LogicalCameraImage g_camera1_data;

void camera2(const osrf_gear::LogicalCameraImage& message_holder) {
  if (g_take_snapshot) {
    ROS_INFO_STREAM("Image from camera: " << message_holder);
    ROS_INFO_STREAM("Image from camera: " << message_holder.models.size());
    g_camera1_data = message_holder;
    ROS_INFO_STREAM("Global camera1 data: " << g_camera1_data.models.size());
    //g_take_snapshot = false;	
  }
}

/// Example class that can hold state and provide methods that handle incoming data.
class MyCompetitionClass
{
public:
  explicit MyCompetitionClass(ros::NodeHandle & node)
  : current_score_(0), has_been_zeroed_(false)
  {
    // %Tag(ADV_CMD)%
    joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>(
      "/ariac/arm/command", 10);
    // %EndTag(ADV_CMD)%
  }

  /// Called when a new message is received.
  void current_score_callback(const std_msgs::Float32::ConstPtr & msg) {
    if (msg->data != current_score_)
    {
      ROS_INFO_STREAM("Score: " << msg->data);
    }
    current_score_ = msg->data;
  }

  /// Called when a new message is received.
  void competition_state_callback(const std_msgs::String::ConstPtr & msg) {
    if (msg->data == "done" && competition_state_ != "done")
    {
      ROS_INFO("Competition ended.");
    }
    competition_state_ = msg->data;
  }

  /// Called when a new Order message is received.
  void order_callback(const osrf_gear::Order::ConstPtr & order_msg) {
    ROS_INFO_STREAM("Received order:\n" << *order_msg);
    received_orders_.push_back(*order_msg);
  }

  // %Tag(CB_CLASS)%
  /// Called when a new JointState message is received.
  void joint_state_callback(
    const sensor_msgs::JointState::ConstPtr & joint_state_msg)
  {
    ROS_INFO_STREAM_THROTTLE(10,
      "Joint States (throttled to 0.1 Hz):\n" << *joint_state_msg);
    // ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
    current_joint_states_ = *joint_state_msg;
    if (!has_been_zeroed_) {
      has_been_zeroed_ = true;
      ROS_INFO("Sending arm to zero joint positions...");
      send_arm_to_zero_state();
    }
  }
  // %EndTag(CB_CLASS)%

  // %Tag(ARM_ZERO)%
  /// Create a JointTrajectory with all positions set to zero, and command the arm.
  void send_arm_to_zero_state() {
    // Create a message to send.
    trajectory_msgs::JointTrajectory msg;

    // Fill the names of the joints to be controlled.
    // Note that the vacuum_gripper_joint is not controllable.
    msg.joint_names.clear();
    msg.joint_names.push_back("iiwa_joint_1");
    msg.joint_names.push_back("iiwa_joint_2");
    msg.joint_names.push_back("iiwa_joint_3");
    msg.joint_names.push_back("iiwa_joint_4");
    msg.joint_names.push_back("iiwa_joint_5");
    msg.joint_names.push_back("iiwa_joint_6");
    msg.joint_names.push_back("iiwa_joint_7");
    msg.joint_names.push_back("linear_arm_actuator_joint");
    // Create one point in the trajectory.
    msg.points.resize(1);
    // Resize the vector to the same length as the joint names.
    // Values are initialized to 0.
    msg.points[0].positions.resize(msg.joint_names.size(), 0.0);
    // How long to take getting to the point (floating point seconds).
    msg.points[0].time_from_start = ros::Duration(0.001);
    ROS_INFO_STREAM("Sending command:\n" << msg);
    joint_trajectory_publisher_.publish(msg);
  }
  // %EndTag(ARM_ZERO)%

  /// Called when a new LogicalCameraImage message is received.
  void logical_camera_callback(
    const osrf_gear::LogicalCameraImage::ConstPtr & image_msg)
  {
    ROS_INFO_STREAM_THROTTLE(10,
      "Logical camera: '" << image_msg->models.size() << "' objects.");
  }

  /// Called when a new Proximity message is received.
  void break_beam_callback(const osrf_gear::Proximity::ConstPtr & msg) {
    if (msg->object_detected) {  // If there is an object in proximity.
      ROS_INFO("Break beam triggered.");
    }
  }

private:
  std::string competition_state_;
  double current_score_;
  ros::Publisher joint_trajectory_publisher_;
  std::vector<osrf_gear::Order> received_orders_;
  sensor_msgs::JointState current_joint_states_;
  bool has_been_zeroed_;
};

void proximity_sensor_callback(const sensor_msgs::Range::ConstPtr & msg) {
  if ((msg->max_range - msg->range) > 0.01) {  // If there is an object in proximity.
    ROS_INFO_THROTTLE(1, "Proximity sensor sees something.");
  }
}

void laser_profiler_callback(const sensor_msgs::LaserScan::ConstPtr & msg) {
  size_t number_of_valid_ranges = std::count_if(
    msg->ranges.begin(), msg->ranges.end(), std::isfinite<float>);
  if (number_of_valid_ranges > 0) {
    ROS_INFO_THROTTLE(1, "Laser profiler sees something.");
  }
}

// %Tag(MAIN)%
int main(int argc, char ** argv) {
  // Last argument is the default name of the node.
  ros::init(argc, argv, "ariac_example_node");

  ros::NodeHandle node;

  // Instance of custom class from above.
  MyCompetitionClass comp_class(node);

  // Subscribe to the '/ariac/current_score' topic.
  ros::Subscriber current_score_subscriber = node.subscribe(
    "/ariac/current_score", 10,
    &MyCompetitionClass::current_score_callback, &comp_class);

  // Subscribe to the '/ariac/competition_state' topic.
  ros::Subscriber competition_state_subscriber = node.subscribe(
    "/ariac/competition_state", 10,
    &MyCompetitionClass::competition_state_callback, &comp_class);

  // %Tag(SUB_CLASS)%
  // Subscribe to the '/ariac/orders' topic.
  ros::Subscriber orders_subscriber = node.subscribe(
    "/ariac/orders", 10,
    &MyCompetitionClass::order_callback, &comp_class);
  // %EndTag(SUB_CLASS)%

  // Subscribe to the '/ariac/joint_states' topic.
  ros::Subscriber joint_state_subscriber = node.subscribe(
    "/ariac/joint_states", 10,
    &MyCompetitionClass::joint_state_callback, &comp_class);

  // %Tag(SUB_FUNC)%
  // Subscribe to the '/ariac/proximity_sensor_1' topic.
  ros::Subscriber proximity_sensor_subscriber = node.subscribe(
    "/ariac/proximity_sensor_1", 10, proximity_sensor_callback);
  // %EndTag(SUB_FUNC)%

  // Subscribe to the '/ariac/break_beam_1_change' topic.
  ros::Subscriber break_beam_subscriber = node.subscribe(
    "/ariac/break_beam_1_change", 10,
    &MyCompetitionClass::break_beam_callback, &comp_class);

  // Subscribe to the '/ariac/logical_camera_1' topic.
  ros::Subscriber logical_camera_subscriber = node.subscribe(
    "/ariac/logical_camera_1", 10,
    &MyCompetitionClass::logical_camera_callback, &comp_class);

  // Subscribe to the '/ariac/laser_profiler_1' topic.
  ros::Subscriber laser_profiler_subscriber = node.subscribe(
    "/ariac/laser_profiler_1", 10, laser_profiler_callback);

  // Subscribe to 'ariac/logical_camera_2' topic
  ros::Subscriber camera2_subscriber = node.subscribe("ariac/logical_camera_2", 1, camera2);
  

  ROS_INFO("Setup complete.");


  //Start the competition!

  // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
  ros::ServiceClient start_client =
    node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  // If it's not already ready, wait for it to be ready.
  // Calling the Service using the client before the server is ready would fail.
  if (!start_client.exists()) {
    ROS_INFO("Waiting for the competition to be ready...");
    start_client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }
  ROS_INFO("Requesting competition start...");
  std_srvs::Trigger srv;  // Combination of the "request" and the "response".
  start_client.call(srv);  // Call the start Service.
  if (!srv.response.success) {  // If not successful, print out why.
    ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
  } else {
    ROS_INFO("Competition started!");
  }

  //Start the conveyor belt!
  ros::ServiceClient convey_client =
    node.serviceClient<osrf_gear::ConveyorBeltControl>("/ariac/conveyor/control");
  osrf_gear::ConveyorBeltControl convey_srv; 
  convey_srv.request.power = 100.0; //100 means conveyor belt is running, 0.0 is stopped.
  convey_srv.response.success = false; //Initially, the conveyor belt is stopped.
  while (!convey_srv.response.success) {
    ROS_WARN("Waiting to start conveyor belt...");
    convey_client.call(convey_srv);
    ros::Duration(0.5).sleep();
  }
  ROS_INFO("Started conveyor belt!");
  
  //Continue running the conveyor belt until a box is detected under the camera...
  bool is_box = false; //When a box has been found
  bool is_below_camera = false; //When the box is underneath the camera area
  while (!is_box) {
    g_take_snapshot = true;
    while (g_camera1_data.models.size() < 1) {
      ros::spinOnce();
      ros::Duration(0.5).sleep();
    }
    ROS_INFO("Box found!");
    is_box = true;
    while (!is_below_camera) {
      ros::spinOnce();
      ros::Duration(0.5).sleep();
      if (g_camera1_data.models[0].pose.position.z < 0.1 && g_camera1_data.models[0].pose.position.z > -0.1) {
        ROS_INFO("Box is below camera");
        is_below_camera = true;
        convey_srv.request.power = 0.0; //Stop the conveyor belt
        convey_srv.response.success = false;
        while (!convey_srv.response.success) {
	  ROS_WARN("Failed to stop conveyor belt...");
	  convey_client.call(convey_srv);
	  ros::Duration(0.5).sleep();
	}
	ros::Duration(5.0).sleep(); //Stay under camera for 5 seconds
	convey_srv.request.power = 100.0; //Restart conveyor belt
	convey_srv.response.success = false;
	while (!convey_srv.response.success) {
	  ROS_WARN("Failed to start conveyor belt...");
	  convey_client.call(convey_srv);
	  ros::Duration(0.5).sleep();
	}
     }
   }
   g_take_snapshot = false;
 }

  //Send a drone to pick up the box!
  ros::ServiceClient drone_client =
    node.serviceClient<osrf_gear::DroneControl>("/ariac/drone");
  
  osrf_gear::DroneControl drone_srv; 
  drone_srv.request.shipment_type = "blah"; 
  drone_srv.response.success = false;
  while (!drone_srv.response.success) {
    ROS_WARN("Could not send drone...");
    drone_client.call(drone_srv);
    ros::Duration(0.5).sleep();
  }
  ROS_INFO("Sent drone!");

  //ros::spin();  // This executes callbacks on new data until ctrl-c.

  return 0;
}
// %EndTag(MAIN)%
// %EndTag(FULLTEXT)%

