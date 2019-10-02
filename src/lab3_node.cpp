// Taken, adopted, and modified from https://bitbucket.org/osrf/ariac/raw/ariac_2017/ariac_example/src/ariac_example_node.cpp

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
// %EndTag(INCLUDE_STATEMENTS)%

// MoveIt header files
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
// Transformation header files
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"

// Declare the transformation buffer to maintain a list of transformations
tf2_ros::Buffer tfBuffer;
// Instantiate a listener that listens to the tf and tf_static topics and to update the buffer.
tf2_ros::TransformListener tfListener(tfBuffer);

// %Tag(START_COMP)%
/// Start the competition by waiting for and then calling the start ROS Service.
void start_competition(ros::NodeHandle & node) {
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
}
// %EndTag(START_COMP)%

/// Example class that can hold state and provide methods that handle incoming data.
class MyCompetitionClass
{
    public:
        moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    
      explicit MyCompetitionClass(ros::NodeHandle & node)
      : has_been_zeroed_(false)
      {
        // %Tag(ADV_CMD)%
        joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>(
          "/ariac/arm/command", 10);
        // %EndTag(ADV_CMD)%
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
        msg.joint_names.push_back("elbow_joint");
        msg.joint_names.push_back("linear_arm_actuator_joint");
        msg.joint_names.push_back("shoulder_lift_joint");
        msg.joint_names.push_back("shoulder_pan_joint");
        msg.joint_names.push_back("wrist_1_joint");
        msg.joint_names.push_back("wrist_2_joint");
        msg.joint_names.push_back("wrist_3_joint");
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
        latest_image_ = image_msg;
      }

      void process_orders() {
        int order_len = received_orders_.size();
        std::set<int> used_indices;
        std::vector<int> completed_orders;
        
        geometry_msgs::TransformStamped tfStamped;
        try {
            tfStamped = tfBuffer.lookupTransform(move_group.getPlanningFrame().c_str(), latest_image_->pose.c_str(), ros::Time(0.0), ros::Duration(1.0));
            ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), tfStamped.child_frame_id.c_str());
        } 
        catch (tf2::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
        }
        
        //for each vector in received_orders_
        for (int order = 0; order < order_len; order++) {
            osrf_gear::Order current_order = received_orders_[order];
            
            //find matching part from latest_image_ -- if/else statement
            int kit_len = current_order.kits.size();
            
            //loop through each kit in the current order
            for (int kit = 0; kit < kit_len; kit++) {
                int obj_len = current_order.kits[kit].objects.size();
                ROS_INFO_STREAM("Found " << obj_len << " objects in kit " << kit);
                
                //loop through each object in each kit
                for (int obj = 0; obj < obj_len; obj++) {
                    osrf_gear::KitObject current_obj = current_order.kits[kit].objects[obj];
                    
                    ROS_INFO_STREAM("Finding matching object for " << current_obj.type << "...");
                    
                    //each obj in the kit, find a matching object on the tray
                    int model_len = latest_image_->models.size();
                    for (int model = 0; model < model_len; model++) {
                        osrf_gear::Model current_model = latest_image_->models[model];
                        
                        //if the object on the tray is of the right type, move arm to that position
                        //also remove that object on the tray from consideration for now
                        //if object on tray has already been used, do not pull that object
                        if (current_model.type.compare(current_obj.type) == 0 && used_indices.find(model) == used_indices.end()) {
                            ROS_INFO_STREAM("Grabbing " << current_model.type << " at pose " << current_model.pose);
                            used_indices.insert(model);
                            //TODO:  move arm to position
                            //transform relative position of part to world position of part -- in assignment
                            // Retrieve the transformation

                            // tf2_ros::Buffer.lookupTransform("to_frame", "from_frame", "how_recent", "how_long_to_wait");
                            //move arm to world position
                        }
                    }
                }
            }
        }
        
        completed_orders.clear();
      }

    private:
      std::string competition_state_;
      ros::Publisher joint_trajectory_publisher_;
      std::vector<osrf_gear::Order> received_orders_;
      sensor_msgs::JointState current_joint_states_;
      bool has_been_zeroed_;
      osrf_gear::LogicalCameraImage::ConstPtr latest_image_;
      
      
};

void proximity_sensor_callback(const sensor_msgs::Range::ConstPtr & msg) {
  if ((msg->max_range - msg->range) > 0.01) {  // If there is an object in proximity.
    ROS_INFO_THROTTLE(1, "Proximity sensor sees something.");
  }
}

// %Tag(MAIN)%
int main(int argc, char ** argv) {
  // Last argument is the default name of the node.
  ros::init(argc, argv, "ariac_example_node");

  ros::NodeHandle node;

  // Instance of custom class from above.
  MyCompetitionClass comp_class(node);

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

  // Subscribe to the '/ariac/logical_camera_1' topic.
  ros::Subscriber logical_camera_subscriber = node.subscribe(
    "/ariac/logical_camera", 10,
    &MyCompetitionClass::logical_camera_callback, &comp_class);

  ros::Rate loop_rate(10);
  ROS_INFO("Setup complete.");
  start_competition(node);

    while (ros::ok())
    {
        ros::spinOnce();
        //handle all orders for MyCompetitionClass
        comp_class.process_orders();
        loop_rate.sleep();
    }

  return 0;
}
// %EndTag(MAIN)%
// %EndTag(FULLTEXT)%

