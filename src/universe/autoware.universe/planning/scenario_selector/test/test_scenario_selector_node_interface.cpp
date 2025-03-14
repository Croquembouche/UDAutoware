// Copyright 2023 TIER IV, Inc.
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

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "planning_interface_test_manager/planning_interface_test_manager.hpp"
#include "planning_interface_test_manager/planning_interface_test_manager_utils.hpp"
#include "scenario_selector/scenario_selector_node.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

TEST(PlanningModuleInterfaceTest, NodeTestWithExceptionTrajectoryLaneDrivingMode)
{
  rclcpp::init(0, nullptr);

  auto test_manager = std::make_shared<planning_test_utils::PlanningInterfaceTestManager>();

  auto node_options = rclcpp::NodeOptions{};
  node_options.append_parameter_override("update_rate", 10.0);
  node_options.append_parameter_override("th_max_message_delay_sec", INFINITY);
  node_options.append_parameter_override("th_arrived_distance_m", 1.0);
  node_options.append_parameter_override("th_stopped_time_sec", 1.0);
  node_options.append_parameter_override("th_stopped_velocity_mps", 0.01);
  auto test_target_node = std::make_shared<ScenarioSelectorNode>(node_options);

  // publish necessary topics from test_manager
  test_manager->publishOdometry(test_target_node, "input/odometry");
  test_manager->publishAcceleration(test_target_node, "input/acceleration");
  test_manager->publishParkingState(test_target_node, "is_parking_completed");
  test_manager->publishTrajectory(test_target_node, "input/parking/trajectory");
  test_manager->publishMap(test_target_node, "input/lanelet_map");
  test_manager->publishRoute(test_target_node, "input/route");

  // set subscriber with topic name: scenario_selector → test_node_
  test_manager->setScenarioSubscriber("output/scenario");

  // set scenario_selector's input topic name(this topic is changed to test node)
  test_manager->setTrajectoryInputTopicName("input/lane_driving/trajectory");

  // test for normal trajectory
  ASSERT_NO_THROW(test_manager->testWithNominalTrajectory(test_target_node));
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // test for trajectory with empty/one point/overlapping point
  test_manager->testWithAbnormalTrajectory(test_target_node);
  rclcpp::shutdown();
}

TEST(PlanningModuleInterfaceTest, NodeTestWithExceptionTrajectoryParkingMode)
{
  rclcpp::init(0, nullptr);

  auto test_manager = std::make_shared<planning_test_utils::PlanningInterfaceTestManager>();

  auto node_options = rclcpp::NodeOptions{};
  node_options.append_parameter_override("update_rate", 10.0);
  node_options.append_parameter_override("th_max_message_delay_sec", INFINITY);
  node_options.append_parameter_override("th_arrived_distance_m", 1.0);
  node_options.append_parameter_override("th_stopped_time_sec", 1.0);
  node_options.append_parameter_override("th_stopped_velocity_mps", 0.01);
  auto test_target_node = std::make_shared<ScenarioSelectorNode>(node_options);

  // publish necessary topics from test_manager
  test_manager->publishOdometry(test_target_node, "input/odometry");
  test_manager->publishAcceleration(test_target_node, "input/acceleration");
  test_manager->publishParkingState(test_target_node, "is_parking_completed");
  test_manager->publishTrajectory(test_target_node, "input/lane_driving/trajectory");
  test_manager->publishMap(test_target_node, "input/lanelet_map");
  test_manager->publishRoute(test_target_node, "input/route");

  // set subscriber with topic name: scenario_selector → test_node_
  test_manager->setScenarioSubscriber("output/scenario");

  // set scenario_selector's input topic name(this topic is changed to test node)
  test_manager->setTrajectoryInputTopicName("input/parking/trajectory");

  // test for normal trajectory
  ASSERT_NO_THROW(test_manager->testWithNominalTrajectory(test_target_node));

  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // test for trajectory with empty/one point/overlapping point
  test_manager->testWithAbnormalTrajectory(test_target_node);
  rclcpp::shutdown();
}
