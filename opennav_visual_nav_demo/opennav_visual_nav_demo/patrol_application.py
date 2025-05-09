#! /usr/bin/env python3
# Copyright 2025 Open Naviation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from threading import Lock, Thread
import time
import random

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import BatteryState, Joy
from visualization_msgs.msg import MarkerArray

###################################################################################################
# For this demo, we will use a route graph of key locations in a space to navigate
# using a new (May 2025) feature, Nav2 Route Server. This feature allows us to define a set of
# locations in a space and then use a route server to navigate between them. This is useful for
# applications where we want to define a set of locations in a space and then use a route server
# to navigate between them rather than freespace planning. Find more info at: 
# https://docs.nav2.org/configuration/packages/configuring-route-server.html
# For example:
# * A graph representing a logistics facility and each node is an aisle terminus, shelf location,
#   or key location. Robot routes through a very large space in real-time using the defined lanes.
# * A graph representing predefined / taught routes the robot is allows to take through a space
#   to avoid navigating in restricted areas and/or through deterministic paths.

# The demonstration will use cuVSLAM, cuVGL for mapping and localization and NvBlox for collision
# avoidance. This autonomy script will call the navigation system to navigate to a new location
# in the route graph repeatedly until the robot is requested to stop or the battery level is low.
# The robot will then navigate back to its dock to charge to be repeated once sufficiently charged.

# This can be reproduced by:
# 1. Go to the location and joystick robot to generate the map using cuVSLAM
# 2. Use the route server tools to create a graph of the locations in the space
#    https://docs.nav2.org/tutorials/docs/route_server_tools.html
# 3. Update this script's configuration for the route graph to use
# 4. Update the dock node ID or location below
# 4. Run, starting at dock charging for closed-loop autonomy
###################################################################################################

"""
A Long-Duration, Vision-Based Navigation Patroling demo using:
* NVIDIA Jetson AGX Orin
* Isaac Perceptor
* Isaac ROS
* ROS 2, Nav2
"""

class GraphNodeLocation:
    def __init__(self, id, x, y):
        self.id = id
        self.x = x
        self.y = y

    def __repr__(self):
        return f"GraphNodeLocation(id={self.id}, x={self.x}, y={self.y})"


class VisualNavigationPatrol(Node):

    def __init__(self):
        super().__init__('vision_navigation_patrol')
        self.demo_thread = None
        self.lock = Lock()
        self.stop = False
        self.graph_nodes = []

        self.navigator = BasicNavigator()

        self.waitUntilActive()
        self.getParameters()

        self.docking_client = ActionClient(self.navigator, DockRobot, 'dock_robot')
        self.undocking_client = ActionClient(self.navigator, UndockRobot, 'undock_robot')

        self.batt_qos = QoSProfile(
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        self.joy_sub = self.create_subscription(
            Joy, 'joy_teleop/joy', self.joyCallback, 10)  # TODO topic
        self.batt_sub = self.create_subscription(
            BatteryState, 'platform/bms/state', self.batteryCallback, self.batt_qos)  # TODO topic
        self.graph_sub = self.create_subscription(
            MarkerArray, 'route_server/route_graph', self.graphCallback, 10)  # TODO topic

        # Could be used to run mission every N minutes, for example once an hour
        # self.timer = self.create_timer(3600.0, self.runDemo)
        print('Navisual navigation patrol node started.')

    def waitUntilActive(self):
        """Block until the components of the navigation system are up and running."""
        self.navigator._waitForNodeToActivate('bt_navigator')
        print('Nav2 is ready for use!')

    def getParameters(self):
        # Get demo buttons (square=3, X=0 on PS4)
        self.declare_parameter('start_button', 2)  # TODO button
        self.declare_parameter('exit_button', 0)  # TODO button
        self.start_button = self.get_parameter('start_button').value
        self.exit_button = self.get_parameter('exit_button').value

        # Get minimum battery to exit demo and return to base
        self.declare_parameter('min_battery_lvl', 0.20)
        self.min_battery_lvl = self.get_parameter('min_battery_lvl').value

        # Get battery level to restart taking missions once recharged automatically
        self.declare_parameter('battery_mission_restart_minimum', 0.80)
        self.battery_mission_restart_minimum = \
            self.get_parameter('battery_mission_restart_minimum').value

    def dockRobot(self, dock_id, nav_to_dock=True):
        while not self.docking_client.wait_for_server(timeout_sec=1.0):
            print('"DockRobot" action server not available, waiting...')
        goal_msg = DockRobot.Goal()
        goal_msg.use_dock_id = True
        goal_msg.dock_id = dock_id
        goal_msg.navigate_to_staging_pose = nav_to_dock
        print('Docking at dock ID: ' + str(dock_id) + '...')
        send_goal_future = self.docking_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.navigator, send_goal_future)
        goal_handle = send_goal_future.result()
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.navigator, result_future)
        status = result_future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            print(f'Docking with failed with status code: {status}')
        else:
            print('Docking successful!')

    def undockRobot(self, dock_type=''):
        while not self.undocking_client.wait_for_server(timeout_sec=1.0):
            print('"UndockRobot" action server not available, waiting...')
        goal_msg = UndockRobot.Goal()
        goal_msg.dock_type = dock_type
        print('Undocking from dock of type: ' + str(dock_type) + '...')
        send_goal_future = self.undocking_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.navigator, send_goal_future)
        goal_handle = send_goal_future.result()
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.navigator, result_future)
        status = result_future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            print(f'Undocking with failed with status code: {status}')
        else:
            print('Undocking successful!')

    def batteryCallback(self, msg):
        if msg.percentage < self.min_battery_lvl:
            with self.lock:
                self.stop = True

        # Could be used to continue mission once battery levels are sufficiently high automatically
        # with self.lock:
        #     if msg.percentage > self.battery_mission_restart_minimum and self.demo_thread is None:
        #         print('Battery recharge sufficient to continue, starting patrol mission!')
        #         self.stop = False
        #         self.demo_thread = Thread(target=self.runDemo)
        #         self.demo_thread.daemon = True
        #         self.demo_thread.start()

    def joyCallback(self, msg):
        if msg.buttons[self.exit_button] == 1:
            print('Stop request detected, stopping patrolling at the end of this mission!')
            with self.lock:
                self.stop = True

        if msg.buttons[self.start_button] == 1 and self.demo_thread is None:
            print('Start request detected, starting a patrolling mission!')
            # Could be triggered by external cloud system to dispatch a new mission
            # A joystick for our demonstration purposes
            self.demo_thread = Thread(target=self.runDemo)
            self.demo_thread.daemon = True
            self.demo_thread.start()

    def graphCallback(self, msg):
        self.graph_nodes.clear()
        for marker in msg.markers:
            # If we're adding a marker and its a node's ID text
            if marker.action == 0 and marker.type == 9:
                self.graph_nodes.append(
                    GraphNodeLocation(
                        int(marker.text),
                        marker.pose.position.x,
                        marker.pose.position.y - 0.07))  # offset for text height in msg

    def runDemo(self):
        stop = False
        nav_start = self.navigator.get_clock().now()
        # Not required, cuVGL will set the initial pose on startup
        # init_pose = PoseStamped()
        # init_pose.header.frame_id = 'map'
        # init_pose.header.stamp = nav_start.to_msg()
        # init_pose.pose.position.x = dock_pose[0]
        # init_pose.pose.position.y = dock_pose[1]
        # init_pose.pose.orientation = self.getQuad(dock_pose[2])
        # self.navigator.setInitialPose(init_pose)
        self.navigator.clearAllCostmaps()
        self.undockRobot(dock_type='charging_dock')

        while rclpy.ok() and stop is False:
            if len(self.graph_nodes) == 0:
                print('No route graph nodes available, please create or publish route graph first.')
                break
            
            # Find our next location to patrol to, could be replaced with a formal
            # policy to select the next location based on a set of application rules
            goal = random.choice(self.graph_nodes)
            print(f'Navigating to the next patrol location {goal.id} ({goal.x}, {goal.y})')

            # Go to next patrol location on the graph
            # self.navigator.goToPose() #TODO use the IDX. Do I need a navigator? Fuck.
            while not self.navigator.isTaskComplete() or not rclpy.ok():
                #########################################################################
                # PLACE ANY APPLICATION FEEDBACK PROCESSING CODE HERE
                pass
                #########################################################################

            #############################################################################
            # PLACE ANY APPLICATION SPECIFIC CODE HERE AT EACH LOCATION OR ERROR HANDLING
            # Wait at picking location for user to place item (simulated with 3s stop)
            time.sleep(3)
            #############################################################################

            with self.lock:
                stop = self.stop

        # Completed task, dock robot to charge
        self.dockRobot(dock_id='home_dock', nav_to_dock=True)
        print('Patrolling mission completed and robot is docked to charge.'
              ' Waiting for next mission...')
        self.stop = False
        self.demo_thread = None


def main():
    rclpy.init()
    node = VisualNavigationPatrol()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    exit(0)


if __name__ == '__main__':
    main()
