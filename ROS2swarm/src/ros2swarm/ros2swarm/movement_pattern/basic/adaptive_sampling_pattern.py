#!/usr/bin/env python3
#    Copyright 2025 [Your Name/Institution]
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.

"""
Adaptive Sampling Pattern for ROS2 Swarm.

Robots perform random walk movement with speeds determined by environmental
complexity zones. Zone information (0=SURVEY, 1=INVESTIGATION, 2=HIGH_RES)
is received from Unity simulation via std_msgs/Int8 topic.
"""

from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
import random
from ros2swarm.movement_pattern.movement_pattern import MovementPattern
from rclpy.qos import qos_profile_sensor_data
from ros2swarm.utils import setup_node


class AdaptiveSamplingPattern(MovementPattern):
    """Adaptive sampling movement pattern with zone-based speed control."""

    def __init__(self):
        """Initialize the adaptive sampling pattern node."""
        super().__init__('adaptive_sampling_pattern')

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('adaptive_sampling_timer_period', 0.1),
                ('new_zone_walk_duration', 2.0),
                ('adaptive_sampling_angular_min', 5.0),
                ('adaptive_sampling_angular_max', 20.0),
                ('adaptive_sampling_rot_interval_min', 3.0),
                ('adaptive_sampling_rot_interval_max', 3.0),
                ('adaptive_sampling_lin_interval_min', 3.0),
                ('adaptive_sampling_lin_interval_max', 8.0),
                ('adaptive_sampling_survey_speed_min', 0.30),
                ('adaptive_sampling_survey_speed_max', 0.30),
                ('adaptive_sampling_investigation_speed_min', 0.20),
                ('adaptive_sampling_investigation_speed_max', 0.20),
                ('adaptive_sampling_high_res_speed_min', 0.10),
                ('adaptive_sampling_high_res_speed_max', 0.10)
            ])

        # Get parameters
        self.omega_min = float(
            self.get_parameter("adaptive_sampling_angular_min")
            .get_parameter_value().double_value
        )
        self.omega_max = float(
            self.get_parameter("adaptive_sampling_angular_max")
            .get_parameter_value().double_value
        )
        self.rot_interval_min = float(
            self.get_parameter("adaptive_sampling_rot_interval_min")
            .get_parameter_value().double_value
        )
        self.rot_interval_max = float(
            self.get_parameter("adaptive_sampling_rot_interval_max")
            .get_parameter_value().double_value
        )
        self.lin_interval_min = float(
            self.get_parameter("adaptive_sampling_lin_interval_min")
            .get_parameter_value().double_value
        )
        self.lin_interval_max = float(
            self.get_parameter("adaptive_sampling_lin_interval_max")
            .get_parameter_value().double_value
        )

        # Speed ranges for each zone
        self.survey_speed_min = float(
            self.get_parameter("adaptive_sampling_survey_speed_min")
            .get_parameter_value().double_value
        )
        self.survey_speed_max = float(
            self.get_parameter("adaptive_sampling_survey_speed_max")
            .get_parameter_value().double_value
        )
        self.investigation_speed_min = float(
            self.get_parameter("adaptive_sampling_investigation_speed_min")
            .get_parameter_value().double_value
        )
        self.investigation_speed_max = float(
            self.get_parameter("adaptive_sampling_investigation_speed_max")
            .get_parameter_value().double_value
        )
        self.high_res_speed_min = float(
            self.get_parameter("adaptive_sampling_high_res_speed_min")
            .get_parameter_value().double_value
        )
        self.high_res_speed_max = float(
            self.get_parameter("adaptive_sampling_high_res_speed_max")
            .get_parameter_value().double_value
        )

        # Movement state
        self.current_zone = 0  # Default to SURVEY (0)
        self.previous_zone = 0
        self.in_forced_walk = False
        self.turn = False
        self.current_msg = Twist()
        self.i = 0

        # Zone subscriber
        self.zone_subscription = self.create_subscription(
            Int8,
            'zone',
            self.zone_callback,
            qos_profile=qos_profile_sensor_data
        )

        # Timers
        self.walk_timer = self.create_timer(
            5.0,
            self.swarm_command_controlled_timer(self.random_walk_callback)
        )
        self.publish_timer = self.create_timer(
            self.get_parameter("adaptive_sampling_timer_period")
            .get_parameter_value().double_value,
            self.swarm_command_controlled_timer(self.timer_callback)
        )

        self.get_logger().info('Adaptive Sampling Pattern initialized')

    def zone_callback(self, msg):
        """
        Update current zone from Unity.

        Args:
            msg (Int8): Zone value (0=SURVEY, 1=INVESTIGATION, 2=HIGH_RES)
        """
        new_zone = msg.data

        # Detect zone transition
        if new_zone != self.previous_zone and not self.in_forced_walk:
            self.get_logger().info(f'Zone transition: {self.previous_zone} to {new_zone}')
            
            # Cancel existing timer
            self.walk_timer.cancel()
            
            # Force walking phase for t seconds at new zone speed
            self.turn = False  # Ensure next callback is walk phase
            self.in_forced_walk = True
            msg_walk = Twist()
            msg_walk.angular.z = 0.0
            msg_walk.linear.x = self.get_speed_for_zone(new_zone)
            self.current_msg = msg_walk
            
            # Schedule walk for t seconds, then resume normal pattern
            self.walk_timer = self.create_timer(
                self.get_parameter("new_zone_walk_duration").get_parameter_value().double_value,
                self.swarm_command_controlled_timer(self.forced_walk_complete)
            )
        
        self.previous_zone = self.current_zone
        self.current_zone = new_zone
        self.get_logger().debug(f'Zone updated to: {self.current_zone}')

    def forced_walk_complete(self):
        """Called after forced walk completes."""
        self.in_forced_walk = False  # CLEAR FLAG
        self.random_walk_callback()  # Resume normal pattern

    def get_speed_for_zone(self, zone):
        """
        Get random speed for current zone.

        Args:
            zone (int): Zone value (0, 1, or 2)

        Returns:
            float: Linear speed in m/s
        """
        if zone == 0:  # SURVEY
            return random.uniform(self.survey_speed_min, self.survey_speed_max)
        elif zone == 1:  # INVESTIGATION
            return random.uniform(
                self.investigation_speed_min,
                self.investigation_speed_max
            )
        elif zone == 2:  # HIGH_RES
            return random.uniform(self.high_res_speed_min, self.high_res_speed_max)
        else:
            # Default to SURVEY if invalid zone
            self.get_logger().warn(f'Invalid zone {zone}, defaulting to SURVEY')
            return random.uniform(self.survey_speed_min, self.survey_speed_max)

    def random_walk_callback(self):
        """Execute random walk behavior with zone-dependent speed."""
        msg = Twist()

        if self.turn:
            # Turn phase: rotate in place
            sign = 1 if random.random() < 0.5 else -1
            msg.angular.z = random.uniform(self.omega_min, self.omega_max) * sign
            # msg.angular.z = self.omega_max * sign
            msg.linear.x = 0.0

            # Schedule next walk callback
            self.walk_timer.cancel()
            duration = random.uniform(self.rot_interval_min, self.rot_interval_max)
            self.walk_timer = self.create_timer(
                duration,
                # 4.0,
                self.swarm_command_controlled_timer(self.random_walk_callback)
            )

        else:
            # Forward phase: move at zone-dependent speed
            msg.angular.z = 0.0
            msg.linear.x = self.get_speed_for_zone(self.current_zone)

            # Schedule next walk callback
            self.walk_timer.cancel()
            duration = random.uniform(self.lin_interval_min, self.lin_interval_max)
            # duration = 3.0
            self.walk_timer = self.create_timer(duration, self.swarm_command_controlled_timer(self.random_walk_callback))

            self.get_logger().debug(
                f'Moving forward at {msg.linear.x:.2f} m/s '
                f'(zone {self.current_zone}) for {duration:.1f}s'
            )

        # Toggle turn state for next callback
        self.turn = not self.turn
        self.current_msg = msg

    def timer_callback(self):
        """Publish current twist message at fixed rate."""
        self.command_publisher.publish(self.current_msg)
        self.get_logger().info(
            f'Publishing {self.i}: linear={self.current_msg.linear.x:.2f}, '
            f'angular={self.current_msg.angular.z:.2f}'
        )
        self.i += 1


def main(args=None):
    """Node entry point."""
    setup_node.init_and_spin(args, AdaptiveSamplingPattern)


if __name__ == '__main__':
    main()