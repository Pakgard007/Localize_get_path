#!/usr/bin/env python3
"""Play a velocity profile from CSV out to /cmd_vel.

By default the profile repeats from the beginning every time it
reaches the end, and keeps going until Ctrl+C. Use --no-loop for a
single pass, or --loop-count N for a fixed number of rounds.
"""

import os
import time
import argparse

import pandas as pd

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist


class CsvCmdVelPlayer(Node):

    def __init__(
        self,
        csv_file,
        topic="/cmd_vel",
        speed_scale=1.0,
        loop=True,
        loop_count=0,
        loop_delay=0.0
    ):
        super().__init__("csv_cmdvel_player")

        self.csv_file = csv_file
        self.topic = topic
        self.speed_scale = speed_scale

        # loop_count = 0 means repeat forever until Ctrl+C
        self.loop = loop
        self.loop_count = max(0, int(loop_count))
        self.loop_delay = max(0.0, float(loop_delay))

        # ==================================================
        # Check CSV
        # ==================================================
        if not os.path.isfile(self.csv_file):
            self.get_logger().error(
                f"CSV file not found:\n{self.csv_file}"
            )
            raise FileNotFoundError(self.csv_file)

        self.get_logger().info(
            f"Loading CSV:\n{self.csv_file}"
        )

        # ==================================================
        # Read CSV
        # ==================================================
        self.df = pd.read_csv(self.csv_file)

        self.get_logger().info(
            f"CSV columns: {list(self.df.columns)}"
        )

        # ==================================================
        # Detect time column
        # ==================================================
        if "t_s" in self.df.columns:
            self.time_column = "t_s"

        elif "ts" in self.df.columns:
            self.time_column = "ts"

        else:
            raise ValueError(
                "Time column not found.\n"
                "Required: t_s or ts"
            )

        # ==================================================
        # Detect velocity column
        # ==================================================
        if "v_target" in self.df.columns:
            self.velocity_column = "v_target"

        elif "vtaget" in self.df.columns:
            self.velocity_column = "vtaget"

        else:
            raise ValueError(
                "Velocity column not found.\n"
                "Required: v_target or vtaget"
            )

        # ==================================================
        # Select required columns
        # ==================================================
        self.df = self.df[
            [
                self.time_column,
                self.velocity_column
            ]
        ].copy()

        # ==================================================
        # Convert to numeric
        # ==================================================
        self.df[self.time_column] = pd.to_numeric(
            self.df[self.time_column],
            errors="coerce"
        )

        self.df[self.velocity_column] = pd.to_numeric(
            self.df[self.velocity_column],
            errors="coerce"
        )

        # ==================================================
        # Remove invalid data
        # ==================================================
        self.df = self.df.dropna()

        self.df = self.df[
            self.df[self.time_column] >= 0
        ]

        # ==================================================
        # Sort by timestamp
        # ==================================================
        self.df = self.df.sort_values(
            by=self.time_column
        ).reset_index(drop=True)

        if len(self.df) == 0:
            raise ValueError(
                "No valid data found in CSV."
            )

        # ==================================================
        # Convert to arrays
        # ==================================================
        self.times = self.df[
            self.time_column
        ].to_numpy()

        self.velocities = self.df[
            self.velocity_column
        ].to_numpy()

        # ==================================================
        # Print information
        # ==================================================
        self.get_logger().info(
            "=========================================="
        )

        self.get_logger().info(
            f"Time column     : {self.time_column}"
        )

        self.get_logger().info(
            f"Velocity column : {self.velocity_column}"
        )

        self.get_logger().info(
            f"Number of rows  : {len(self.df)}"
        )

        self.get_logger().info(
            f"Start time      : {self.times[0]:.3f} s"
        )

        self.get_logger().info(
            f"End time        : {self.times[-1]:.3f} s"
        )

        self.get_logger().info(
            f"Max velocity    : "
            f"{self.velocities.max():.3f} m/s"
        )

        self.get_logger().info(
            f"Speed scale     : {self.speed_scale:.3f}"
        )

        self.get_logger().info(
            f"Publish topic   : {self.topic}"
        )

        if self.loop:

            rounds = (
                "infinite (until Ctrl+C)"
                if self.loop_count == 0
                else f"{self.loop_count}"
            )

            self.get_logger().info(
                f"Loop            : {rounds}"
            )

            self.get_logger().info(
                f"Loop delay      : {self.loop_delay:.3f} s"
            )

        else:

            self.get_logger().info(
                "Loop            : off (play once)"
            )

        self.get_logger().info(
            "=========================================="
        )

        # ==================================================
        # ROS Publisher
        # ==================================================
        self.publisher = self.create_publisher(
            Twist,
            self.topic,
            10
        )

        # ==================================================
        # Playback variables
        # ==================================================
        self.index = 0

        self.start_time = None

        self.started = False

        self.finished = False

        self.round_number = 1

        # ==================================================
        # Cycle length used when looping
        # span of the CSV + one average sample step, so the
        # last row still gets its own slice of time before
        # the profile restarts from the first row
        # ==================================================
        span = float(
            self.times[-1] - self.times[0]
        )

        if len(self.times) > 1:

            step = span / (len(self.times) - 1)

        else:

            step = 0.0

        self.cycle_duration = (
            span
            + step
            + self.loop_delay
        )

        # ==================================================
        # Timer
        # Check every 5 ms
        # ==================================================
        self.timer = self.create_timer(
            0.005,
            self.timer_callback
        )

        # ==================================================
        # Start delay
        # 2 seconds
        # ==================================================
        self.start_timer = self.create_timer(
            2.0,
            self.start_playback
        )

        self.get_logger().info(
            "Waiting 2 seconds before playback..."
        )

    # ======================================================
    # Start playback
    # ======================================================

    def start_playback(self):

        if self.started:
            return

        self.started = True

        self.start_time = time.monotonic()

        self.start_timer.cancel()

        self.get_logger().info(
            "=========================================="
        )

        self.get_logger().info(
            "START CMD_VEL PLAYBACK"
        )

        self.get_logger().info(
            "=========================================="
        )

    # ======================================================
    # Main playback
    # ======================================================

    def timer_callback(self):

        # --------------------------------------------------
        # Before start
        # --------------------------------------------------
        if not self.started:

            self.publish_velocity(0.0)

            return

        # --------------------------------------------------
        # Finished
        # --------------------------------------------------
        if self.finished:

            self.publish_velocity(0.0)

            return

        # --------------------------------------------------
        # Calculate elapsed time
        # --------------------------------------------------
        elapsed = (
            time.monotonic()
            - self.start_time
        )

        # --------------------------------------------------
        # Requested pause between rounds: command 0 to keep the
        # cart at rest instead of holding the last value of the
        # previous round.
        # Only when loop_delay > 0 - with no delay the rounds run
        # back to back and the last value simply holds until the
        # next round's first row is due, so no 0 is injected.
        # --------------------------------------------------
        if (
            self.loop_delay > 0.0
            and self.index == 0
            and elapsed < self.times[0]
        ):

            self.publish_velocity(0.0)

            return

        # --------------------------------------------------
        # Send CSV values according to time
        # --------------------------------------------------
        while (
            self.index < len(self.times)
            and elapsed >= self.times[self.index]
        ):

            velocity = (
                self.velocities[self.index]
                * self.speed_scale
            )

            self.publish_velocity(
                velocity
            )

            self.get_logger().info(
                f"[CMD_VEL] "
                f"elapsed={elapsed:8.3f} s | "
                f"csv_t={self.times[self.index]:8.3f} s | "
                f"v={velocity:6.3f} m/s"
            )

            self.index += 1

        # --------------------------------------------------
        # End of CSV
        # --------------------------------------------------
        if self.index >= len(self.times):

            keep_going = (
                self.loop
                and (
                    self.loop_count == 0
                    or self.round_number < self.loop_count
                )
            )

            if keep_going:

                # ------------------------------------------
                # Restart the profile.
                # start_time is pushed forward by exactly one
                # cycle instead of being reset to "now", so a
                # slow callback does not make the loop drift.
                # ------------------------------------------
                self.index = 0

                self.start_time += self.cycle_duration

                self.round_number += 1

                self.get_logger().info(
                    "=========================================="
                )

                self.get_logger().info(
                    f"LOOP RESTART: round {self.round_number}"
                )

                self.get_logger().info(
                    "=========================================="
                )

                return

            self.publish_velocity(0.0)

            self.finished = True

            self.timer.cancel()

            self.get_logger().info(
                "=========================================="
            )

            self.get_logger().info(
                "CSV PLAYBACK FINISHED"
            )

            self.get_logger().info(
                "CMD_VEL STOPPED"
            )

            self.get_logger().info(
                "=========================================="
            )

    # ======================================================
    # Publish Twist
    # ======================================================

    def publish_velocity(self, velocity):

        msg = Twist()

        # Linear velocity
        msg.linear.x = float(velocity)

        msg.linear.y = 0.0
        msg.linear.z = 0.0

        # Angular velocity
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        self.publisher.publish(msg)

    # ======================================================
    # Safety Stop
    # ======================================================

    def safety_stop(self):

        # โดน SIGTERM context จะถูกปิดไปก่อน publish ตอนนั้นจะ raise
        # แล้วกลบสาเหตุจริงที่ทำให้โปรแกรมจบ
        if not self.context.ok():
            return

        msg = Twist()

        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        self.publisher.publish(msg)

        self.get_logger().info(
            "SAFETY STOP: cmd_vel = 0.0"
        )


# ==========================================================
# MAIN
# ==========================================================

def main(args=None):

    # ------------------------------------------------------
    # Script directory
    # ------------------------------------------------------
    script_dir = os.path.dirname(
        os.path.abspath(__file__)
    )

    # ------------------------------------------------------
    # Default CSV
    # Same directory as this Python file
    # ------------------------------------------------------
    default_csv = os.path.join(
        script_dir,
        "loop1_qp_comfort.csv"
    )

    # ------------------------------------------------------
    # Arguments
    # ------------------------------------------------------
    parser = argparse.ArgumentParser(
        description=(
            "Play v_target from CSV according to t_s "
            "and publish /cmd_vel."
        )
    )

    parser.add_argument(
        "--csv",
        default=default_csv,
        help=(
            "CSV file path "
            "(default: loop1_qp_comfort.csv "
            "in script directory)"
        )
    )

    parser.add_argument(
        "--topic",
        default="/cmd_vel",
        help="ROS2 cmd_vel topic"
    )

    parser.add_argument(
        "--speed-scale",
        type=float,
        default=1.0,
        help="Velocity multiplier"
    )

    parser.add_argument(
        "--no-loop",
        dest="loop",
        action="store_false",
        help=(
            "Play the CSV once and stop "
            "(default: repeat until Ctrl+C)"
        )
    )

    parser.add_argument(
        "--loop-count",
        type=int,
        default=0,
        help=(
            "Number of rounds to play "
            "(0 = forever, default: 0)"
        )
    )

    parser.add_argument(
        "--loop-delay",
        type=float,
        default=0.0,
        help=(
            "Seconds at cmd_vel = 0 between rounds "
            "(default: 0.0)"
        )
    )

    parser.set_defaults(loop=True)

    parsed_args = parser.parse_args()

    # ------------------------------------------------------
    # ROS init
    # ------------------------------------------------------
    rclpy.init(args=args)

    node = None

    try:

        node = CsvCmdVelPlayer(
            csv_file=parsed_args.csv,
            topic=parsed_args.topic,
            speed_scale=parsed_args.speed_scale,
            loop=parsed_args.loop,
            loop_count=parsed_args.loop_count,
            loop_delay=parsed_args.loop_delay
        )

        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):

        print("\nStop requested.")

    except Exception as e:

        # SIGTERM: rclpy ปิด context ระหว่าง spin แล้วโยน RCLError ออกมา
        # ไม่ใช่ error จริง จึงไม่ต้องรายงาน
        if rclpy.ok():

            print(
                f"\nERROR: {e}"
            )

    finally:

        # --------------------------------------------------
        # Safety stop
        # --------------------------------------------------
        if node is not None:

            node.safety_stop()

            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":

    main()