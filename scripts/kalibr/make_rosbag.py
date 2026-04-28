#!/usr/bin/env python3
import argparse
import csv
import os

import cv2
import rosbag
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Imu


def stamp_from_us(value):
    micros = int(value)
    return rospy.Time(secs=micros // 1000000, nsecs=(micros % 1000000) * 1000)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--bag", required=True)
    # W4: intrinsic-only datasets carry no IMU stream and no
    # trigger-matched timestamps. With --no-imu we keep every recorded
    # frame (matched or not) and skip the IMU topic entirely.
    parser.add_argument("--no-imu", action="store_true")
    args = parser.parse_args()

    bridge = CvBridge()
    with rosbag.Bag(args.bag, "w") as bag:
        with open(os.path.join(args.dataset, "frames.csv"), newline="") as frames_file:
            for row in csv.DictReader(frames_file):
                if not args.no_imu and row.get("matched") != "1":
                    continue
                image_path = os.path.join(args.dataset, row["image_path"])
                image = cv2.imread(image_path, cv2.IMREAD_COLOR)
                if image is None:
                    raise RuntimeError(f"failed to read image: {image_path}")
                msg = bridge.cv2_to_imgmsg(image, encoding="bgr8")
                # Without trigger matches the only timestamp we have is the
                # capture time; ros_timestamp_us was filled with that fallback
                # by CalibrationRecorder when the trigger was absent.
                msg.header.stamp = stamp_from_us(row["ros_timestamp_us"])
                msg.header.frame_id = row["camera_id"]
                topic = f"/posest/{row['camera_id']}/image_raw"
                bag.write(topic, msg, msg.header.stamp)

        if args.no_imu:
            return

        with open(os.path.join(args.dataset, "imu.csv"), newline="") as imu_file:
            for row in csv.DictReader(imu_file):
                msg = Imu()
                msg.header.stamp = stamp_from_us(row["timestamp_us"])
                msg.header.frame_id = "imu"
                msg.linear_acceleration.x = float(row["accel_x_mps2"])
                msg.linear_acceleration.y = float(row["accel_y_mps2"])
                msg.linear_acceleration.z = float(row["accel_z_mps2"])
                msg.angular_velocity.x = float(row["gyro_x_radps"])
                msg.angular_velocity.y = float(row["gyro_y_radps"])
                msg.angular_velocity.z = float(row["gyro_z_radps"])
                bag.write("/posest/imu/data_raw", msg, msg.header.stamp)


if __name__ == "__main__":
    main()
