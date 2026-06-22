#!/usr/bin/env python3
"""Trim the static (no-motion) head + tail of a recorded bag. Handheld recordings start
and end with the camera sitting still; those frames give the VIO nothing (and can mislead
init). Detect the motion window from the GYRO magnitude (static = noise, moving = spikes),
add a margin, and re-write only the in-window messages to a new bag (all topics preserved).

  trim_static_bag.py <in_bag_dir> <out_bag_dir> [imu_topic] [gyro_thresh] [margin_s]

Defaults: imu=/camera/camera/imu, gyro_thresh=0.15 rad/s, margin=0.5 s. Fast: reads only
the IMU topic to find the window, then a single copy pass.
"""
import sys, math
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu

inb, outb = sys.argv[1], sys.argv[2]
imu_topic = sys.argv[3] if len(sys.argv) > 3 else "/camera/camera/imu"
thresh = float(sys.argv[4]) if len(sys.argv) > 4 else 0.15
margin = float(sys.argv[5]) if len(sys.argv) > 5 else 0.5


def reader(uri, topics=None):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id="mcap"),
           rosbag2_py.ConverterOptions("", ""))
    if topics:
        r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return r


# --- pass 1: gyro magnitude over time (bag timestamps, ns) ---
r1 = reader(inb, [imu_topic])
ts, mag = [], []
while r1.has_next():
    _, data, t = r1.read_next()
    m = deserialize_message(data, Imu)
    g = m.angular_velocity
    ts.append(t)
    mag.append(math.sqrt(g.x * g.x + g.y * g.y + g.z * g.z))
del r1
if not ts:
    print(f"NO IMU on {imu_topic} — aborting"); sys.exit(1)

t0_bag, t1_bag = ts[0], ts[-1]
# first / last sample exceeding threshold = motion onset / offset
moving = [i for i, v in enumerate(mag) if v > thresh]
if not moving:
    print("no motion detected above threshold — keeping full bag"); sys.exit(1)
start = ts[moving[0]] - int(margin * 1e9)
end = ts[moving[-1]] + int(margin * 1e9)
start = max(start, t0_bag)
end = min(end, t1_bag)
total = (t1_bag - t0_bag) / 1e9
kept = (end - start) / 1e9
print(f"  bag span: {total:.1f}s | motion window: [{(start-t0_bag)/1e9:.1f}, "
      f"{(end-t0_bag)/1e9:.1f}]s -> keeping {kept:.1f}s "
      f"(trim head {(start-t0_bag)/1e9:.1f}s + tail {(t1_bag-end)/1e9:.1f}s)")

# --- pass 2: copy every topic's messages inside the window ---
r2 = reader(inb)
w = rosbag2_py.SequentialWriter()
w.open(rosbag2_py.StorageOptions(uri=outb, storage_id="mcap"),
       rosbag2_py.ConverterOptions("", ""))
for tm in r2.get_all_topics_and_types():
    w.create_topic(tm)
n_in, n_out = 0, 0
while r2.has_next():
    topic, data, t = r2.read_next()
    n_in += 1
    if start <= t <= end:
        w.write(topic, data, t)
        n_out += 1
del w, r2
print(f"  wrote {n_out}/{n_in} messages -> {outb}")
