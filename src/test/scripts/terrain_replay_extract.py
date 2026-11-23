#!/usr/bin/env python3
"""把 rosbag 里地图节点的输入（odom + /lio/cloud_world）导出成 test_terrain_replay 读的二进制帧文件。

只读 bag。默认每帧 = 点云到达时最近一条 /lio/base_odom（与 ros2_node 里 odom/cloud 两个回调的实际时序一致）。
传 --odom-topic 时改用该话题（如 /lio/odom，与点云同一后验状态/时间戳）：位姿先由
T_w_base = T_w_imu * inverse(T_base_imu) 转换到 base，再按 header stamp 配对（相同优先，否则最近更早）。

格式（小端）：
  int32 n_frames
  每帧: float64 stamp, float64 px,py,pz, float64 qw,qx,qy,qz, int32 n_pts, n_pts * float32[3]

用法:
  python3 terrain_replay_extract.py <bag.db3> <out.bin>
  python3 terrain_replay_extract.py <bag.db3> <out.bin> \
      --odom-topic /lio/odom --imu-in-base 0.011 0.02329 0.10588 0 0 0
"""
import argparse
import bisect
import math
import sqlite3
import struct

import numpy as np
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2

ODOM = '/lio/base_odom'
CLOUD = '/lio/cloud_world'


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def quat_from_euler(roll, pitch, yaw):
    """(w,x,y,z)，对应 R = Rz(yaw) * Ry(pitch) * Rx(roll)，角度为弧度。"""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (cy * cp * cr + sy * sp * sr,
            cy * cp * sr - sy * sp * cr,
            cy * sp * cr + sy * cp * sr,
            sy * cp * cr - cy * sp * sr)


def quat_rotate(q, v):
    w, x, y, z = q
    qv = np.array([x, y, z])
    t = 2.0 * np.cross(qv, v)
    return v + w * t + np.cross(qv, t)


def odom_pose_base(m, imu_in_base):
    """返回 [px,py,pz,qw,qx,qy,qz]；imu_in_base 为 None 时原样返回。"""
    p = m.pose.pose.position
    q = m.pose.pose.orientation
    if imu_in_base is None:
        return (p.x, p.y, p.z, q.w, q.x, q.y, q.z)

    x, y, z, roll, pitch, yaw = imu_in_base
    q_wi = (q.w, q.x, q.y, q.z)
    norm = math.sqrt(sum(v * v for v in q_wi))
    q_wi = tuple(v / norm for v in q_wi)
    # T_base_imu 的旋转；其逆为共轭四元数。
    q_bi = quat_from_euler(math.radians(roll), math.radians(pitch), math.radians(yaw))
    q_ib = (q_bi[0], -q_bi[1], -q_bi[2], -q_bi[3])
    t_ib = quat_rotate(q_ib, -np.array([x, y, z]))
    p_wb = np.array([p.x, p.y, p.z]) + quat_rotate(q_wi, t_ib)
    q_wb = quat_mul(q_wi, q_ib)
    norm = math.sqrt(sum(v * v for v in q_wb))
    q_wb = tuple(v / norm for v in q_wb)
    return (p_wb[0], p_wb[1], p_wb[2], q_wb[0], q_wb[1], q_wb[2], q_wb[3])


def cloud_xyz(m):
    off = {f.name: f.offset for f in m.fields}
    dt = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
                   'offsets': [off['x'], off['y'], off['z']], 'itemsize': m.point_step})
    a = np.frombuffer(m.data, dtype=dt, count=m.width * m.height)
    xyz = np.stack([a['x'], a['y'], a['z']], 1).astype('<f4')
    return xyz[np.isfinite(xyz).all(1)]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('bag', help='rosbag2 .db3 路径')
    ap.add_argument('out', help='输出二进制帧文件')
    ap.add_argument('--odom-topic', default=None,
                    help='建图位姿话题（如 /lio/odom）；缺省沿用 /lio/base_odom 的接收时序配对')
    ap.add_argument('--imu-in-base', nargs=6, type=float, default=None,
                    metavar=('X', 'Y', 'Z', 'ROLL', 'PITCH', 'YAW'),
                    help='IMU 在 base 系下的位姿 [x,y,z,roll,pitch,yaw]，m/deg')
    args = ap.parse_args()
    if args.odom_topic and args.imu_in_base is None:
        ap.error('--odom-topic 需要同时给出 --imu-in-base（可用 0 0 0 0 0 0 表示重合）')

    odom_topic = args.odom_topic or ODOM
    con = sqlite3.connect(f'file:{args.bag}?mode=ro&immutable=1', uri=True)
    cur = con.cursor()
    tid = {n: i for i, n in cur.execute('select id, name from topics')}
    for name in (odom_topic, CLOUD):
        if name not in tid:
            raise SystemExit(f'bag 中缺少话题 {name}')

    if args.odom_topic:
        # 按 header stamp 建索引，点云优先用完全相同 stamp，否则用最近更早的位姿。
        odom_by_stamp = {}
        for (data,) in cur.execute(
                'select data from messages where topic_id = ? order by timestamp', (tid[odom_topic],)):
            m = deserialize_message(data, Odometry)
            odom_by_stamp[stamp_ns(m.header.stamp)] = odom_pose_base(m, args.imu_in_base)
        stamps = sorted(odom_by_stamp)

        frames, n_cloud, n_exact = [], 0, 0
        for ts, data in cur.execute(
                'select timestamp, data from messages where topic_id = ? order by timestamp', (tid[CLOUD],)):
            m = deserialize_message(data, PointCloud2)
            n_cloud += 1
            cs = stamp_ns(m.header.stamp)
            if cs in odom_by_stamp:
                n_exact += 1
                odom = odom_by_stamp[cs]
            else:
                idx = bisect.bisect_right(stamps, cs) - 1
                if idx < 0:
                    continue
                odom = odom_by_stamp[stamps[idx]]
            frames.append((ts * 1e-9, odom, cloud_xyz(m)))
        summary = f'{len(frames)} frames -> {args.out} (odom={odom_topic}, identical_stamp={n_exact}/{n_cloud})'
    else:
        rows = cur.execute(
            'select topic_id, timestamp, data from messages where topic_id in (?, ?) order by timestamp',
            (tid[ODOM], tid[CLOUD])).fetchall()
        frames, odom = [], None
        for topic, ts, data in rows:
            if topic == tid[ODOM]:
                m = deserialize_message(data, Odometry)
                odom = odom_pose_base(m, None)
                continue
            if odom is None:
                continue
            frames.append((ts * 1e-9, odom, cloud_xyz(deserialize_message(data, PointCloud2))))
        summary = f'{len(frames)} frames -> {args.out}'

    with open(args.out, 'wb') as f:
        f.write(struct.pack('<i', len(frames)))
        for ts, o, xyz in frames:
            f.write(struct.pack('<8d', ts, *o))
            f.write(struct.pack('<i', len(xyz)))
            f.write(xyz.tobytes())
    print(summary)


if __name__ == '__main__':
    main()
