#!/usr/bin/env python3
"""把 rosbag 里地图节点的输入（/lio/base_odom + /lio/cloud_world）导出成 test_terrain_replay 读的二进制帧文件。

只读 bag。每帧 = 点云到达时最近一条 base_odom（与 ros2_node 里 odom/cloud 两个回调的实际时序一致）。

格式（小端）：
  int32 n_frames
  每帧: float64 stamp, float64 px,py,pz, float64 qw,qx,qy,qz, int32 n_pts, n_pts * float32[3]

用法: python3 terrain_replay_extract.py <bag.db3> <out.bin>
"""
import sqlite3
import struct
import sys

import numpy as np
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2

ODOM = '/lio/base_odom'
CLOUD = '/lio/cloud_world'


def main():
    bag, out = sys.argv[1], sys.argv[2]
    con = sqlite3.connect(f'file:{bag}?mode=ro&immutable=1', uri=True)
    cur = con.cursor()
    tid = {n: i for i, n in cur.execute('select id, name from topics')}
    rows = cur.execute(
        'select topic_id, timestamp, data from messages where topic_id in (?, ?) order by timestamp',
        (tid[ODOM], tid[CLOUD])).fetchall()
    frames, odom = [], None
    for topic, ts, data in rows:
        if topic == tid[ODOM]:
            m = deserialize_message(data, Odometry)
            p, q = m.pose.pose.position, m.pose.pose.orientation
            odom = (p.x, p.y, p.z, q.w, q.x, q.y, q.z)
            continue
        if odom is None:
            continue
        m = deserialize_message(data, PointCloud2)
        off = {f.name: f.offset for f in m.fields}
        dt = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
                       'offsets': [off['x'], off['y'], off['z']], 'itemsize': m.point_step})
        a = np.frombuffer(m.data, dtype=dt, count=m.width * m.height)
        xyz = np.stack([a['x'], a['y'], a['z']], 1).astype('<f4')
        xyz = xyz[np.isfinite(xyz).all(1)]
        frames.append((ts * 1e-9, odom, xyz))
    with open(out, 'wb') as f:
        f.write(struct.pack('<i', len(frames)))
        for ts, o, xyz in frames:
            f.write(struct.pack('<8d', ts, *o))
            f.write(struct.pack('<i', len(xyz)))
            f.write(xyz.tobytes())
    print(f'{len(frames)} frames -> {out}')


if __name__ == '__main__':
    main()
