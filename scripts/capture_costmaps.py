#!/usr/bin/env python3
# capture_costmaps.py — subscribe slamko's GLOBAL (~/volumetric_costmap, latched) + LOCAL
# (~/local_costmap, rolling) OccupancyGrids and render both to a side-by-side PNG. Proof, not claim.
#   usage: ros2 run ... OR  python3 capture_costmaps.py [out.png] [node_ns=/provider_fusion_node]
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

OUT = sys.argv[1] if len(sys.argv) > 1 else '/tmp/costmaps.png'
NS = sys.argv[2] if len(sys.argv) > 2 else '/provider_fusion_node'

def grid_to_img(msg):
    w, h = msg.info.width, msg.info.height
    a = np.array(msg.data, dtype=np.int16).reshape(h, w)  # row-major, +row→+y
    return a, msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y

class Cap(Node):
    def __init__(self):
        super().__init__('costmap_capture')
        self.g = None; self.l = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        vol = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, f'{NS}/volumetric_costmap', self.cb_g, latched)
        self.create_subscription(OccupancyGrid, f'{NS}/local_costmap', self.cb_l, vol)
    def cb_g(self, m): self.g = m
    def cb_l(self, m): self.l = m

def render(n, out):
    if n.g is None and n.l is None:
        return False
    cmap = ListedColormap(['#1a1a2e', '#2bd66b', '#e63946'])  # -1 unknown / 0 free / 100 occ
    fig, axes = plt.subplots(1, 2, figsize=(16, 8), dpi=120)
    for ax, (name, msg) in zip(axes, [('GLOBAL ~/volumetric_costmap (latched, whole map)', n.g),
                                      ('LOCAL ~/local_costmap (rolling, from live nvblox)', n.l)]):
        if msg is None:
            ax.set_title(f'{name}\n(not received)', color='#eee'); ax.set_facecolor('#111'); continue
        a, res, ox, oy = grid_to_img(msg)
        disp = np.full(a.shape, 0, dtype=np.uint8)  # 0 unknown
        disp[a == 0] = 1; disp[a >= 100] = 2
        ax.imshow(disp, origin='lower', cmap=cmap, vmin=0, vmax=2,
                  extent=[ox, ox + a.shape[1]*res, oy, oy + a.shape[0]*res], interpolation='nearest')
        occ = int((a >= 100).sum()); free = int((a == 0).sum())
        ax.set_title(f'{name}\n{a.shape[1]}x{a.shape[0]} @ {res:.2f} m  ·  {occ} occ / {free} free cells',
                     color='#eee', fontsize=11)
        ax.set_aspect('equal'); ax.tick_params(colors='#888')
        for s in ax.spines.values(): s.set_color('#333')
    fig.patch.set_facecolor('#0a0a0a'); fig.tight_layout()
    fig.savefig(out, facecolor='#0a0a0a', bbox_inches='tight')
    plt.close(fig)
    return True

def main():
    # CONTINUOUS capture: spin for up to `dur` s, re-rendering every ~8 s so the LAST render before
    # the node dies has the FULL map (the global grows as the bag plays — a mid-run grab is only the
    # room mapped so far). 3rd arg = duration seconds (default 240).
    import time
    dur = float(sys.argv[3]) if len(sys.argv) > 3 else 240.0
    rclpy.init()
    n = Cap()
    t0 = time.time(); last = 0.0; saved = False
    while rclpy.ok() and time.time() - t0 < dur:
        rclpy.spin_once(n, timeout_sec=0.5)
        if time.time() - last > 8.0 and (n.g is not None or n.l is not None):
            if render(n, OUT):
                saved = True
                occ = sum(1 for v in n.g.data if v >= 100) if n.g else 0
                print(f'[{int(time.time()-t0)}s] saved {OUT} (global occ cells: {occ})', flush=True)
            last = time.time()
    if saved: print('final map saved (last update before exit)')
    else: print('no costmaps received')
    rclpy.shutdown()

if __name__ == '__main__':
    main()
