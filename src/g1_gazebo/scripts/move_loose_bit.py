#!/usr/bin/env python3
"""Move the loose bit to a new pose on the table while the simulation is running.

The bit is laid flat on a hex face with its tip pointing along `yaw`.
x, y are the bit's center in the world frame (meters), yaw in radians.

Example:  ros2 run g1_gazebo move_loose_bit.py 0.36 0.12 --yaw 0.8
"""

import argparse
import math
import subprocess
import sys

MODEL = 'bit_ph2_loose'
WORLD = 'g1_table_world'
TABLE_HEIGHT = 0.75
BIT_LEN = 0.050     # model origin is at the shank end, not the center
BIT_AF = 0.012      # hex across flats


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('x', type=float, help='bit center x in the world frame (m)')
    parser.add_argument('y', type=float, help='bit center y in the world frame (m)')
    parser.add_argument('--yaw', type=float, default=0.0,
                        help='direction the tip points, radians (0 = +x, away from the robot)')
    args = parser.parse_args()

    # Origin = center shifted back half a length along the bit axis.
    ox = args.x - BIT_LEN / 2 * math.cos(args.yaw)
    oy = args.y - BIT_LEN / 2 * math.sin(args.yaw)
    oz = TABLE_HEIGHT + BIT_AF / 2 + 0.001
    # Orientation = Rz(yaw) * Ry(pi/2): bit axis (+z) turned to point along yaw.
    a, b = args.yaw / 2, math.pi / 4
    qw, qx, qy, qz = (math.cos(a) * math.cos(b), -math.sin(a) * math.sin(b),
                      math.cos(a) * math.sin(b), math.sin(a) * math.cos(b))

    req = (f'name: "{MODEL}", position: {{x: {ox:.4f}, y: {oy:.4f}, z: {oz:.4f}}}, '
           f'orientation: {{w: {qw:.6f}, x: {qx:.6f}, y: {qy:.6f}, z: {qz:.6f}}}')
    result = subprocess.run(
        ['gz', 'service', '-s', f'/world/{WORLD}/set_pose', '--reqtype', 'gz.msgs.Pose',
         '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', req],
        capture_output=True, text=True)
    if 'data: true' not in result.stdout:
        sys.exit(f'Failed to move {MODEL}. Is the simulation running?\n'
                 f'{result.stdout}{result.stderr}')
    print(f'Moved {MODEL} to center ({args.x:.3f}, {args.y:.3f}), yaw {args.yaw:.2f} rad')


if __name__ == '__main__':
    main()
