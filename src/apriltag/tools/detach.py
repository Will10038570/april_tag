#!/usr/bin/env python3
"""Call /wheelchair/detach and print its response (see wheelchair_cmd.py).

Inside the container, with ROS + the workspace sourced and the same ROS_DOMAIN_ID /
CYCLONEDDS_URI as apriltag_control:
    python3 /mnt/work_space/src/apriltag/tools/detach.py
"""

from wheelchair_cmd import main

if __name__ == '__main__':
    main('detach')
