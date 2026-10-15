#!/usr/bin/env bash
# Send /up/dock_wheelchair, wait for it to finish (any result), wait
# WAIT_BETWEEN_S, then send /up/undock_wheelchair and wait for it to finish.
# Results are only printed, not checked: undock is sent even if dock failed
# (apriltag_control accepts it in IDLE and really backs up DISTANCE_M).
#
# Run inside the april_tag_serive container with ROS + the workspace sourced,
# using the shell's ROS_DOMAIN_ID / CYCLONEDDS_URI:
#     docker exec -it april_tag_serive bash -ic '/mnt/work_space/src/apriltag/tools/dock_then_undock.sh'
# Use -it: Ctrl+C then reaches ros2 in the container, which cancels the running
# goal and prints its result; the script then exits without sending the next
# goal. Without -t, Ctrl+C only stops the host-side docker exec.

# ---- parameters -------------------------------------------------------------
TAG_ID=0               # -1 = any id; must match the tag apriltag_detection tracks
TAG_FAMILY=tag36h11    # must match apriltag_detection's tracking_tag_family, else TAG_LOST
DISTANCE_M=1.0         # m, undock backs up until the tag is this far ahead; > 0
WAIT_BETWEEN_S=5      # s, wait between dock finishing and sending undock
# -----------------------------------------------------------------------------

log() { echo "[$(date '+%H:%M:%S')] $*"; }

# ros2 action send_goal cancels its goal on SIGINT and then exits normally, so
# stop here instead of going on to the next step. bash runs the trap after the
# foreground ros2 has finished.
trap 'log "interrupted, not sending further goals"; exit 130' INT

log "sending dock goal: tag_id=${TAG_ID} tag_family=${TAG_FAMILY}"
ros2 action send_goal -f /up/dock_wheelchair apriltag_interfaces/action/DockWheelchair \
    "{tag_id: ${TAG_ID}, tag_family: ${TAG_FAMILY}}"

log "dock finished, waiting ${WAIT_BETWEEN_S} s"
sleep "${WAIT_BETWEEN_S}"

log "sending undock goal: tag_id=${TAG_ID} tag_family=${TAG_FAMILY} distance_m=${DISTANCE_M}"
ros2 action send_goal -f /up/undock_wheelchair apriltag_interfaces/action/UndockWheelchair \
    "{tag_id: ${TAG_ID}, tag_family: ${TAG_FAMILY}, distance_m: ${DISTANCE_M}}"

log "undock finished, done"
