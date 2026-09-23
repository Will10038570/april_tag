"""Target dispatch helpers."""

from apriltag.ros.ros_io import publish_pose_and_tf, publish_twist
from apriltag.runtime.control_flow import publish_control
from apriltag.runtime.safety_guard import handle_target_lost


def choose_best_target(targets):
    """Choose target with smallest positive forward z; fallback to first target."""
    if not targets:
        return None

    forward_targets = [tr for tr in targets if tr["t"][2] > 0]
    if forward_targets:
        return min(forward_targets, key=lambda item: item["t"][2])
    return targets[0]


def process_targets_pipeline(
    targets,
    now,
    last_target_time,
    stopped_on_target_loss,
    last_time,
    max_dt,
    trajectory_planner,
    lqr_tracker,
    max_vx,
    max_vy,
    max_vw,
    logger,
    cmd_pub,
    clock,
    camera_frame,
    pose_pub,
    traj_pub,
    tf_broadcaster,
    lost_target_timeout,
):
    """One-shot target processing pipeline used by image callback.

    Returns:
        (last_target_time, stopped_on_target_loss, last_time, latest_plan)
    """

    def _safe_stop(reset_planner: bool = False) -> None:
        publish_twist(cmd_pub, 0.0, 0.0, 0.0)
        if reset_planner:
            trajectory_planner.reset()
            lqr_tracker.reset()

    if targets:
        last_target_time = now
        if stopped_on_target_loss:
            logger.info("Target reacquired, resuming control.")
            stopped_on_target_loss = False

        best = choose_best_target(targets)
        latest_plan = None
        try:
            last_time, latest_plan = publish_control(
                best_target=best,
                now=now,
                last_time=last_time,
                max_dt=max_dt,
                trajectory_planner=trajectory_planner,
                lqr_tracker=lqr_tracker,
                max_vx=max_vx,
                max_vy=max_vy,
                max_vw=max_vw,
                publish_twist_fn=lambda vx, vy, vw: publish_twist(cmd_pub, vx, vy, vw),
                logger=logger,
            )
        except Exception as exc:
            logger.warn(f"Control publish failed: {exc}")
            _safe_stop(reset_planner=True)

        publish_pose_and_tf(
            clock=clock,
            camera_frame=camera_frame,
            pose_pub=pose_pub,
            traj_pub=traj_pub,
            tf_broadcaster=tf_broadcaster,
            tag_id=best["id"],
            t_vec=best["t"],
            r_mat=best.get("R", None),
        )
        return last_target_time, stopped_on_target_loss, last_time, latest_plan

    last_target_time, stopped_on_target_loss = handle_target_lost(
        now=now,
        last_target_time=last_target_time,
        lost_target_timeout=lost_target_timeout,
        stopped_on_target_loss=stopped_on_target_loss,
        safe_stop_fn=_safe_stop,
        logger=logger,
    )
    return last_target_time, stopped_on_target_loss, last_time, None
