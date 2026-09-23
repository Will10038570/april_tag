"""Safety and target-loss handling helpers."""


def handle_target_lost(now, last_target_time, lost_target_timeout, stopped_on_target_loss, safe_stop_fn, logger):
    """Handle target timeout and safe stop; return updated (last_target_time, stopped_flag)."""
    if last_target_time is None:
        return now, stopped_on_target_loss

    if (now - last_target_time) >= lost_target_timeout and not stopped_on_target_loss:
        logger.warn("Target lost, sending stop command and resetting planner.")
        safe_stop_fn(reset_planner=True)
        return last_target_time, True

    return last_target_time, stopped_on_target_loss
