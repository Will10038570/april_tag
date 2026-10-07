"""S3 stuck protection: probe-turn when the yaw direction cannot be told.

S2 turns a yaw of unknown direction into 0, so in Stage 1 the robot can sit
at the stage distance with |yaw| ~ 5 deg: the LQR does not turn, and the
in-position check (on |yaw|) never passes. The size of the yaw is still
measured, so a short turn shows which way reduces it.

Pure Python (no ROS). Call step() once per target frame; it returns a wz
that replaces the LQR's wz, or None to keep the LQR's.
"""

import math
from dataclasses import dataclass
from typing import Optional


@dataclass
class YawProbeConfig:
    trigger_frames: int = 10       # consecutive frames with c_sign < c_sign_low
    c_sign_low: float = 0.5
    trigger_yaw: float = 0.05      # rad, yaw_abs_f above this
    wz: float = 0.03               # rad/s while turning
    turn_time: float = 1.0         # s per turn
    settle_frames: int = 9         # still frames after a turn before yaw_abs_f is compared
    end_yaw_deg: float = 1.5       # done when yaw_abs_f is below this
    change_deg: float = 0.5        # yaw_abs_f change that counts as better / worse
    end_c_sign: float = 1.0        # direction known: hand back to the LQR
    initial_direction: float = 1.0
    max_turns: int = 8


class YawProbe:
    IDLE = 'idle'
    TURN = 'turn'
    SETTLE = 'settle'

    def __init__(self, config: Optional[YawProbeConfig] = None):
        self.cfg = config or YawProbeConfig()
        self.reset()

    def reset(self) -> None:
        self.phase = self.IDLE
        self.direction = math.copysign(1.0, self.cfg.initial_direction)
        self._low_frames = 0
        # a probe finished at its minimum: do not start again until the
        # conditions reset (left Stage 1 / tolerance, or c_sign recovered)
        self._latched = False
        self._before = 0.0
        self._turn_end = 0.0
        self._settled = 0
        self._turns = 0
        self._progress = False
        self._unclear = False
        self._last_turn = False
        self.last_event = ''

    @property
    def active(self) -> bool:
        return self.phase != self.IDLE

    def _finish(self, reason: str) -> float:
        self.phase = self.IDLE
        self._latched = True
        self._low_frames = 0
        self.last_event = f'probe done: {reason}'
        return 0.0

    def _turn(self, now: float) -> float:
        self.phase = self.TURN
        self._turn_end = now + self.cfg.turn_time
        self._turns += 1
        return self.direction * self.cfg.wz

    def step(self, now: float, enabled: bool, c_sign: float, yaw_abs_f: float) -> Optional[float]:
        """enabled: Stage 1 running with x / y within the in-position tolerance."""
        cfg = self.cfg
        self.last_event = ''
        if not enabled:
            if self.phase != self.IDLE or self._latched or self._low_frames:
                self.reset()
            return None
        if c_sign >= cfg.end_c_sign:
            self._low_frames = 0
            self._latched = False
            if self.phase != self.IDLE:
                self.phase = self.IDLE
                self.last_event = 'probe done: direction known, back to LQR'
            return None

        if self.phase == self.IDLE:
            if c_sign >= cfg.c_sign_low:
                self._low_frames = 0
                self._latched = False
                return None
            self._low_frames += 1
            if (self._latched or self._low_frames < cfg.trigger_frames
                    or not yaw_abs_f > cfg.trigger_yaw):
                return None
            self._before = yaw_abs_f
            self._turns = 0
            self._progress = False
            self._unclear = False
            self._last_turn = False
            self.direction = math.copysign(1.0, cfg.initial_direction)
            self.last_event = (f'probe start: |yaw| median {math.degrees(yaw_abs_f):.2f}deg, '
                               f'turning {"+" if self.direction > 0 else "-"}')
            return self._turn(now)

        if self.phase == self.TURN:
            if now < self._turn_end:
                return self.direction * cfg.wz
            if self._last_turn:
                return self._finish('turned back to the minimum')
            self.phase = self.SETTLE
            self._settled = 0
            return 0.0

        # SETTLE: hold still until the median window holds only new frames
        self._settled += 1
        if self._settled < cfg.settle_frames:
            return 0.0
        after = yaw_abs_f
        before, self._before = self._before, after
        change = math.radians(cfg.change_deg)
        if after < math.radians(cfg.end_yaw_deg):
            return self._finish(f'|yaw| median {math.degrees(after):.2f}deg')
        if self._turns >= cfg.max_turns:
            return self._finish(f'{cfg.max_turns} turns')
        if after < before - change:
            self._progress = True
            self.last_event = f'probe: |yaw| {math.degrees(before):.2f} -> {math.degrees(after):.2f}deg, same way'
            return self._turn(now)
        if after > before + change:
            self.direction = -self.direction
            if self._progress:
                # went past the minimum: one turn back, then stop there
                self._last_turn = True
                self.last_event = 'probe: past the minimum, turning back once'
            else:
                self.last_event = f'probe: |yaw| {math.degrees(before):.2f} -> {math.degrees(after):.2f}deg, reversing'
            return self._turn(now)
        if self._progress or self._unclear:
            return self._finish(f'|yaw| median no longer decreasing ({math.degrees(after):.2f}deg)')
        self._unclear = True
        self.last_event = 'probe: no clear change, same way again'
        return self._turn(now)
