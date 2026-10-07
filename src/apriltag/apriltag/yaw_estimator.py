"""YawEstimator: clean up the tag's x / y / yaw errors before the LQR.

Pure Python (no ROS), see "Yaw 估計模組交接". Per frame of the target tag:

- S1 quality gate: never drops a frame (x / y are always used); c_quality
  in [0, 1] only weights the yaw. 0 if a corner is within border_px of the
  image edge or the decode needed bit corrections (hamming >= 1), else
  decision_margin mapped linearly from [margin_lo, margin_hi] to [0, 1].
- S2 sign confidence: Δh = left edge - right edge (px). The edge nearer to
  the camera looks longer, so the sign of Δh tells which way the tag is
  turned; SNR = |Δh| / sigma_dh says whether that difference is real.
  c_sign = clip(SNR - 1, 0, 1) scales the measured yaw (soft sign), so a yaw
  whose direction cannot be told is used as 0.
- S4 precision: sigma_size from a distance table at the equivalent
  distance d_eq = d * ref_tag_size / tag_size (the table holds for a tag of
  ref_tag_size), divided by c_quality; infinite beyond the table (yaw unused).
- S5 filter: yaw_f follows the published wz (predict), rejects outliers,
  accepts a new value after a run of consistent rejected frames, and is
  updated with yaw_used weighted by sigma_eff. x / y get an EMA. yaw_abs_f,
  the median of |yaw_meas| over the last frames, is what the in-position
  check and the S3 probe use: it measures the size of the yaw whatever its
  sign.

yaw_f is the LQR input, not a faithful yaw estimate: it is pulled to 0
when the direction is unknown.
"""

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np


@dataclass
class YawEstimatorConfig:
    # S1: corners closer than this to the image edge (px) disable the yaw;
    # decision_margin mapped from [margin_lo, margin_hi] to c_quality [0, 1]
    border_px: float = 10.0
    margin_lo: float = 20.0
    margin_hi: float = 60.0
    # S2: Δh noise (px, std of Δh with camera and tag still) and the SNR
    # ramp of c_sign
    sigma_dh: float = 0.39
    snr_lo: float = 1.0
    snr_hi: float = 2.0
    # S4: yaw precision (deg) at equivalent distances (m) for a tag of
    # ref_tag_size; beyond the last distance the yaw is not used
    s4_distances: Sequence[float] = (0.3, 0.5, 0.7)
    s4_sigmas_deg: Sequence[float] = (1.0, 2.0, 3.0)
    ref_tag_size: float = 0.0475
    # S5: process noise (rad^2/s), outlier gate max(gate_sigma * sigma,
    # gate_min_deg), anti-stuck: accept_frames rejected frames all within
    # consistency_deg of their mean are taken as the new value
    q_proc: float = 0.02 ** 2
    gate_sigma: float = 3.0
    gate_min_deg: float = 3.0
    accept_frames: int = 5
    consistency_deg: float = 3.0
    # frames in the |yaw_meas| median (yaw_abs_f)
    abs_window: int = 9
    # x / y EMA time constant (s)
    xy_tau: float = 0.1


# ---- S1 -----------------------------------------------------------------------

def quality_score(corners, image_width: int, image_height: int, hamming: int,
                  decision_margin: float, border_px: float = 10.0,
                  margin_lo: float = 20.0, margin_hi: float = 60.0) -> float:
    """c_quality in [0, 1]: 0 near the image edge or with bit corrections, else from decision_margin."""
    pts = np.asarray(corners, dtype=float).reshape(-1, 2)
    if pts.shape[0] != 4 or not np.any(pts):
        return 0.0
    if image_width > 0 and image_height > 0:
        if (pts[:, 0].min() < border_px or pts[:, 1].min() < border_px
                or pts[:, 0].max() > image_width - 1 - border_px
                or pts[:, 1].max() > image_height - 1 - border_px):
            return 0.0
    if int(hamming) >= 1:
        return 0.0
    span = max(float(margin_hi) - float(margin_lo), 1e-9)
    return min(max((float(decision_margin) - float(margin_lo)) / span, 0.0), 1.0)


# ---- S2 -----------------------------------------------------------------------

def edge_height_diff(corners) -> float:
    """Δh = left edge length - right edge length (px) of a tag's 4 corner pixels.

    The left edge joins the two corners with the smallest image x, the right
    edge the other two, so the corner order does not matter (the tag must
    not be rolled by ~45 deg or more). Δh has the same sign as the yaw from
    math_utils.rotation_matrix_to_yaw_error.
    """
    pts = np.asarray(corners, dtype=float).reshape(4, 2)
    order = np.argsort(pts[:, 0])
    left, right = pts[order[:2]], pts[order[2:]]
    return float(np.linalg.norm(left[0] - left[1]) - np.linalg.norm(right[0] - right[1]))


def sign_confidence(dh: float, sigma_dh: float, snr_lo: float = 1.0, snr_hi: float = 2.0):
    """Return (SNR, c_sign): c_sign is 0 at SNR <= snr_lo, 1 at SNR >= snr_hi, linear between."""
    snr = abs(float(dh)) / max(float(sigma_dh), 1e-9)
    c_sign = min(max((snr - snr_lo) / max(snr_hi - snr_lo, 1e-9), 0.0), 1.0)
    return snr, c_sign


def soft_sign_yaw(yaw_meas: float, c_sign: float, dh: float) -> float:
    """yaw_used = c_sign * yaw_meas, with the direction taken from Δh.

    Where c_sign > 0, Δh is clearly above its noise, so its sign is the more
    reliable direction: a pose flip (yaw_meas with the wrong sign) is
    corrected. With consistent signs this is exactly c_sign * yaw_meas.
    """
    if c_sign <= 0.0 or dh == 0.0:
        return 0.0
    return float(c_sign) * math.copysign(abs(float(yaw_meas)), dh)


# ---- S4 -----------------------------------------------------------------------

def equivalent_distance(d: float, tag_size: float, ref_tag_size: float) -> float:
    """Distance at which a tag of ref_tag_size looks as large as tag_size does at d."""
    return float(d) * float(ref_tag_size) / float(tag_size)


def sigma_size(d: float, tag_size: float, ref_tag_size: float, c_quality: float = 1.0,
               distances: Sequence[float] = (0.3, 0.5, 0.7),
               sigmas_deg: Sequence[float] = (1.0, 2.0, 3.0)) -> float:
    """Yaw precision (rad) from the distance table; inf when the yaw must not be used."""
    if c_quality <= 0.0:
        return math.inf
    d_eq = equivalent_distance(d, tag_size, ref_tag_size)
    if d_eq > distances[-1]:
        return math.inf
    sigma_deg = float(np.interp(d_eq, distances, sigmas_deg))  # clamps below distances[0]
    return math.radians(sigma_deg) / float(c_quality)


def effective_sigma(sigma_size_rad: float, c_sign: float, yaw_meas: float) -> float:
    """sigma_eff = sqrt(sigma_size^2 + ((1 - c_sign) |yaw_meas|)^2)."""
    return math.hypot(sigma_size_rad, (1.0 - float(c_sign)) * abs(float(yaw_meas)))


# ---- S5 -----------------------------------------------------------------------

class YawFilter:
    """1-D yaw filter driven by the published wz, with outlier gate and anti-stuck."""

    def __init__(self, config: YawEstimatorConfig):
        self.cfg = config
        self.reset()

    def reset(self) -> None:
        self.yaw_f = 0.0
        self.P = 0.0
        self.initialized = False
        self._rejected = []
        self._abs = deque(maxlen=max(int(self.cfg.abs_window), 1))

    def yaw_abs_f(self) -> float:
        return float(np.median(self._abs)) if self._abs else math.inf

    def update(self, yaw_used: float, sigma_eff: float, yaw_meas: float,
               wz_cmd: float, dt: float) -> Optional[bool]:
        """One frame; returns True accepted, False rejected, None not used (sigma_eff inf)."""
        cfg = self.cfg
        self._abs.append(abs(float(yaw_meas)))
        if self.initialized and dt > 0.0:
            self.yaw_f -= float(wz_cmd) * dt
            self.P += cfg.q_proc * dt
        if not math.isfinite(sigma_eff):
            return None

        r = sigma_eff ** 2
        if not self.initialized:
            self.yaw_f, self.P, self.initialized = float(yaw_used), r, True
            return True

        gate = max(cfg.gate_sigma * math.sqrt(self.P + r), math.radians(cfg.gate_min_deg))
        if abs(yaw_used - self.yaw_f) > gate:
            self._rejected.append(float(yaw_used))
            self._rejected = self._rejected[-int(cfg.accept_frames):]
            if len(self._rejected) >= int(cfg.accept_frames):
                mean = float(np.mean(self._rejected))
                if max(abs(v - mean) for v in self._rejected) <= math.radians(cfg.consistency_deg):
                    self.yaw_f, self.P = mean, r
                    self._rejected = []
                    return True
            return False

        self._rejected = []
        k = self.P / (self.P + r)
        self.yaw_f += k * (float(yaw_used) - self.yaw_f)
        self.P = (1.0 - k) * self.P
        return True


class EmaFilter:
    def __init__(self, tau: float):
        self.tau = float(tau)
        self.value: Optional[float] = None

    def reset(self) -> None:
        self.value = None

    def update(self, x: float, dt: float) -> float:
        if self.value is None or self.tau <= 0.0:
            self.value = float(x)
        elif dt > 0.0:
            self.value += (1.0 - math.exp(-dt / self.tau)) * (float(x) - self.value)
        return self.value


@dataclass
class YawEstimate:
    x_f: float
    y_f: float
    yaw_f: float
    yaw_abs_f: float
    yaw_meas: float
    yaw_used: float
    dh: float
    snr: float
    c_sign: float
    c_quality: float
    sigma_size: float
    sigma_eff: float
    accepted: Optional[bool] = field(default=None)


class YawEstimator:
    """S1 -> S2 -> S4 -> S5 for one target tag; reset() when the target changes."""

    def __init__(self, config: Optional[YawEstimatorConfig] = None):
        self.cfg = config or YawEstimatorConfig()
        self.yaw_filter = YawFilter(self.cfg)
        self.x_filter = EmaFilter(self.cfg.xy_tau)
        self.y_filter = EmaFilter(self.cfg.xy_tau)

    def reset(self) -> None:
        self.yaw_filter.reset()
        self.x_filter.reset()
        self.y_filter.reset()

    def update(self, x: float, y: float, yaw_meas: float, d: float, corners,
               image_width: int, image_height: int, hamming: int, decision_margin: float,
               tag_size: float, wz_cmd: float, dt: float) -> YawEstimate:
        """One frame of the target tag.

        x, y, yaw_meas: raw errors (m, m, rad); d: forward distance t_z (m);
        corners: 4 corner pixels; tag_size: this tag's real size (m);
        wz_cmd: wz actually published since the previous frame; dt: time
        since the previous frame (s).
        """
        cfg = self.cfg
        has_corners = corners is not None and np.size(corners) == 8 and np.any(corners)
        c_quality = quality_score(corners, image_width, image_height, hamming, decision_margin,
                                  cfg.border_px, cfg.margin_lo, cfg.margin_hi) if has_corners else 0.0
        dh = edge_height_diff(corners) if has_corners else 0.0
        snr, c_sign = sign_confidence(dh, cfg.sigma_dh, cfg.snr_lo, cfg.snr_hi)
        yaw_used = soft_sign_yaw(yaw_meas, c_sign, dh)
        s_size = sigma_size(d, tag_size, cfg.ref_tag_size, c_quality,
                            cfg.s4_distances, cfg.s4_sigmas_deg)
        s_eff = effective_sigma(s_size, c_sign, yaw_meas) if math.isfinite(s_size) else math.inf
        accepted = self.yaw_filter.update(yaw_used, s_eff, yaw_meas, wz_cmd, dt)
        return YawEstimate(
            x_f=self.x_filter.update(x, dt),
            y_f=self.y_filter.update(y, dt),
            yaw_f=self.yaw_filter.yaw_f,
            yaw_abs_f=self.yaw_filter.yaw_abs_f(),
            yaw_meas=float(yaw_meas),
            yaw_used=yaw_used,
            dh=dh,
            snr=snr,
            c_sign=c_sign,
            c_quality=c_quality,
            sigma_size=s_size,
            sigma_eff=s_eff,
            accepted=accepted,
        )
