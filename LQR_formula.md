# AprilTag 對位控制：LQR 公式整理

適用範圍：`apriltag_control` 的 **Stage 1 / Stage 2**。Leave(Detaching)不使用 LQR,固定以 `vx = -max_vx` 後退。

相關程式：

| 內容 | 檔案 |
|---|---|
| 誤差計算 | `src/apriltag/apriltag/domain/math_utils.py`(`optical_to_control_error`)、`control_node.py`(約 1066–1093 行) |
| 參考軌跡 planner | `src/apriltag/apriltag/control.py`(`TrajectoryPlanner`) |
| LQR | `src/apriltag/apriltag/control.py`(`LQRTracker`) |
| 限速 / 發布 | `src/apriltag/apriltag/runtime/control_flow.py`(`publish_control`、`clamp_control`) |

---

## 0. 座標系

| 座標系 | 軸向 |
|---|---|
| 相機 optical frame | x → 右、y → 下、z → 前 |
| 控制座標(車體) | x → 前、y → 左、yaw → 逆時針為正(由上往下看) |

---

## 1. 量測：tag 相對相機的位置

detection 每張影像給出 tag 在相機 optical frame 的位置 $t = (t_x, t_y, t_z)$ 與旋轉矩陣 $R$。

$$
\text{forward} = t_z ,\qquad \text{lateral} = -t_x ,\qquad \text{yaw}_{meas} = \operatorname{atan2}(-R_{02},\ R_{22})
$$

| 變數 | 物理意義 | 單位 |
|---|---|---|
| $t_x, t_z$ | tag 中心在相機座標的右方 / 前方距離 | m |
| forward | 相機到 tag 的前向距離 | m |
| lateral | tag 在相機左方的距離(左為正) | m |
| $\text{yaw}_{meas}$ | 車體相對 tag 正面的偏航角(正對時為 0) | rad |

這三個值會經過 `YawEstimator` 濾波(平滑雜訊、判斷 yaw 正負號),得到 $x_f$、$y_f$、$\text{yaw}_f$。

---

## 2. 誤差 $e$(LQR 的狀態)

每個 stage 有自己的 calibration 目標點 `z_ref`、`x_ref`、`yaw_ref`(在 `april_tag_config.json` 的 `calibration`)。

$$
\begin{aligned}
x_{err} &= x_f - z_{ref} \\
y_{err} &= y_f \quad(\text{濾波前 } = \text{lateral} + x_{ref}^{cal}) \\
\text{yaw}_{err} &= \text{yaw}_f - \text{yaw}_{ref}^{cal}
\end{aligned}
\qquad
e = \begin{bmatrix} x_{err} \\ y_{err} \\ \text{yaw}_{err} \end{bmatrix}
$$

| 變數 | 物理意義 | 單位 | 正值代表 |
|---|---|---|---|
| $x_{err}$ | 前向距離還差多少到目標距離 | m | 離 tag 太遠,要往前 |
| $y_{err}$ | 橫向偏差 | m | 目標在左邊,要往左 |
| $\text{yaw}_{err}$ | 角度偏差 | rad | 要逆時針轉 |
| $z_{ref}$ | 目標前向距離(calibration;出廠 stage1 0.50 m、stage2 0.28 m) | m | |
| $x_{ref}^{cal}$ | 目標橫向偏移(calibration;出廠 0.026 m) | m | |
| $\text{yaw}_{ref}^{cal}$ | 目標偏航角(calibration;出廠 0) | rad | |

> 注意：$x_{ref}^{cal}$、$\text{yaw}_{ref}^{cal}$ 是 **calibration 的目標點**,和下一節 planner 的 $e_{ref}$ 不是同一個東西。

---

## 3. 參考軌跡 $e_{ref}$(TrajectoryPlanner)

用途：讓車子**平順起步**,不要一開始就用全部誤差去算速度。

$$
\alpha = \min\!\left(1,\ \frac{dt}{\tau}\right),\qquad \tau = 0.5\ \text{s}
$$

$$
\begin{aligned}
\text{第一幀:}\quad & e_{ref} = e \\
\text{之後每幀:}\quad & e_{ref}[k] = e_{ref}[k-1] + \alpha\,\bigl(0 - e_{ref}[k-1]\bigr) = (1-\alpha)\,e_{ref}[k-1]
\end{aligned}
$$

$$
e_{ref} = \begin{bmatrix} x_{ref} \\ y_{ref} \\ \text{yaw}_{ref} \end{bmatrix}
$$

| 變數 | 物理意義 | 單位 |
|---|---|---|
| $e_{ref}$ | 「此刻允許保留的誤差」,從初始誤差以時間常數 $\tau$ 指數衰減到 0 | m, m, rad |
| $\tau$ | 衰減時間常數(`smooth_tau`),約 $3\tau = 1.5$ s 後衰減到 5% 以下 | s |
| $\alpha$ | 每一步往 0 靠近的比例 | 無單位 |
| $dt$ | 兩次控制之間的時間,限制在 $[0.001,\ 0.2]$ s(`max_dt`) | s |

$e_{ref}$ 只在第一幀看量測值，之後只隨時間衰減，**不看量測、也不看速度**(前饋)。stage 開始或換 target 時 reset。

---

## 4. LQR

### 4.1 設計用的模型

把三個軸都當成獨立的積分器：速度 $u$ 持續 $dt$ 秒，誤差就減少 $u \cdot dt$。

$$
e[k+1] = A\,e[k] + B\,u[k],\qquad A = I_3,\quad B = -dt \cdot I_3
$$

$$
u = \begin{bmatrix} v_x \\ v_y \\ \omega_z \end{bmatrix}
$$

| 變數 | 物理意義 | 單位 |
|---|---|---|
| $v_x$ | 前進速度(正 = 往前) | m/s |
| $v_y$ | 橫移速度(正 = 往左) | m/s |
| $\omega_z$ | 旋轉角速度(正 = 逆時針) | rad/s |

> 此模型沒有包含 G7+ 的延遲、加速度限制、低速 deadband,也沒有包含相機與車體中心的幾何耦合；這些靠回授修正。

### 4.2 代價函數與權重

$$
J = \sum_{k} \left( e[k]^\top Q\, e[k] + u[k]^\top R\, u[k] \right)
$$

$$
Q = \operatorname{diag}(1,\ 1,\ 0.5),\qquad R = \operatorname{diag}(3,\ 1,\ 3)
$$

| 變數 | 物理意義 |
|---|---|
| $Q$ | 誤差的懲罰權重(x, y, yaw)。越大 → 越在意該軸誤差 → 速度越大 |
| $R$ | 速度的懲罰權重(vx, vy, wz)。越大 → 越捨不得出力 → 速度越小 |

(`control_node.py`:`LQRTracker(r_weights=(3.0, 1.0, 3.0))`,`q_weights` 用預設值)

### 4.3 求增益 $K$

解離散代數 Riccati 方程(`scipy.linalg.solve_discrete_are`):

$$
P = A^\top P A - A^\top P B\,(R + B^\top P B)^{-1} B^\top P A + Q
$$

$$
K = (R + B^\top P B)^{-1} B^\top P A
$$

因為 $A$、$B$、$Q$、$R$ 都是對角矩陣，三軸互相獨立，每軸的增益可寫成純量公式：

$$
p = \frac{q\,dt^2 + \sqrt{q^2 dt^4 + 4\,dt^2\,q\,r}}{2\,dt^2},\qquad
|k| = \frac{dt \cdot p}{r + dt^2\,p} \;\approx\; \sqrt{q / r}\quad(dt \text{ 很小時})
$$

| 軸 | $q$ | $r$ | $\lvert k\rvert$(dt = 0.033 s) | $\lvert k\rvert$(dt = 0.1 s) |
|---|---|---|---|---|
| x | 1 | 3 | 0.572 | 0.561 |
| y | 1 | 1 | 0.984 | 0.951 |
| yaw | 0.5 | 3 | 0.406 | 0.400 |

| 變數 | 物理意義 | 單位 |
|---|---|---|
| $P$ | Riccati 方程的解(未來總代價的權重) | — |
| $K$ | 回授增益：誤差每差 1 單位，要給多少速度 | 1/s |

$dt$ 與上次計算時相差 20% 以內會沿用快取的 $K$。

### 4.4 控制律

$$
u = -K\,(e - e_{ref})
$$

因為 $B$ 帶負號，$K$ 的對角元素是負的，展開後每軸就是一個**比例控制(P 控制)**:

$$
\boxed{
\begin{aligned}
v_x &= |k_x| \cdot (x_{err} - x_{ref}) &&\approx 0.56 \cdot (x_{err} - x_{ref}) \\
v_y &= |k_y| \cdot (y_{err} - y_{ref}) &&\approx 0.95 \cdot (y_{err} - y_{ref}) \\
\omega_z &= |k_{yaw}| \cdot (\text{yaw}_{err} - \text{yaw}_{ref}) &&\approx 0.40 \cdot (\text{yaw}_{err} - \text{yaw}_{ref})
\end{aligned}
}
$$

- 速度**只由當下誤差與 $e_{ref}$ 決定，不會加上前一次的速度**。
- 回授：每一幀由相機重新量測 $e$,車子動了，$e$ 就變，速度跟著變。

---

## 5. LQR 之後的處理(依序)

### 5.1 S3 yaw probe 覆寫

Stage 1 時 x / y 已在 tolerance 內但無法判斷 yaw 方向時，$\omega_z$ 會被探測轉動取代(`wz_override`),$v_x$、$v_y$ 不變。

### 5.2 非有限值保護

任一軸是 NaN / inf → 三軸都送 0。

### 5.3 限速(`clamp_control`,每軸獨立)

$$
\begin{aligned}
&\text{(1) 上限:}\quad v \leftarrow \operatorname{clip}(v,\ -v_{max},\ v_{max}) \\
&\text{(2) 進場縮放(只對 } v_x > 0\text{):}\quad v_x \leftarrow s \cdot v_x \\
&\text{(3) 下限:}\quad v \ne 0 \text{ 且 } |v| < v_{min} \ \Rightarrow\ v \leftarrow \operatorname{sign}(v)\cdot v_{min}
\end{aligned}
$$

| 變數 | 物理意義 | 出廠值 |
|---|---|---|
| $v_{max}$ | 每軸最大速度(`max_vx` / `max_vy` / `max_vw`) | 0.1 m/s、0.1 m/s、0.05 rad/s |
| $v_{min}$ | 每軸最小速度，補償 G7+ 低速 deadband(`min_vx` / `min_vy` / `min_vw`) | 0.01、0.01、0.01 |
| $s$ | Stage 1 直線進場的前進縮放(`vx_scale`,0–1);Stage 2 為 1 | — |

### 5.4 Stage 1 直線進場縮放 $s$

$$
\text{ramp}(v;\ full,\ zero) =
\begin{cases}
1 & |v| \le full \\
\dfrac{zero - |v|}{zero - full} & full < |v| < zero \\
0 & |v| \ge zero
\end{cases}
$$

$$
s =
\begin{cases}
\text{ramp}(\,lat;\ 0.03,\ 0.08\,) & \text{yaw 尚無效} \\
\text{ramp}(\,lat;\ 0.03,\ 0.08\,)\cdot \text{ramp}(\,\text{yaw}_{err};\ 0.035,\ 0.105\,) & \text{yaw 有效}
\end{cases}
$$

| 變數 | 物理意義 | 單位 |
|---|---|---|
| $lat$ | yaw 無效時 = 相機 $y_{err}$;yaw 有效時 = 相機到 stage1 進場線的距離(`approach_lateral_error`) | m |
| $s$ | 偏離進場線越多，前進越慢；偏太多就不前進，只橫移 / 轉向 | 無單位 |

`stage1_approach.enabled: false` 時 $s = 1$。

### 5.5 發布

| Stage | topic |
|---|---|
| Stage 1 | `/cmd_vel` |
| Stage 2 | `/pre_cmd_vel`(G7+ 精準模式) |

---

## 6. 完整流程

```
① 相機影像
   ↓
② detection:tag 位置 t、旋轉 R
   ↓
③ YawEstimator 濾波 → 誤差 e = [x_err, y_err, yaw_err]
   ↓
④ planner:e_ref 從 e 開始，以 τ = 0.5 s 衰減到 0
   ↓
⑤ LQR:u = |K| · (e − e_ref)
   ↓
⑥ probe 覆寫 wz → 上限 → 進場縮放 → 下限
   ↓
⑦ /cmd_vel 或 /pre_cmd_vel
   ↓
⑧ 車子移動 → 相機看到的 tag 改變 → 回到 ①(回授)
```

---

## 7. 數值範例(x 軸)

假設 $x_{err} = 0.20$ m,車子尚未移動,$dt \approx 0.033$ s:

| 時間 | $x_{err}$ | $x_{ref}$ | $x_{err} - x_{ref}$ | $v_x = 0.56 \times$ 差值 | 限速後 |
|---|---|---|---|---|---|
| 0 s | 0.20 | 0.20 | 0 | 0 | 0 |
| 0.5 s | 0.20 | ≈ 0.07 | 0.13 | 0.073 | 0.073 m/s |
| 2 s | 0.20 | ≈ 0.004 | 0.196 | 0.110 | **0.100 m/s**(上限) |

實際上車子會前進，$x_{err}$ 變小，$v_x$ 也跟著變小；到目標附近 $|v_x| < 0.01$ 時會被提高到 0.01 m/s(下限),直到進入 tolerance 並停住 `hold_seconds`。
