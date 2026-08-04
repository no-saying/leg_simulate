#!/usr/bin/env python3
"""实时绘制前进速度、pitch、位置和频谱。

输入来自 controller 标准输出；用于在线看 plot，不参与控制。
"""
import sys, re
from collections import deque
import numpy as np
import matplotlib.pyplot as plt

N = 500          # 时域显示点数
N_fft = 256      # FFT 点数
v_all = deque(maxlen=N_fft)  # 用于频谱的滑动窗口

t_d, v_d, p_d, x_d = deque(maxlen=N), deque(maxlen=N), deque(maxlen=N), deque(maxlen=N)
pos = 0.0  # 累积位移

plt.ion()
fig, (ax_v, ax_p, ax_x, ax_s) = plt.subplots(4, 1, figsize=(10, 10))
fig.canvas.manager.set_window_title('Balance Monitor')

lv, = ax_v.plot([], [], 'b-', lw=1)
lp, = ax_p.plot([], [], 'r-', lw=1)
lx, = ax_x.plot([], [], 'g-', lw=1)
ax_v.set_ylabel('v (rad/s)'); ax_v.set_ylim(-300, 300); ax_v.grid(True, alpha=0.3)
ax_p.set_ylabel('pitch (deg)'); ax_p.set_ylim(-5, 5); ax_p.grid(True, alpha=0.3)
ax_x.set_ylabel('pos (m)'); ax_x.set_ylim(-10, 10); ax_x.grid(True, alpha=0.3)

# 频谱图: 用 bar 显示
ax_s.set_ylabel('|V(f)|'); ax_s.set_xlabel('freq (Hz)')
ax_s.set_xlim(0, 50); ax_s.set_ylim(0, 50)
ax_s.grid(True, alpha=0.3)
bars = ax_s.bar(np.arange(N_fft//2), np.zeros(N_fft//2), width=1.0)

plt.tight_layout()
plt.show(block=False)

pat = re.compile(r'\[t=([\d.]+)\].*?pit=([\-\d.]+)d.*?v=([\-\d.]+)')
# dt: 日志现在基于 sim_time 限频 (~0.05s间隔), 但正则仍能正确提取 v 值
dt = 0.05  # 日志间隔约 0.05s

BATCH = 5   # 每 5 行数据才重绘一次
count = 0

for line in sys.stdin:
    m = pat.search(line)
    if not m:
        continue
    t = float(m.group(1))
    p = float(m.group(2))
    v = float(m.group(3))
    # 位移积分: v(rad/s) * r(0.06m) → m/s, 累加
    pos += v * 0.06 * dt
    t_d.append(t); p_d.append(p); v_d.append(v); x_d.append(pos)
    v_all.append(v)

    count += 1
    if count % BATCH != 0:
        continue  # 攒够 BATCH 行再画

    # 时域
    lv.set_data(list(t_d), list(v_d))
    lp.set_data(list(t_d), list(p_d))
    lx.set_data(list(t_d), list(x_d))
    t_min = max(0, t - 10)
    ax_v.set_xlim(t_min, t + 0.5)
    ax_p.set_xlim(t_min, t + 0.5)
    ax_x.set_xlim(t_min, t + 0.5)

    # 频谱 (每 BATCH*2 次更新)
    if count % (BATCH * 2) == 0 and len(v_all) >= 64:
        win = np.hanning(len(v_all))
        fft = np.abs(np.fft.rfft(np.array(v_all) * win))
        freqs = np.fft.rfftfreq(len(v_all), d=dt)
        # 截断到 50Hz
        mask = freqs <= 50
        freqs = freqs[mask]
        fft = fft[mask]
        # 更新 bar 高度
        for i, (fq, amp) in enumerate(zip(freqs, fft)):
            if i < N_fft//2:
                bars[i].set_height(amp)
                bars[i].set_x(fq)
        ax_s.set_ylim(0, max(10, np.max(fft) * 1.2))

    fig.canvas.draw_idle()
    fig.canvas.flush_events()
