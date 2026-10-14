#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""turn 穿越 ±0.5 的方向翻转审计 —— 复刻 motion_task.cpp 的单腿腿循环, 离线扫描

背景: turn 让一侧腿按 s = 1-2|turn| 缩放。跨 0.5 时该腿 leg_stride 变号 →
相位表换套 (walk) + x 取反, 现状是**当帧生效**, 没有保护。

三组对照:
  old  —— 现状: direction = sign(s), 当帧翻转, 无保护
  bad  —— 挂起 + 支撑窗口, 但翻转插在**边沿锁存之前** (验证顺序约束)
  new  —— 挂起 + 支撑窗口, 翻转在边沿锁存之后 (拟实施)

指标: 全部起始相位 × 多场景, 右前腿足端 x 的**最大单帧位移** (mm)。
参照: 正常行走 (turn 恒定) 本身的单帧位移。
翻转时机用**右腿自身的相位**(相对它自己的摆动起点的位置)记录, 供窗口验证。
"""
import math

DUTY, GAP = 0.20, 0.04
STRIDE = 70.0
DPHI = 0.01592            # speed 2.5, omega 2.0, 20ms/帧 → 每帧走过的步态周期数
WARM = 100                # 预热帧数 (turn=0.4 自洽)


def compute_offsets(direction):
    if direction > 0:
        st = {'LF': DUTY + GAP, 'LH': 0.0, 'RF': 0.5 + DUTY + GAP, 'RH': 0.5}
    else:
        st = {'LF': 0.0, 'LH': DUTY + GAP, 'RF': 0.5, 'RH': 0.5 + DUTY + GAP}
    out = {}
    for k, v in st.items():
        if v >= 1.0:
            v -= 1.0
        out[k] = (1.0 - v) if v > 0.001 else 0.0
    return out


OFF_F = compute_offsets(+1)['RF']    # RF 前进表偏移
OFF_R = compute_offsets(-1)['RF']    # RF 后退表偏移


def ease(t):
    return t * t * (3.0 - 2.0 * t)


def traj_x(pn, abs_s, direction, x_lift, b, b_land, x_land):
    if pn < DUTY:
        x = x_lift + (abs_s * 0.5 - x_lift) * ease(pn / DUTY)
    else:
        x = x_land - (b - b_land)
    return -x if direction < 0 else x


def run(turn_seq, phase0, mech):
    """右前腿 (RF) 逐帧复刻。返回 (xs, flip_marks); flip_marks = 翻转发生时腿自身的 pn"""
    b = b_land = x_land = x_lift = 0.0
    swing = False
    leg_dir = 1
    phase = phase0
    xs, flips = [], []

    # ---- 预热 (turn=0.4, 让锚定量自洽) ----
    for _ in range(WARM):
        turn = 0.4
        s = 1.0 - 2.0 * turn
        abs_s = abs(s) * STRIDE
        pn = (phase + OFF_F) % 1.0          # 预热期 turn<0.5, 用前进表
        in_sw = pn < DUTY
        if in_sw and not swing:
            b_at = b - (abs_s / (1 - DUTY)) * pn
            x_lift = x_land - (b_at - b_land)
        elif (not in_sw) and swing:
            over = (pn - DUTY) / (1 - DUTY)
            b_land = b - abs_s * over
            x_land = abs_s * 0.5
        swing = in_sw
        xs.append(traj_x(pn, abs_s, +1, x_lift, b, b_land, x_land))
        b += (abs_s / (1 - DUTY)) * DPHI
        phase = (phase + DPHI) % 1.0

    # ---- 扫描段 ----
    for turn in turn_seq:
        s = 1.0 - 2.0 * turn
        abs_s = abs(s) * STRIDE
        target = 1 if s >= 0.0 else -1

        if mech == 'old':
            direction = target
            pn = (phase + (OFF_R if direction < 0 else OFF_F)) % 1.0
            in_sw = pn < DUTY
            if in_sw and not swing:
                b_at = b - (abs_s / (1 - DUTY)) * pn
                x_lift = x_land - (b_at - b_land)
            elif (not in_sw) and swing:
                over = (pn - DUTY) / (1 - DUTY)
                b_land = b - abs_s * over
                x_land = abs_s * 0.5
            swing = in_sw
            x = traj_x(pn, abs_s, direction, x_lift, b, b_land, x_land)
            b += (abs_s / (1 - DUTY)) * DPHI

        else:
            direction = leg_dir
            pn = (phase + (OFF_R if direction < 0 else OFF_F)) % 1.0

            if mech == 'bad':
                # 顺序反: 先翻, 后边沿 (锁存会用翻后的 pn)
                if target != leg_dir:
                    dn = -leg_dir
                    pn_n = (phase + (OFF_R if dn < 0 else OFF_F)) % 1.0
                    if pn >= DUTY and pn_n >= DUTY:
                        leg_dir = dn
                        b, b_land, x_land, x_lift = -b, -b_land, -x_land, -x_lift
                        direction, pn = dn, pn_n
                        flips.append(pn_n)
                # 边沿 (用最终 pn; 若刚翻且 pn 跨过边界, 锁存写 +S/2 覆盖取反 ⇒ 跳)
                in_sw = pn < DUTY
                if in_sw and not swing:
                    b_at = b - (abs_s / (1 - DUTY)) * pn
                    x_lift = x_land - (b_at - b_land)
                elif (not in_sw) and swing:
                    over = (pn - DUTY) / (1 - DUTY)
                    b_land = b - abs_s * over
                    x_land = abs_s * 0.5
                swing = in_sw

            else:  # new —— 边沿先用**旧表 pn**, 翻转在后
                in_sw = pn < DUTY
                if in_sw and not swing:
                    b_at = b - (abs_s / (1 - DUTY)) * pn
                    x_lift = x_land - (b_at - b_land)
                elif (not in_sw) and swing:
                    over = (pn - DUTY) / (1 - DUTY)
                    b_land = b - abs_s * over
                    x_land = abs_s * 0.5
                swing = in_sw
                # 翻转判定 (窗口: 切换前后都在支撑相)
                if target != leg_dir:
                    dn = -leg_dir
                    pn_n = (phase + (OFF_R if dn < 0 else OFF_F)) % 1.0
                    if pn >= DUTY and pn_n >= DUTY:
                        leg_dir = dn
                        b, b_land, x_land, x_lift = -b, -b_land, -x_land, -x_lift
                        direction, pn = dn, pn_n
                        flips.append(pn_n)

            x = traj_x(pn, abs_s, direction, x_lift, b, b_land, x_land)
            b += (abs_s / (1 - DUTY)) * DPHI

        xs.append(x)
        phase = (phase + DPHI) % 1.0

    return xs, flips


def max_jump(xs):
    return max(abs(xs[i + 1] - xs[i]) for i in range(len(xs) - 1)) if len(xs) > 1 else 0.0


def scan(mech, label):
    # 场景序列 (turn_seq 即 g_ramp_turn 的目标序列; A 阶段限速后实参 ≤0.04/帧)
    slow   = [0.4 + 0.002 * i for i in range(101)] + [0.6] * 80     # 慢扫
    fast   = [0.4 + 0.04 * i for i in range(6)] + [0.6] * 80        # 0.04/帧 (A 限速极限)
    jitter = [0.5 + 0.03 * math.sin(2 * math.pi * i / 60.0) for i in range(240)]
    back   = [0.6 - 0.04 * i for i in range(6)] + [0.4] * 80        # 反向穿越 (1→0 方向)

    worst = 0.0
    worst_where = ''
    flip_pos = []
    for name, seq in (('slow', slow), ('fast', fast), ('jitter', jitter), ('back', back)):
        for k in range(240):
            phase0 = k / 240.0
            xs, flips = run(seq, phase0, mech)
            j = max_jump(xs[WARM:])       # 只看扫描段
            if j > worst:
                worst, worst_where = j, '%s phase0=%.3f' % (name, phase0)
            flip_pos += flips
    # 翻转时机分布: 腿自身 pn < duty 的比例 (摆动中翻转 = 坏)
    in_swing_frac = (sum(1 for p in flip_pos if p < DUTY) / len(flip_pos)) if flip_pos else 0.0
    print('%-4s %-14s max_jump=%6.2f mm  (%s)   翻转 %d 次, 其中摆动中 %.0f%%'
          % (label, '', worst, worst_where, len(flip_pos), in_swing_frac * 100))
    return worst


print('=== turn 穿越 ±0.5 单腿方向翻转审计 (RF 腿, walk duty=0.2 stride=70) ===')
print('参照: 正常行走 (turn 恒定 0.4) 单帧位移 3~4mm (现有拐角台阶)')
print()
ref = []
for k in range(240):
    xs, _ = run([0.4] * 200, k / 240.0, 'old')
    ref.append(max_jump(xs))
print('参照 正常行走          max_jump=%6.2f mm' % max(ref))
print()
scan('old', 'old')
scan('bad', 'bad')
scan('new', 'new')
