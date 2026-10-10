#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""中位点间距 (center_spacing) 配平求解器 —— 离线, 独立于 stop_audit 内核。

口径与固件完全同源 (ik.c + motion_task.cpp 的 traj_combo_bad / ik_pos_check):
  - 髋角 = α ± β (前腿 +, 后腿 -);  α = atan2(z, x);  β = acos((L1²+d²-L2²)/(2·L1·d))
  - 膝角 = acos((L1²+L2²-d²)/(2·L1·L2))  (0=折叠 180=伸直; 左右/后腿镜像不影响裕量)
  - 可达: d ≤ L1+L2-1 (超了就是"够不着", 该间距非法)
细采样整条足端轨迹 (摆动 + 支撑), 对候选间距 s 取**全轨迹四腿最小角度裕量**,
输出最大化它的 s* —— 即"等裕量/最大最小余量"配平点。裕量与 0/180 限位比较, 单位 deg。

s 的语义与固件一致: 前腿中位点 = 中点 + s/2, 后腿 = 中点 - s/2。
  s > 0 展开, s < 0 收窄 (大步长低站高时最优方向可能翻转, 本工具会如实报负值)。

用法:
    python center_spacing_solver.py                        # 默认 S70 H70 L30
    python center_spacing_solver.py -s 70 -H 45 -l 1
    python center_spacing_solver.py -s 106 -H 50 -l 30
    python center_spacing_solver.py -s 70 -H 45 -l 30 --center 0
"""
import argparse
import math


def _clamp(v):
    return max(-1.0, min(1.0, v))


def _leg_min_margin(x, z, front, L1, L2, hip_lim, knee_lim):
    """一条腿在足端 (x,z) 处两关节的最小裕量 (deg); 超可达返回 None。"""
    d = math.hypot(x, z)
    if d > L1 + L2 - 1.0:
        return None
    ck = _clamp((L1 * L1 + L2 * L2 - d * d) / (2.0 * L1 * L2))
    k = math.degrees(math.acos(ck))
    mk = min(k - knee_lim[0], knee_lim[1] - k)
    cb = _clamp((L1 * L1 + d * d - L2 * L2) / (2.0 * L1 * d))
    beta = math.degrees(math.acos(cb))
    alpha = math.degrees(math.atan2(z, x))
    hip = alpha + beta if front else alpha - beta
    mh = min(hip - hip_lim[0], hip_lim[1] - hip)
    return min(mh, mk)


def min_margin(spacing, args, n=400):
    """整条轨迹上四腿的最小裕量; 任一点超可达返回 None (该间距非法)。"""
    half = spacing * 0.5
    worst = 1e9
    for i in range(n):
        p = i / n
        if p < args.duty:
            t = p / args.duty
            ease = t * t * (3 - 2 * t)
            x0 = -args.stride / 2 + args.stride * ease
            z = args.height - args.lift * math.sin(ease * math.pi)
        else:
            x0 = args.stride / 2 - args.stride * (p - args.duty) / (1 - args.duty)
            z = args.height
        for off, front in ((half, True), (-half, False)):
            m = _leg_min_margin(x0 + args.center + off, z, front,
                                args.l1, args.l2, args.hip, args.knee)
            if m is None:
                return None
            worst = min(worst, m)
    return worst


def main():
    ap = argparse.ArgumentParser(description='center_spacing balance solver')
    ap.add_argument('-s', '--stride', type=float, default=70.0)
    ap.add_argument('-H', '--height', type=float, default=70.0)
    ap.add_argument('-l', '--lift', type=float, default=30.0)
    ap.add_argument('--center', type=float, default=0.0, help='中点平移 (mm)')
    ap.add_argument('--bl', type=float, default=62.5, help='前后髋半距 (mm)')
    ap.add_argument('--l1', type=float, default=40.0)
    ap.add_argument('--l2', type=float, default=45.0)
    ap.add_argument('--hip', type=float, nargs=2, default=[0.0, 180.0])
    ap.add_argument('--knee', type=float, nargs=2, default=[10.0, 170.0])
    ap.add_argument('--duty', type=float, default=0.20)
    args = ap.parse_args()

    smax = 2.0 * args.bl            # 固件量级闸门 ±2×前后髋半距
    print('=== center_spacing balance : S=%.0f H=%.0f L=%.0f center=%.0f ==='
          % (args.stride, args.height, args.lift, args.center))
    print('geometry: L1=%.1f L2=%.1f bl=%.1f  limits: hip[%.0f,%.0f] knee[%.0f,%.0f]'
          % (args.l1, args.l2, args.bl, args.hip[0], args.hip[1],
             args.knee[0], args.knee[1]))

    m0 = min_margin(0.0, args)
    print('  margin @ s=0     : %s' %
          ('%.1f deg' % m0 if m0 is not None else 'INVALID (unreachable)'))

    # 粗扫 1mm -> 细扫 0.1mm
    best_s, best_m = None, -1e9
    s = -smax
    while s <= smax + 1e-9:
        m = min_margin(s, args)
        if m is not None and m > best_m:
            best_m, best_s = m, s
        s += 1.0
    if best_s is not None:
        s = best_s - 1.0
        while s <= best_s + 1.0:
            m = min_margin(s, args)
            if m is not None and m > best_m:
                best_m, best_s = m, s
            s += 0.1

    if best_s is None:
        print('  NO valid spacing in [%.0f, %.0f] -- 先调整 S/H/lift 组合' % (-smax, smax))
        return
    if abs(best_s) > smax - 1.0:
        # 最优贴在闸门边上, 说明还想更远但被 ±2bl 卡住
        edge = '  (at +/-2*bl gate!)'
    else:
        edge = ''
    print('  BEST spacing s*  : %+.1f mm  ->  min margin %.1f deg%s'
          % (best_s, best_m, edge))
    print('  per-leg offsets  : front %+.1f  rear %+.1f'
          % (best_s / 2.0, -best_s / 2.0))
    if best_s < 0:
        print('  note: optimal direction is CONTRACT (negative spacing)')

    print('  curve (s -> min margin):')
    row = []
    for s in range(-100, 101, 10):
        if abs(s) > smax:
            continue
        m = min_margin(float(s), args)
        row.append('%d:%s' % (s, '%.1f' % m if m is not None else 'N/A'))
    print('    ' + '  '.join(row))


if __name__ == '__main__':
    main()
