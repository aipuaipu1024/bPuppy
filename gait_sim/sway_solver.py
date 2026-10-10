#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""walk 静态稳定裕量审计 + 前后摆动 (sway) 配平求解器 —— 离线, 独立。

背景: walk 里每条腿抬起时, 支撑三足的那条斜边几乎从机身中心穿过
(四脚站在髋正下方是矩形, 对角线过中心) —— 后腿刚离地时重心到边界只有 ~2mm。
本工具逐相位复刻足端轨迹, 同时算两个量:

  ① 稳定裕量 stab(t): 机身系里 重心 → 支撑多边形(着地脚凸包)边界 的最小距离 (mm)。
     ⚠ 本机构脚**不能左右移动** (ik 只有 (x,z), 足端 y 恒为 ±bw, 见 motion_task.cpp
     的 roll 补偿注释) ⇒ 摆动只能做**前后**方向。
  ② 关节裕量 ang(t): 四腿髋/膝到限位的最小距离 (deg), 公式与 center_spacing_solver
     完全同源; 超可达返回 INVALID。

摆动定义 (与固件一致):
    sway_foot(t) = -A · cos(4π(t - δ))      (t = 周期相位, t=0 是 LH 抬脚; 正向)
即把整幅足端图案前后平移, 等效于把重心沿 +A·cos 方向挪 —— A>0 时后腿抬的瞬间
重心前移、前腿抬的瞬间重心后移。逆走时符号随 direction 取反 (镜像对称, 裕量同值)。

扫 (A, δ) 输出两目标的最坏值 —— 选幅值/相位时"稳不稳"和"髋膝顶不顶"要一起看。

用法:
    python sway_solver.py                          # 默认 S70 H70 L30 几何
    python sway_solver.py -s 70 -H 45 -l 1 --spacing 66
    python sway_solver.py -s 70 -H 70 -l 30 --curve
"""
import argparse
import math

# 腿序与 compute_offsets 的输出一致: [LF, LH, RF, RH]; start = 该腿抬脚相位 (正向)
LEGS = [
    # name, start, front, side(+1=左 / -1=右)  —— 稳定裕量要用真实 y 符号
    ('LF', 0.24, True, +1),
    ('LH', 0.00, False, +1),
    ('RF', 0.74, True, -1),
    ('RH', 0.50, False, -1),
]


def _clamp(v):
    return max(-1.0, min(1.0, v))


def _leg_min_margin(x, z, front, L1, L2, hip_lim, knee_lim):
    """一条腿在足端 (x,z)(相对髋) 处两关节的最小裕量 (deg); 超可达返回 None。"""
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


def _foot(p, S, H, L, duty):
    """腿-local 相位 p → (x_rel, z)。摆动 smoothstep / 支撑线性 (与 foot_trajectory 同源)。"""
    if p < duty:
        u = p / duty
        e = u * u * (3.0 - 2.0 * u)
        return (-S / 2 + S * e, H - L * math.sin(e * math.pi))
    return (S / 2 - S * (p - duty) / (1.0 - duty), H)


def _hull(pts):
    """凸包 (单调链), 返回逆时针序。"""
    pts = sorted(pts)
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lo = []
    for pt in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], pt) <= 0:
            lo.pop()
        lo.append(pt)
    up = []
    for pt in reversed(pts):
        while len(up) >= 2 and cross(up[-2], up[-1], pt) <= 0:
            up.pop()
        up.append(pt)
    return lo[:-1] + up[:-1]


def _poly_margin(poly, com):
    """重心到凸多边形边界的最小有符号距离 (mm), 正 = 在多边形内 (poly 须为 CCW)。"""
    m = 1e9
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        ex, ey = x2 - x1, y2 - y1
        el = math.hypot(ex, ey)
        if el < 1e-9:
            continue
        d = (ex * (com[1] - y1) - ey * (com[0] - x1)) / el  # CCW ⇒ 内侧为正
        m = min(m, d)
    return m


def sample(t, args, A=0.0, delta=0.0):
    """相位 t 的 (stab, ang) 两个裕量; ang 超可达返回 None。"""
    sway = -A * math.cos(4.0 * math.pi * (t - delta))
    grounded = []
    worst_ang = 1e9
    for name, start, front, side in LEGS:
        p = (t - start) % 1.0
        xr, z = _foot(p, args.stride, args.height, args.lift, args.duty)
        sp = args.spacing / 2.0 if front else -args.spacing / 2.0
        x_hip = xr + args.center + sp + sway
        if p >= args.duty:
            bx = args.bl if front else -args.bl
            grounded.append((bx + x_hip, args.bw * side))
        m = _leg_min_margin(x_hip, z, front, args.l1, args.l2, args.hip, args.knee)
        if m is None:
            return None, None
        worst_ang = min(worst_ang, m)
    poly = _hull(grounded)
    stab = _poly_margin(poly, (args.com_x, args.com_y)) if len(poly) >= 3 else 1e9
    return stab, worst_ang


def sweep(args, A, delta, n):
    ws, wa = 1e9, 1e9
    ts, ta = 0.0, 0.0
    for i in range(n):
        t = i / n
        s, a = sample(t, args, A, delta)
        if s is None:
            return None, None, None, None
        if s < ws:
            ws, ts = s, t
        if a < wa:
            wa, ta = a, t
    return ws, wa, ts, ta


def main():
    ap = argparse.ArgumentParser(description='walk stability margin + fore-aft sway solver')
    ap.add_argument('-s', '--stride', type=float, default=70.0)
    ap.add_argument('-H', '--height', type=float, default=70.0)
    ap.add_argument('-l', '--lift', type=float, default=30.0)
    ap.add_argument('--duty', type=float, default=0.20)
    ap.add_argument('--center', type=float, default=0.0, help='中点平移 (mm)')
    ap.add_argument('--spacing', type=float, default=0.0, help='中位点间距 (mm)')
    ap.add_argument('--bl', type=float, default=62.5, help='前后髋半距 (mm)')
    ap.add_argument('--bw', type=float, default=59.0, help='左右髋半宽 (mm)')
    ap.add_argument('--com-x', type=float, default=0.0, help='真实重心前后偏置 (mm, +前)')
    ap.add_argument('--com-y', type=float, default=0.0, help='真实重心左右偏置 (mm)')
    ap.add_argument('--l1', type=float, default=40.0)
    ap.add_argument('--l2', type=float, default=45.0)
    ap.add_argument('--hip', type=float, nargs=2, default=[0.0, 180.0])
    ap.add_argument('--knee', type=float, nargs=2, default=[10.0, 170.0])
    ap.add_argument('--n', type=int, default=2000, help='相位采样数')
    ap.add_argument('--cycle', type=float, default=1.26, help='步态周期 (s), 只用于换算摆动速度/加速度')
    ap.add_argument('--curve', action='store_true', help='打印选定 A/delta 的裕量曲线')
    ap.add_argument('--A', type=float, default=None)
    ap.add_argument('--delta', type=float, default=0.05)
    args = ap.parse_args()

    print('=== walk stability + sway : S=%.0f H=%.0f L=%.0f center=%.0f spacing=%.0f ==='
          % (args.stride, args.height, args.lift, args.center, args.spacing))
    print('geometry: L1=%.1f L2=%.1f bl=%.1f bw=%.1f  com=(%+.0f, %+.0f)  limits: hip[%.0f,%.0f] knee[%.0f,%.0f]'
          % (args.l1, args.l2, args.bl, args.bw, args.com_x, args.com_y,
             args.hip[0], args.hip[1], args.knee[0], args.knee[1]))

    # 基线 (无摆动) + 四个抬脚瞬间
    ws, wa, ts, ta = sweep(args, 0.0, 0.0, args.n)
    if ws is None:
        print('!! 轨迹本身超可达 (先查 S/H/lift/spacing)')
        return
    print('baseline (A=0): min stab %.2fmm @t=%.3f | min angle %.1f deg @t=%.3f' % (ws, ts, wa, ta))
    for start, nm in ((0.0, 'LH lift'), (0.24, 'LF lift'), (0.50, 'RH lift'), (0.74, 'RF lift')):
        s, a = sample(start, args)
        print('   t=%.2f %-8s: stab %6.2f mm   ang %6.1f deg' % (start, nm, s, a))

    if args.A is not None:
        A, delta = args.A, args.delta
        ws, wa, ts, ta = sweep(args, A, delta, args.n)
        if ws is None:
            print('A=%.1f delta=%.3f -> INVALID (超可达)' % (A, delta))
            return
        print('A=%.1f delta=%.3f: min stab %.2fmm @t=%.3f | min angle %.1f deg @t=%.3f'
              % (A, delta, ws, ts, wa, ta))
        if args.curve:
            print('  curve (t: stab / ang):')
            for i in range(0, 100, 2):
                t = i / 100.0
                s, a = sample(t, args, A, delta)
                print('    t=%.2f  stab %7.2f  ang %6.1f' % (t, s, a))
            print('  baseline curve:')
            for i in range(0, 20):
                t = i / 20.0
                s, a = sample(t, args, 0.0, 0.0)
                print('    t=%.2f  stab %7.2f  ang %6.1f' % (t, s, a))
        return

    # 网格: A x delta —— 同时看两个目标
    As = [4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 20.0, 24.0, 28.0, 32.0]
    deltas = [0.0, 0.025, 0.05, 0.075, 0.10, 0.125, 0.15]
    print('\n  A\\delta ' + ''.join('%9.3f' % d for d in deltas))
    for A in As:
        row = []
        for d in deltas:
            ws_t, wa_t, _, _ = sweep(args, A, d, args.n)
            if ws_t is None:
                row.append('  INVALID')
                continue
            row.append('%4.1f/%4.0f' % (ws_t, wa_t))
        print('  %5.1f  %s' % (A, ''.join('%9s' % c for c in row)))
    print('  (格内 = min stab mm / min angle deg)')

    # 按"关节裕量不跌破基线-3度"的约束找最佳稳定裕量
    print('\n  best under ang>=%.1f deg:' % (wa - 3.0))
    floor = wa - 3.0
    best = None
    for A in [x * 0.5 for x in range(0, 61)]:
        for d in [x * 0.005 for x in range(0, 41)]:
            ws2, wa2, ts2, ta2 = sweep(args, A, d, 400)
            if ws2 is None or wa2 is None or wa2 < floor:
                continue
            if best is None or ws2 > best[0]:
                best = (ws2, wa2, A, d, ts2, ta2)
    if not best:
        print('    none')
        return
    A, d = best[2], best[3]
    print('    A=%.1f  delta=%.3f  -> min stab %.2fmm @t=%.3f | min ang %.1f deg @t=%.3f'
          % (A, d, best[0], best[4], best[1], best[5]))
    # 摆动本身的运动学 (供手感/动力学判断): v = 4πA/T, a = (4π)²A/T²
    T = args.cycle
    print('    sway kinematics: v_max %.0f mm/s, a_max %.0f mm/s^2 (%.2f g)  [cycle=%.2fs]'
          % (4 * math.pi * A / T, (4 * math.pi) ** 2 * A / T ** 2,
             (4 * math.pi) ** 2 * A / T ** 2 / 9800.0, T))
    print('    four critical instants with this sway:')
    for t, nm in ((0.0, 'LH lift'), (0.44, 'LF land'), (0.50, 'RH lift'), (0.94, 'RF land')):
        s, a = sample(t, args, A, d)
        print('      t=%.2f %-8s: stab %6.2f mm   ang %6.1f deg   (baseline %5.2f mm)'
              % (t, nm, s, a, sample(t, args)[0]))


if __name__ == '__main__':
    main()
