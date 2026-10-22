"""
motion_kb — KittenBlock 在线积木的短调用层

为什么需要它
------------
网页版 KittenBlock 在线执行时，会把一条命令按 20 字符切片；**每个非最后分片
都要等板子回显**，等不到就 3 秒超时并 Ctrl-C 中止整条命令。参数块原来的
pycode 有 57~227 字符（3~12 片），一次按键要等十几次回显，所以"设置参数"
特别慢。

把 _ok 回写、_speed/_stride/_lift/_height 同步、失败嘤嘤叫这些**收尾逻辑**
搬到板子上（本模块只随固件存在一份），积木每次只发一次短调用：

    _m.p(70,10,60)      # set_params(stride, lift, height)
    _m.s(6,1)           # set_speed + set_direction
    _m.c(-2)            # set_center
    _m.t(0.5)           # set_turn
    _m.b(5,-5)          # set_body_pose
    _m.g('trot')        # set_turn(0) + set_gait
    _m.go(1,0)          # 方向块: 用全局 _speed/_stride/_lift/_height 起步
    _m.x()              # set_gait('stop')

接线（afterConnect / libs.import 各一次）:
    import motion_kb as _m
    _m.bind(globals())

★ 本模块必须冻结进固件（frozen/manifest.py），不要放 VFS：
  VFS 里的同名 .py 会盖住固件版（见 AGENTS.md 易错点 14）。
"""

import bpuppy_motion as _mm
import bpuppy_imu as _imu
import poses as _ps
import voice as _v

# 主全局 dict（由 bind() 传入）。_ok / _speed / _stride / _lift / _height
# 都住在里面；「设置成功?」积木读的就是这里的 _ok。
_G = None


def bind(g):
    """把主全局 dict 交给本模块。afterConnect / libs.import 各调一次。"""
    global _G
    _G = g


def _done(ok):
    """收尾：写回 _ok；被拒时嘤一声。返回 None，REPL 不会多打一行。"""
    if _G is not None:
        _G['_ok'] = ok
    if not ok:
        _v.say(*_v.SND_YING)


def p(stride, lift, height):
    """set_params(stride, lift, height)。成功才回写三个参数全局。"""
    ok = _mm.set_params(stride, lift, height)
    if ok and _G is not None:
        _G['_stride'] = abs(stride)
        _G['_lift'] = lift
        _G['_height'] = height
    _done(ok)


def s(speed, direction=1):
    """set_direction(direction) + set_speed(speed)。

    两个全局仍按旧 pycode 的语义**无条件**写入（即使固件拒绝速度），
    这样「前进/后退」块读到的 _speed/_dir 与之前一致。
    """
    if _G is not None:
        _G['_speed'] = speed
        _G['_dir'] = direction
    ok1 = _mm.set_direction(direction)
    ok2 = _mm.set_speed(speed)
    _done(ok1 and ok2)


def t(turn):
    """set_turn(turn)。超限时固件会钳位但仍返回 False，沿用旧的嘤声提示。"""
    _done(_mm.set_turn(turn))


def b(pitch, roll):
    """set_body_pose(pitch, roll)。"""
    _done(_mm.set_body_pose(pitch, roll))


def c(offset):
    """set_center(offset)。注意：这个 API 会写 NVS（掉电保存）。"""
    _done(_mm.set_center(offset))


def g(gait):
    """切换步态：先清转弯，再 set_gait。返回值是 None，不做成功判断。"""
    _mm.set_turn(0)
    _mm.set_gait(gait)


def go(direction=1, turn=0):
    """方向块：设方向/转弯/速度/参数，再起步。

    速度与参数从主全局读，和旧 pycode 里
    set_speed(_speed) + set_params(_stride, _lift, _height) 等价；
    bind() 没调用时退回固件默认值，不会 NameError。
    """
    _mm.set_direction(direction)
    _mm.set_turn(turn)
    if _G is not None:
        _mm.set_speed(_G.get('_speed', 2.5))
        _mm.set_params(_G.get('_stride', 70),
                       _G.get('_lift', 30),
                       _G.get('_height', 70))
    _mm.set_gait('go')


def x():
    """停止：set_gait('stop')。"""
    _mm.set_gait('stop')


def get(i):
    """读 get_params() 的第 i 项（0=speed, 1=stride, 2=height, 3=lift,
    5=turn, 7=direction）。"""
    return _mm.get_params()[i]


# ---- 以下为舵机 / 语音 / IMU 积木的短名包装（可选，不改变行为） ----

def sv(ch, deg):
    _ps.set_servo(ch, deg)


def st(n):
    _ps.set_step(n)


def osc(ch, amp, hz, cycles):
    _ps.oscillate(ch, amp, hz, cycles)


def imu():
    _imu.init(0, 14, 21, 0x68)


def w(name):
    _v.say(*(_v.SND_WANG if name == 'WANG' else _v.SND_YING))


def n(num):
    _v.say_num(num)
