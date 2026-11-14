"""
Adaptive cruise control (pure math).   Lecture 5b.

Constant time-gap policy: the desired distance grows with speed,
    s_des = s0 + h v                       (s0 standstill gap, h time gap)
Two controllers, the most cautious wins:
    cruise:  a_cc  = k_v (v_set - v)
    follow:  a_acc = k_s (s - s_des) + k_dv (v_lead - v)
    a = clamp(min(a_cc, a_acc), a_min, a_max)
Pedals (static map): a > 0 -> throttle = a / a_throttle_full,  a < 0 -> brake = -a / a_brake_full
"""


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def acc_acceleration(v, v_set, gap, v_lead, p):
    """gap = None when there is no vehicle ahead."""
    a = p['k_v'] * (v_set - v)
    if gap is not None:
        s_des = p['s0'] + p['time_gap'] * v
        a = min(a, p['k_s'] * (gap - s_des) + p['k_dv'] * (v_lead - v))
    return clamp(a, p['a_min'], p['a_max'])


def pedals(a, p):
    """Acceleration request -> (throttle, brake) in [0, 1]."""
    if a >= 0.0:
        return clamp(a / p['a_throttle_full'], 0.0, 1.0), 0.0
    return 0.0, clamp(-a / p['a_brake_full'], 0.0, 1.0)
