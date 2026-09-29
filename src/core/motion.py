"""差速混合:把语义化指令 (vx, wz) 换算为左右轮占空比。

vx: 线速度分量 [-1, 1],正=前进;wz: 角速度分量 [-1, 1],正=左转。
混合公式:left = vx - wz,right = vx + wz
  wz > 0 时左轮慢、右轮快,车身向左转;vx 与 wz 同号叠加时单边会超过上限,
  此时按峰值等比缩小(而非分别截断),保持转向比例不变。
"""

MAX_DUTY = 0.8  # TT 电机额定 3-6V,7.4V 直驱的占空比上限(见《硬件选型与BOM.md》注意事项 3)


def _clamp(value):
    return max(-1.0, min(1.0, float(value)))


def mix(vx, wz):
    """(vx, wz) → (left, right),返回值均在 [-MAX_DUTY, MAX_DUTY]。"""
    vx = _clamp(vx)
    wz = _clamp(wz)
    left = vx - wz
    right = vx + wz
    peak = max(abs(left), abs(right))
    if peak > MAX_DUTY:
        left = left / peak * MAX_DUTY
        right = right / peak * MAX_DUTY
    return left, right
