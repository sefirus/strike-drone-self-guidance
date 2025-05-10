
def clamp(val: float, lo: float, hi: float) -> float:
    """Clamp *val* to the inclusive range [*lo*, *hi*]."""
    return max(lo, min(val, hi))