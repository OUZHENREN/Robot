import math
from typing import List


def rad_to_deg(value_rad: float) -> float:
    return value_rad * 180.0 / math.pi


def m_to_mm(value_m: float) -> float:
    return value_m * 1000.0


def round_list(values: List[float], digits: int) -> List[float]:
    return [round(v, digits) for v in values]
