from __future__ import annotations

from typing import Dict, List, Tuple

from src.models import CarPose, Cone


_SCENARIOS: Dict[str, Tuple[List[Cone], CarPose]] = {
    "1": (
        [],
        CarPose(x=0.0, y=0.0, yaw=0.0),
    ),

    "2": (
        [Cone(x=1.0, y=3.0, color=1), Cone(x=1.0, y=1.0, color=0)],
        CarPose(x=0.0, y=0.0, yaw=0.4),
    ),

    "3": (
        [
            Cone(x=1.0, y=3.0, color=1),
            Cone(x=3.0, y=3.0, color=1),
            Cone(x=1.0, y=1.0, color=0),
            Cone(x=3.0, y=1.0, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=0.8),
    ),

    "4": (
        [Cone(x=4.0, y=2.0, color=0)],
        CarPose(x=0.0, y=0.0, yaw=1.2),
    ),

    "5": (
        [Cone(x=4.0, y=2.0, color=0), Cone(x=3.0, y=2.0, color=0)],
        CarPose(x=0.0, y=0.0, yaw=1.6),
    ),

    "6": (
        [
            Cone(x=4.0, y=4.0, color=1),
            Cone(x=4.0, y=2.0, color=0),
            Cone(x=3.0, y=2.0, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=2.0),
    ),

    "7": (
        [
            Cone(x=2.0, y=3.0, color=1),
            Cone(x=4.0, y=3.0, color=1),
            Cone(x=2.0, y=2.0, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=1.4),
    ),

    "8": (
        [Cone(x=3.0, y=3.0, color=1), Cone(x=5.0, y=3.0, color=1)],
        CarPose(x=0.0, y=0.0, yaw=1.8),
    ),

    "9": (
        [Cone(x=3.0, y=3.0, color=1)],
        CarPose(x=0.0, y=0.0, yaw=0.2),
    ),

    "10": (
        [Cone(x=5.0, y=3.0, color=1), Cone(x=5.0, y=2.0, color=0)],
        CarPose(x=0.0, y=0.0, yaw=0.6),
    ),

    "11": (
        [
            Cone(x=1.0, y=3.0, color=1),
            Cone(x=4.0, y=5.0, color=1),
            Cone(x=1.0, y=2.0, color=0),
            Cone(x=4.0, y=2.0, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=1.0),
    ),

    "12": (
        [Cone(x=5.0, y=2.0, color=0)],
        CarPose(x=0.0, y=0.0, yaw=1.4),
    ),

    "13": (
        [Cone(x=5.0, y=3.0, color=0), Cone(x=5.0, y=1.0, color=0)],
        CarPose(x=0.0, y=0.0, yaw=0.1),
    ),

    "14": (
        [
            Cone(x=3.0, y=5.0, color=1),
            Cone(x=3.0, y=2.0, color=0),
            Cone(x=5.0, y=2.0, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=5.2),
    ),

    "15": (
        [
            Cone(x=2.0, y=5.0, color=1),
            Cone(x=3.0, y=4.0, color=1),
            Cone(x=3.0, y=2.0, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=1.6),
    ),

    "16": (
        [Cone(x=0.0, y=3.0, color=1), Cone(x=2.0, y=5.0, color=1)],
        CarPose(x=0.0, y=0.0, yaw=0.2),
    ),

    "17": (
        [Cone(x=3.0, y=5.0, color=1)],
        CarPose(x=0.0, y=0.0, yaw=0.6),
    ),

    "18": (
        [Cone(x=2.0, y=4.0, color=1), Cone(x=4.0, y=3.0, color=0)],
        CarPose(x=0.0, y=0.0, yaw=1.0),
    ),

    "19": (
        [
            Cone(x=2.0, y=3.0, color=1),
            Cone(x=5.0, y=3.0, color=1),
            Cone(x=2.0, y=0.0, color=0),
            Cone(x=5.0, y=2.0, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=1.4),
    ),

    "20": (
        [Cone(x=0.0, y=2.0, color=0)],
        CarPose(x=0.0, y=0.0, yaw=1.8),
    ),
        
        # 21: three yellow cones, no blue cones
    "21": (
        [
            Cone(x=1.0, y=4.0, color=1),
            Cone(x=3.0, y=4.5, color=1),
            Cone(x=5.0, y=5.0, color=1),
        ],
        CarPose(x=0.0, y=0.0, yaw=0.6),
    ),

    # 22: three blue cones, no yellow cones
    "22": (
        [
            Cone(x=1.0, y=1.0, color=0),
            Cone(x=3.0, y=1.5, color=0),
            Cone(x=5.0, y=2.0, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=1.2),
    ),

    # 23: three yellow cones, forming a curve
    "23": (
        [
            Cone(x=2.0, y=4.0, color=1),
            Cone(x=3.0, y=5.0, color=1),
            Cone(x=4.5, y=5.5, color=1),
        ],
        CarPose(x=0.0, y=0.0, yaw=1.8),
    ),

    # 24: three blue cones, forming a curve
    "24": (
        [
            Cone(x=2.0, y=1.0, color=0),
            Cone(x=3.5, y=1.5, color=0),
            Cone(x=5.0, y=2.5, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=2.4),
    ),

    # 25: three cones on one side, and one on the other
    "25": (
        [
            Cone(x=1.0, y=3.5, color=1),
            Cone(x=3.0, y=4.0, color=1),
            Cone(x=5.0, y=4.5, color=1),
            Cone(x=3.0, y=2.0, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=0.4),
    ),

    # 26: sharp S-curve alternating cones
    "26": (
        [
            Cone(x=1.0, y=2.0, color=1),
            Cone(x=2.0, y=3.5, color=0),
            Cone(x=3.5, y=4.0, color=1),
            Cone(x=5.0, y=3.0, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=1.0),
    ),

    # 27: tight left-hand curve of mixed cones
    "27": (
        [
            Cone(x=1.0, y=4.0, color=1),
            Cone(x=2.5, y=4.8, color=1),
            Cone(x=4.0, y=5.0, color=1),
            Cone(x=3.0, y=3.0, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=1.6),
    ),

    # 28: complex double arc, narrow spacing
    "28": (
        [
            Cone(x=1.0, y=1.5, color=0),
            Cone(x=2.0, y=2.5, color=1),
            Cone(x=3.0, y=3.0, color=0),
            Cone(x=4.5, y=4.0, color=1),
        ],
        CarPose(x=0.0, y=0.0, yaw=2.0),
    ),

    # 29: reverse U-curve, car facing opposite to entry
    "29": (
        [
            Cone(x=1.0, y=4.0, color=0),
            Cone(x=2.0, y=5.0, color=0),
            Cone(x=3.0, y=5.2, color=1),
            Cone(x=4.0, y=4.5, color=1),
        ],
        CarPose(x=0.5, y=1.0, yaw=3.1),
    ),

    # 30: steep incline curve with blue cones only
    "30": (
        [
            Cone(x=2.0, y=1.0, color=0),
            Cone(x=3.0, y=2.0, color=0),
            Cone(x=4.5, y=3.5, color=0),
            Cone(x=6.0, y=5.0, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=2.6),
    ),

    # 31: mixed spiral curve
    "31": (
        [
            Cone(x=1.0, y=2.0, color=1),
            Cone(x=2.0, y=3.0, color=0),
            Cone(x=3.0, y=4.0, color=1),
            Cone(x=4.0, y=4.5, color=0),
            Cone(x=5.0, y=4.7, color=1),
        ],
        CarPose(x=0.0, y=0.0, yaw=1.8),
    ),

    # 32: long tight chicane
    "32": (
        [
            Cone(x=1.0, y=1.0, color=0),
            Cone(x=2.0, y=2.5, color=1),
            Cone(x=3.0, y=1.5, color=0),
            Cone(x=4.0, y=3.0, color=1),
            Cone(x=5.0, y=2.0, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=1.2),
    ),

    # 33: semi-circle curve
    "33": (
        [
            Cone(x=2.0, y=2.0, color=1),
            Cone(x=3.0, y=3.0, color=1),
            Cone(x=4.0, y=3.5, color=1),
            Cone(x=5.0, y=3.0, color=0),
            Cone(x=6.0, y=2.0, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=1.5),
    ),

    # 34: wide curve with unbalanced spacing
    "34": (
        [
            Cone(x=1.0, y=3.0, color=1),
            Cone(x=2.0, y=4.0, color=1),
            Cone(x=4.5, y=5.0, color=1),
            Cone(x=2.5, y=2.0, color=0),
            Cone(x=5.5, y=3.0, color=0),
        ],
        CarPose(x=0.0, y=0.0, yaw=2.8),
    ),

    # 35: closed loop-like path
    "35": (
        [
            Cone(x=1.0, y=2.0, color=1),
            Cone(x=2.0, y=3.5, color=0),
            Cone(x=3.0, y=4.0, color=1),
            Cone(x=4.0, y=3.0, color=0),
            Cone(x=3.5, y=1.5, color=1),
        ],
        CarPose(x=5.0, y=1.0, yaw=3.4),
    ),
}


def get_scenario_names() -> List[str]:
    return sorted(_SCENARIOS.keys(), key=lambda s: int(s))


def make_scenario(name: str) -> Tuple[List[Cone], CarPose]:
    if name not in _SCENARIOS:
        valid = ", ".join(get_scenario_names())
        raise ValueError(f"Unknown scenario '{name}'. Valid options: {valid}")
    cones, car = _SCENARIOS[name]
    return list(cones), CarPose(x=car.x, y=car.y, yaw=car.yaw)
