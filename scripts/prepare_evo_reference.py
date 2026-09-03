#!/usr/bin/env python3
"""Convert timestamped poses to evo's TUM format.

The MCD mapping poses are stored as:
  timestamp tx ty tz qx qy qz qw

Use --z-up when the Block-Map was generated with generate_bms --z-up.  This
left-multiplies every world pose by Rx(pi), matching the map frame used by the
localizer.
"""

import argparse
import math
from pathlib import Path
from typing import Iterable, Tuple


Quaternion = Tuple[float, float, float, float]  # x, y, z, w
Vector3 = Tuple[float, float, float]


def quaternion_multiply(lhs: Quaternion, rhs: Quaternion) -> Quaternion:
    lx, ly, lz, lw = lhs
    rx, ry, rz, rw = rhs
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def normalize(quaternion: Quaternion) -> Quaternion:
    norm = math.sqrt(sum(value * value for value in quaternion))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("invalid zero or non-finite quaternion")
    return tuple(value / norm for value in quaternion)  # type: ignore[return-value]


def rotate(vector: Vector3, quaternion: Quaternion) -> Vector3:
    x, y, z = vector
    qx, qy, qz, qw = quaternion
    # q * [v, 0] * conjugate(q), expanded to avoid temporary quaternions.
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + qy * tz - qz * ty,
        y + qw * ty + qz * tx - qx * tz,
        z + qw * tz + qx * ty - qy * tx,
    )


def parse_pose(fields: Iterable[str], pose_order: str) -> Tuple[float, Vector3, Quaternion]:
    values = [float(field) for field in fields]
    if len(values) != 8:
        raise ValueError(f"expected 8 fields, found {len(values)}")
    timestamp, tx, ty, tz, q0, q1, q2, q3 = values
    if not all(math.isfinite(value) for value in values):
        raise ValueError("pose contains a non-finite value")
    quaternion = (q0, q1, q2, q3) if pose_order == "xyzw" else (q1, q2, q3, q0)
    return timestamp, (tx, ty, tz), normalize(quaternion)


def main() -> int:
    parser = argparse.ArgumentParser(description="Prepare a reference trajectory for evo")
    parser.add_argument("input", type=Path, help="input timestamped pose file")
    parser.add_argument("output", type=Path, help="output TUM trajectory")
    parser.add_argument(
        "--pose-order",
        choices=("xyzw", "wxyz"),
        default="xyzw",
        help="quaternion field order in the input (default: xyzw)",
    )
    parser.add_argument(
        "--z-up",
        action="store_true",
        help="left-multiply poses by Rx(pi) for maps generated with --z-up",
    )
    args = parser.parse_args()

    world_rotation: Quaternion = (1.0, 0.0, 0.0, 0.0) if args.z_up else (0.0, 0.0, 0.0, 1.0)
    output_lines = []
    previous_timestamp = -math.inf
    previous_quaternion = None

    with args.input.open("r", encoding="utf-8") as stream:
        for line_number, raw_line in enumerate(stream, 1):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            try:
                timestamp, translation, quaternion = parse_pose(
                    line.split(), args.pose_order
                )
            except ValueError as error:
                parser.error(f"{args.input}:{line_number}: {error}")
            if timestamp <= previous_timestamp:
                parser.error(
                    f"{args.input}:{line_number}: timestamps are not strictly increasing"
                )
            previous_timestamp = timestamp

            translation = rotate(translation, world_rotation)
            quaternion = normalize(quaternion_multiply(world_rotation, quaternion))
            if previous_quaternion is None and quaternion[3] < 0.0:
                quaternion = tuple(-value for value in quaternion)  # type: ignore[assignment]
            elif previous_quaternion is not None:
                dot = sum(a * b for a, b in zip(previous_quaternion, quaternion))
                if dot < 0.0:
                    quaternion = tuple(-value for value in quaternion)  # type: ignore[assignment]
            previous_quaternion = quaternion

            output_lines.append(
                "{:.9f} {:.9f} {:.9f} {:.9f} {:.9f} {:.9f} {:.9f} {:.9f}\n".format(
                    timestamp, *translation, *quaternion
                )
            )

    if not output_lines:
        parser.error(f"no poses found in {args.input}")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("".join(output_lines), encoding="utf-8")
    print(f"Wrote {len(output_lines)} TUM poses to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
