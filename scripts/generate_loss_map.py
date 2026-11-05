"""Generate a synthetic motor + inverter loss map (CSV) from a physical loss model.

Real projects get this map from a motor test bench. We do not have one, so
the map is generated from a standard loss decomposition for a permanent-magnet
synchronous machine (PMSM) with inverter:

    P_loss = k_cu * T^2          copper losses (current ~ torque, I^2 R)
           + k_inv * |T|         inverter conduction/switching (~ current)
           + k_fe * w^1.5        iron losses (hysteresis + eddy currents)
           + k_fr * w            bearing friction and windage

T in Nm, w in rad/s, P in W. Coefficients are chosen so that the peak
efficiency is about 96 % and part-load efficiency at high speed drops to
85-90 %, which is the typical shape of published PMSM maps.
The simulator only reads the CSV, so a measured map can replace this file.

Usage:  python scripts/generate_loss_map.py configs/maps/generic_pmsm_loss.csv
"""

import sys
from pathlib import Path

import numpy as np

K_CU = 0.0624  # W/Nm^2  -> 6.0 kW at 310 Nm
K_INV = 2.0  # W/Nm
K_FE = 0.0722  # W/(rad/s)^1.5  -> 3.0 kW at 1200 rad/s
K_FR = 0.417  # W/(rad/s)  -> 0.5 kW at 1200 rad/s

SPEED_RPM = np.arange(0.0, 12000.0 + 1, 500.0)
TORQUE_NM = np.arange(0.0, 320.0 + 1, 20.0)


def loss_w(speed_rpm, torque_nm):
    w = speed_rpm * 2 * np.pi / 60
    t = np.abs(torque_nm)
    return K_CU * t**2 + K_INV * t + K_FE * w**1.5 + K_FR * w


def main(out: str) -> None:
    s, t = np.meshgrid(SPEED_RPM, TORQUE_NM, indexing="ij")
    z = loss_w(s, t)
    header = "speed_rpm/torque_nm," + ",".join(f"{v:g}" for v in TORQUE_NM)
    rows = [f"{sp:g}," + ",".join(f"{v:.3f}" for v in z[i]) for i, sp in enumerate(SPEED_RPM)]
    Path(out).write_text(header + "\n" + "\n".join(rows) + "\n", encoding="utf-8")
    print(f"wrote {out}: {len(SPEED_RPM)} speeds x {len(TORQUE_NM)} torques")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "configs/maps/generic_pmsm_loss.csv")
