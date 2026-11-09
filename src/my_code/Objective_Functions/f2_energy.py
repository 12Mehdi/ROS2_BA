"""f2 - Energy: joint torques from the dynamic model (Pinocchio, URDF).

tau = M(q) qdd + c(q, qd) + g(q)    (full rigid-body model, includes gravity)
computed at every sample with the recursive Newton-Euler algorithm (RNEA).

Input `s` is the dict returned by bspline_traj.sample() / load_trajectory():
  s["t"], s["q"], s["qd"], s["qdd"]   (time and joint position/velocity/acceleration)

Two levels: joint_torques() = joint side (tau above); motor_torques() = full
model with gears, rotor inertia and friction (motor side).
"""

import numpy as np


class Robot:
    """Loads the URDF once; reuse the same object for every evaluation."""

    def __init__(self, urdf_path):
        import pinocchio as pin

        self.pin = pin
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        assert self.model.nq == 6, "expected a 6-joint arm"


def joint_torques(robot, s):
    """Torque of every joint at every sample [N m], shape (N, 6)."""
    pin, m, d = robot.pin, robot.model, robot.data
    tau = np.empty_like(s["q"])
    for k in range(len(s["t"])):
        tau[k] = pin.rnea(m, d, s["q"][k], s["qd"][k], s["qdd"][k])
    return tau


def motor_torques(robot, s, gear, Jzz, fric_coulomb=None, fric_viscous=None):
    """Full model, motor side, shape (N, 6) [N m]:

      tau_M = diag(1/u_G) (M(q) qdd + c(q,qd) + g(q)) + h(q,qd) + diag(u_G J_zz) qdd

    gear          u_G   (6,) gear ratios (motor turns per joint turn)   -> from manufacturer
    Jzz           J_zz  (6,) rotor inertia about its axis [kg m^2]      -> from manufacturer
    fric_coulomb  Fc    (6,) motor-side Coulomb friction torque [N m]   (default 0)
    fric_viscous  Fv    (6,) motor-side viscous friction [N m s/rad of joint speed] (default 0)
    h(q, qd) = Fc * sign(qd) + Fv * qd   (simple friction model; replace if the
    manufacturer gives a better one).
    u_G and J_zz have no defaults on purpose: they are not in the URDF.
    """
    u = np.asarray(gear, dtype=float)
    J = np.asarray(Jzz, dtype=float)
    fc = np.zeros(6) if fric_coulomb is None else np.asarray(fric_coulomb, dtype=float)
    fv = np.zeros(6) if fric_viscous is None else np.asarray(fric_viscous, dtype=float)
    h = fc * np.sign(s["qd"]) + fv * s["qd"]
    return joint_torques(robot, s) / u + h + (u * J) * s["qdd"]
