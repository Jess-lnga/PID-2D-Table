"""ESP32 inverse kinematics port, lengths in mm and angles in degrees."""
from dataclasses import dataclass
from pathlib import Path
import math
import re
import numpy as np

DEFAULT_HEADER = Path(__file__).resolve().parents[1] / 'src/kinematics.h'


@dataclass(frozen=True)
class Parameters:
    L1: float
    L2: float
    Rb: float
    Rp: float
    h0: float
    elbow_sign: tuple
    q_min: tuple
    q_max: tuple


def load_parameters(header=DEFAULT_HEADER):
    """Read numeric initializers, never execute C++ or silently invent dimensions."""
    source = Path(header).read_text(encoding='utf-8')
    source = re.sub(r'/\*.*?\*/|//[^\n]*', '', source, flags=re.S)
    def scalar(name):
        match = re.search(r'\bdouble\s+' + name + r'\s*=\s*([^;]+);', source)
        if not match:
            raise ValueError(f'Paramètre absent : {name}')
        try:
            return float(match[1].strip())
        except ValueError as exc:
            raise ValueError(f'{name} doit avoir une valeur numérique dans le header') from exc
    def triple(name):
        match = re.search(r'\b' + name + r'\s*\[3\]\s*=\s*\{([^}]+)\}', source)
        if not match:
            raise ValueError(f'Paramètre absent : {name}')
        values = tuple(float(v.strip()) for v in match[1].split(','))
        if len(values) != 3:
            raise ValueError(f'{name} doit contenir trois valeurs')
        return values
    return Parameters(*(scalar(n) for n in ('L1', 'L2', 'Rb', 'Rp', 'h0')),
                      triple('elbowSign'), triple('qMinDeg'), triple('qMaxDeg'))


@dataclass
class Result:
    error: str = 'None'
    failed_leg: int = -1
    angles: np.ndarray | None = None
    center: np.ndarray | None = None
    rotation: np.ndarray | None = None
    bases: np.ndarray | None = None
    joints: np.ndarray | None = None
    elbows: np.ndarray | None = None
    yaw_deg: float = 0.0

    @property
    def valid(self):
        return self.error == 'None'


def compute_kinematics(p, z, roll_deg, pitch_deg):
    """Same branch, tolerances, limits and error names as kinematics.cpp."""
    out = Result()
    def fail(error, leg=-1):
        out.error, out.failed_leg = error, leg
        return out
    if (not all(math.isfinite(v) for v in (p.L1, p.L2, p.Rb, p.Rp, p.h0))
            or p.L1 <= 0 or p.L2 <= 0 or p.Rb < 0 or p.Rp <= 0 or p.h0 < 0):
        return fail('InvalidParameters')
    for i in range(3):
        if (p.elbow_sign[i] not in (-1, 1)
                or not math.isfinite(p.q_min[i]) or not math.isfinite(p.q_max[i])
                or p.q_min[i] > p.q_max[i] or p.q_max[i]-p.q_min[i] > 360):
            return fail('InvalidParameters', i)
    if (not all(math.isfinite(v) for v in (z, roll_deg, pitch_deg))
            or z < 0 or not math.isfinite(p.h0+z)):
        return fail('InvalidInput')
    roll, pitch = (math.radians(math.remainder(v, 360)) for v in (roll_deg, pitch_deg))
    sr, cr, sp, cp = math.sin(roll), math.cos(roll), math.sin(pitch), math.cos(pitch)
    a, b = sp*sr, cp+cr
    if math.hypot(a, b) < 1e-10:
        return fail('SingularOrientation')
    yaw = math.atan2(a, b)
    if yaw > math.pi/2:
        yaw -= math.pi
    if yaw < -math.pi/2:
        yaw += math.pi
    sy, cy = math.sin(yaw), math.cos(yaw)
    R = np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                  [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                  [-sp, cp*sr, cp*cr]])
    center = np.array([.5*p.Rp*(R[0, 0]-R[1, 1]), -p.Rp*R[1, 0], p.h0+z])
    scale = p.L1+p.L2+p.Rb+p.Rp+center[2]
    if not math.isfinite(scale):
        return fail('InvalidInput')
    alpha = np.arange(3)*2*math.pi/3
    radial = np.column_stack((np.cos(alpha), np.sin(alpha), np.zeros(3)))
    bases = p.Rb*radial
    joints = (R @ (p.Rp*radial).T).T + center
    out.center, out.rotation, out.bases, out.joints = center, R, bases, joints
    out.yaw_deg = math.degrees(yaw)
    angles = []
    for i, (X, Y, Z) in enumerate(joints):
        c, s = math.cos(alpha[i]), math.sin(alpha[i])
        u, v = c*X+s*Y-p.Rb, Z
        d = math.hypot(u, v)
        if not math.isfinite(d):
            return fail('InvalidInput', i)
        if d <= 1e-10*scale:
            return fail('SingularLeg', i)
        if d > p.L1+p.L2+1e-9*scale or d < abs(p.L1-p.L2)-1e-9*scale:
            return fail('Unreachable', i)
        cosine = (d/p.L1+p.L1/d-(p.L2/d)*(p.L2/p.L1))/2
        if not math.isfinite(cosine):
            return fail('InvalidInput', i)
        if not -1-1e-8 <= cosine <= 1+1e-8:
            return fail('Unreachable', i)
        q = math.degrees(math.atan2(v, u)+p.elbow_sign[i]*math.acos(max(-1, min(1, cosine))))
        q += 360*math.ceil((p.q_min[i]-q-1e-8)/360)
        if q < p.q_min[i]-1e-8 or q > p.q_max[i]+1e-8:
            return fail('JointLimit', i)
        angles.append(max(p.q_min[i], min(p.q_max[i], q)))
    out.angles = np.array(angles)
    q_rad = np.radians(out.angles)
    out.elbows = bases + p.L1*np.cos(q_rad)[:, None]*radial
    out.elbows[:, 2] += p.L1*np.sin(q_rad)
    return out
