//! Quaternions (w, x, y, z; f64) for the simulated ground truth.

pub type Quat = [f64; 4];
pub type Vec3 = [f64; 3];

pub const IDENTITY: Quat = [1.0, 0.0, 0.0, 0.0];

pub fn mul(a: Quat, b: Quat) -> Quat {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}

pub fn conj(q: Quat) -> Quat {
    [q[0], -q[1], -q[2], -q[3]]
}

pub fn normalize(q: Quat) -> Quat {
    let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    [q[0] / n, q[1] / n, q[2] / n, q[3] / n]
}

pub fn cross(a: Vec3, b: Vec3) -> Vec3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

/// v' = q v q*.
pub fn rotate(q: Quat, v: Vec3) -> Vec3 {
    let r = [q[1], q[2], q[3]];
    let t = cross(r, v).map(|x| 2.0 * x);
    let c = cross(r, t);
    [v[0] + q[0] * t[0] + c[0], v[1] + q[0] * t[1] + c[1], v[2] + q[0] * t[2] + c[2]]
}

/// Rotation of `deg` degrees about unit `axis`.
pub fn axis_angle(axis: Vec3, deg: f64) -> Quat {
    let h = deg.to_radians() / 2.0;
    [h.cos(), axis[0] * h.sin(), axis[1] * h.sin(), axis[2] * h.sin()]
}

/// q after turning at body rate `w` (rad/s) for `dt` seconds.
pub fn integrate(q: Quat, w: Vec3, dt: f64) -> Quat {
    let n = (w[0] * w[0] + w[1] * w[1] + w[2] * w[2]).sqrt();
    if n < 1e-12 {
        return q;
    }
    let h = n * dt / 2.0;
    let s = h.sin() / n;
    normalize(mul(q, [h.cos(), w[0] * s, w[1] * s, w[2] * s]))
}

/// Body rate (rad/s) that turns `from` into `to` in `dt` seconds.
pub fn rate_between(from: Quat, to: Quat, dt: f64) -> Vec3 {
    let mut d = mul(conj(from), to);
    if d[0] < 0.0 {
        d = d.map(|x| -x);
    }
    let s = (d[1] * d[1] + d[2] * d[2] + d[3] * d[3]).sqrt();
    if s < 1e-12 {
        return [0.0; 3];
    }
    let angle = 2.0 * s.atan2(d[0]);
    [d[1] / s * angle / dt, d[2] / s * angle / dt, d[3] / s * angle / dt]
}

/// Angle between two orientations, degrees.
pub fn angle_deg(a: Quat, b: Quat) -> f64 {
    let d = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]).abs().min(1.0);
    2.0 * d.acos().to_degrees()
}

/// Tilt (pitch / roll) error, degrees: the angle between world up as each orientation sees it from
/// the controller. A pure heading (yaw) difference gives 0.
pub fn tilt_deg(a: Quat, b: Quat) -> f64 {
    let ua = rotate(conj(a), [0.0, 1.0, 0.0]);
    let ub = rotate(conj(b), [0.0, 1.0, 0.0]);
    let d = (ua[0] * ub[0] + ua[1] * ub[1] + ua[2] * ub[2]).clamp(-1.0, 1.0);
    d.acos().to_degrees()
}

/// Heading error, degrees: the yaw of `a` relative to `b`.
pub fn yaw_diff_deg(a: Quat, b: Quat) -> f64 {
    let fa = rotate(a, [0.0, 0.0, -1.0]);
    let fb = rotate(b, [0.0, 0.0, -1.0]);
    let ha = (-fa[0]).atan2(-fa[2]);
    let hb = (-fb[0]).atan2(-fb[2]);
    let mut d = (ha - hb).to_degrees();
    while d > 180.0 {
        d -= 360.0;
    }
    while d < -180.0 {
        d += 360.0;
    }
    d
}

/// Yaw, pitch, roll in degrees (Y-X-Z order: yaw about +Y, + = turned left; pitch about +X, + = up;
/// roll about +Z, + = rolled left), as host/vrtest.py shows them.
pub fn ypr(q: Quat) -> Vec3 {
    let [w, x, y, z] = q;
    let pitch = (2.0 * (w * x - y * z)).clamp(-1.0, 1.0).asin();
    let yaw = (2.0 * (x * z + w * y)).atan2(1.0 - 2.0 * (x * x + y * y));
    let roll = (2.0 * (x * y + w * z)).atan2(1.0 - 2.0 * (x * x + z * z));
    [yaw.to_degrees(), pitch.to_degrees(), roll.to_degrees()]
}

pub fn from_ypr(yaw: f64, pitch: f64, roll: f64) -> Quat {
    let y = axis_angle([0.0, 1.0, 0.0], yaw);
    let p = axis_angle([1.0, 0.0, 0.0], pitch);
    let r = axis_angle([0.0, 0.0, 1.0], roll);
    normalize(mul(y, mul(p, r)))
}
