  
export function quat_to_rpy(qw, qx, qy, qz) {
    // same quat to rpy as in the deepmimic_mujoco env
    let q0 = qw;
    let q1 = qx;
    let q2 = qy;
    let q3 = qz;
    let roll = Math.atan2(
        2 * ((q2 * q3) + (q0 * q1)),
        q0**2 - q1**2 - q2**2 + q3**2
    ); // radians
    let pitch = Math.asin(2 * ((q1 * q3) - (q0 * q2)));
    let yaw = Math.atan2(
        2 * ((q1 * q2) + (q0 * q3)),
        q0**2 + q1**2 - q2**2 - q3**2
    );
    return [roll, pitch, yaw];
  }