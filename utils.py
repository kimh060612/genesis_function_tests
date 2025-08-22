import math

def quat_mul(p, q):
    pw, px, py, pz = p
    qw, qx, qy, qz = q
    return (
        pw*qw - px*qx - py*qy - pz*qz,
        pw*qx + px*qw + py*qz - pz*qy,
        pw*qy - px*qz + py*qw + pz*qx,
        pw*qz + px*qy - py*qx + pz*qw,
    )

def quat_normalize(q):
    w, x, y, z = q
    n = math.sqrt(w*w + x*x + y*y + z*z)
    return (w/n, x/n, y/n, z/n)

def euler_to_quat(roll, pitch, yaw, order='ZYX', degrees=False):
    """Intrinsic rotations. order is a string of three axes, e.g. 'ZYX' or 'XYZ'."""
    if degrees:
        roll, pitch, yaw = map(lambda a: a*math.pi/180.0, (roll, pitch, yaw))
    qx = (math.cos(roll/2),  math.sin(roll/2), 0.0, 0.0)
    qy = (math.cos(pitch/2), 0.0, math.sin(pitch/2), 0.0)
    qz = (math.cos(yaw/2),   0.0, 0.0, math.sin(yaw/2))

    # map axis char to elemental quaternion
    qmap = {'X': qx, 'Y': qy, 'Z': qz}

    # intrinsic rotations: left-to-right multiply of listed axes
    # e.g., 'ZYX' -> q = qZ ⊗ qY ⊗ qX
    q = (1.0, 0.0, 0.0, 0.0)
    for ax in order:
        q = quat_mul(qmap[ax], q)
    return quat_normalize(q)

if __name__ == "__main__":
    # 예: ZYX(yaw-pitch-roll), yaw=90deg
    print(euler_to_quat(0.0, 0, 270, order='ZYX', degrees=True))
    # ≈ (0.7071, 0.0, 0.0, 0.7071)