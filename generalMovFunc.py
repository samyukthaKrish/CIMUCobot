from xarm.wrapper import XArmAPI
import math
import time

arm = XArmAPI('127.0.0.1')

SAFE_POSE = [0, -30.0, 0, -10.0, 175.0]

# xArm5 DH parameters from manual Appendix 8
D1     = 267.0
A2     = 289.48866
A3     = 351.158796
D5     = 97.0
A5     = 76.0
T2_OFF = math.radians(-79.34995)
T3_OFF = math.radians(156.599924)
T4_OFF = math.radians(-77.249974)

def mat_mul(A, B):
    """4x4 matrix multiply."""
    C = [[0]*4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            for k in range(4):
                C[i][j] += A[i][k] * B[k][j]
    return C

def dh_matrix(alpha, a, d, theta):
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return [
        [ct,    -st,    0,    a      ],
        [st*ca,  ct*ca, -sa,  -sa*d  ],
        [st*sa,  ct*sa,  ca,   ca*d  ],
        [0,      0,      0,    1     ]
    ]

DH_PARAMS = [
    (0,          0,   D1,  0      ),
    (-math.pi/2, 0,   0,   T2_OFF ),
    (0,          A2,  0,   T3_OFF ),
    (0,          A3,  0,   T4_OFF ),
    (-math.pi/2, A5,  D5,  0      ),
]

def forward_kinematics(joint_angles_deg):
    """Returns [x, y, z] in mm from joint angles in degrees."""
    T = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
    for i, (alpha, a, d, offset) in enumerate(DH_PARAMS):
        theta = math.radians(joint_angles_deg[i]) + offset
        T = mat_mul(T, dh_matrix(alpha, a, d, theta))
    return [T[0][3], T[1][3], T[2][3]]

def norm3(v):
    return math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)

def inverse_kinematics(target_xyz, seed=None, tol=1.0, max_iter=300, step=0.5):
    """
    Numerical IK using finite-difference Jacobian, no numpy needed.
    """
    if seed is None:
        seed = SAFE_POSE[:]
    
    joints = list(seed)
    target = target_xyz
    eps = 0.1  # degrees for numerical jacobian

    limits = [
        (-360, 360),
        (-118, 120),
        (-225, 11),
        (-97,  180),
        (-360, 360),
    ]

    for iteration in range(max_iter):
        curr = forward_kinematics(joints)
        err = [target[i] - curr[i] for i in range(3)]
        dist = norm3(err)

        if dist < tol:
            return joints

        # Build 3x5 Jacobian numerically
        J = [[0.0]*5 for _ in range(3)]
        for j in range(5):
            j_plus = list(joints)
            j_plus[j] += eps
            p_plus = forward_kinematics(j_plus)
            for r in range(3):
                J[r][j] = (p_plus[r] - curr[r]) / eps

        # Pseudoinverse via J^T(JJ^T)^-1 — works for 3x5 (overdetermined in joints)
        # Compute JJ^T (3x3)
        JJT = [[0.0]*3 for _ in range(3)]
        for r in range(3):
            for c in range(3):
                for k in range(5):
                    JJT[r][c] += J[r][k] * J[c][k]

        # Add damping for stability
        lam = 0.01 * dist
        for i in range(3):
            JJT[i][i] += lam

        # Solve JJT * x = err  (3x3 system via Cramer's rule / Gaussian elim)
        x = gaussian_solve(JJT, err)
        if x is None:
            return None

        # delta_joints = J^T * x
        delta = [0.0]*5
        for j in range(5):
            for r in range(3):
                delta[j] += J[r][j] * x[r]

        # Clamp step
        delta_norm = norm3(delta[:3]) + abs(delta[3]) + abs(delta[4])
        max_step = step
        if delta_norm > max_step:
            scale = max_step / delta_norm
            delta = [d * scale for d in delta]

        for j in range(5):
            joints[j] += delta[j]
            joints[j] = max(limits[j][0], min(limits[j][1], joints[j]))

    return None  # did not converge

def gaussian_solve(A, b):
    """Solve 3x3 system Ax=b via Gaussian elimination."""
    n = 3
    M = [A[i][:] + [b[i]] for i in range(n)]
    for col in range(n):
        # Pivot
        max_row = max(range(col, n), key=lambda r: abs(M[r][col]))
        M[col], M[max_row] = M[max_row], M[col]
        if abs(M[col][col]) < 1e-12:
            return None
        for row in range(col+1, n):
            f = M[row][col] / M[col][col]
            for k in range(col, n+1):
                M[row][k] -= f * M[col][k]
    x = [0.0]*n
    for i in range(n-1, -1, -1):
        x[i] = M[i][n]
        for j in range(i+1, n):
            x[i] -= M[i][j] * x[j]
        x[i] /= M[i][i]
    return x

def prepare_arm():
    arm.clean_error()
    arm.motion_enable(True)
    arm.set_mode(0)
    arm.set_state(0)
    time.sleep(0.5)

def go_home(speed=20):
    code = arm.set_servo_angle(angle=SAFE_POSE, speed=speed, wait=True)
    if code != 0:
        print(f"Home failed, code={code}")
        return False
    return True

def move_to(x, y, z, speed=15):
    """Move xArm5 TCP to (x, y, z) in mm. Just works."""
    prepare_arm()
    go_home()

    print(f"Moving to X:{x} Y:{y} Z:{z}...")
    joints = inverse_kinematics([x, y, z], seed=SAFE_POSE[:])

    if joints is None:
        print("Could not find IK solution — target unreachable")
        return False

    predicted = forward_kinematics(joints)
    print(f"Joints:    {[f'{a:.1f}' for a in joints]}")
    print(f"Predicted: X:{predicted[0]:.1f} Y:{predicted[1]:.1f} Z:{predicted[2]:.1f}")

    code = arm.set_servo_angle(angle=joints, speed=speed, mvacc=500, wait=True)
    if code != 0:
        print(f"Move failed, code={code}")
        arm.clean_error()
        arm.set_state(0)
        return False

    _, actual = arm.get_position()
    print(f"Actual:    X:{actual[0]:.1f} Y:{actual[1]:.1f} Z:{actual[2]:.1f}")
    err = math.sqrt((actual[0]-x)**2 + (actual[1]-y)**2 + (actual[2]-z)**2)
    print(f"Error: {err:.1f}mm")
    return True

#testing out code 
prepare_arm()

move_to(300, 0, 300)
move_to(300, 100, 300)
move_to(300, -100, 300)
move_to(300, 0, 200)

time.sleep(1)
arm.disconnect()