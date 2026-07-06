# Generates the PTP/LIN/CIRC/SPLINE motion-types demo program for HexaArm Mini.
# See README.md in this folder for usage, tuning knobs and the verification pipeline.
import os as _os
from pathlib import Path as _Path
# All data paths below are relative to the GIT ROOT (the folder containing Enterprise-RDT-dev/):
# resolve it from this script's location so the generator runs from any working directory.
_os.chdir(_Path(__file__).resolve().parents[3])
# Generates the PTP/LIN/CIRC/SPLINE demo program for HexaArm Mini.
# FK/IK straight from the URDF so every taught pose is verified reachable and the PTP corners
# get real joint solutions.
# Output: the CONTROLLER wire format - versioned envelope {formatVersion: 2, program: {name,
# steps}} with ProgramStepStruct fields exactly as RdtJson.h serializes them. The old
# pendant-local step format ({code, name, params, uuid}) is DEAD since the pendant store was
# removed (studio 0.6.24); FileManager refuses it ("not a versioned program file").
import json, math, io, sys
import xml.etree.ElementTree as ET

CFG = 'HexaMotion/configs/hexacore_config.json'
# The robot changed (HexaArm Mini -> HexaArmMedium_Light). Read the URDF from the controller config
# so this generator can NEVER again ship a demo built against the wrong robot: whatever arm the
# controller runs, the demo is sized and IK-verified for THAT arm.
URDF = json.load(io.open(CFG, encoding='utf-8-sig'))['urdfPath']
print('URDF from config:', URDF)

# ---------- minimal matrix math ----------
def mat_mul(A, B):
    return [[sum(A[i][k]*B[k][j] for k in range(4)) for j in range(4)] for i in range(4)]

def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return [[1,0,0,0],[0,c,-s,0],[0,s,c,0],[0,0,0,1]]

def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return [[c,0,s,0],[0,1,0,0],[-s,0,c,0],[0,0,0,1]]

def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return [[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1]]

def trans(x, y, z):
    return [[1,0,0,x],[0,1,0,y],[0,0,1,z],[0,0,0,1]]

def rot_rpy(r, p, y):  # URDF fixed-axis: R = Rz(y)*Ry(p)*Rx(r)
    return mat_mul(rot_z(y), mat_mul(rot_y(p), rot_x(r)))

def rot_axis(ax, a):  # Rodrigues
    x, y, z = ax
    n = math.sqrt(x*x+y*y+z*z); x, y, z = x/n, y/n, z/n
    c, s, C = math.cos(a), math.sin(a), 1-math.cos(a)
    return [[x*x*C+c, x*y*C-z*s, x*z*C+y*s, 0],
            [y*x*C+z*s, y*y*C+c, y*z*C-x*s, 0],
            [z*x*C-y*s, z*y*C+x*s, z*z*C+c, 0],
            [0,0,0,1]]

# ---------- URDF chain ----------
tree = ET.parse(URDF)
joints = []
for j in tree.getroot().iter('joint'):
    o = j.find('origin'); a = j.find('axis')
    xyz = [float(v) for v in o.get('xyz').split()]
    rpy = [float(v) for v in o.get('rpy').split()]
    axis = [float(v) for v in a.get('xyz').split()] if a is not None else None
    joints.append((j.get('name'), j.get('type'), xyz, rpy, axis))

# The controller prepends the config root_transform (modelRoot*, the URDF up-axis fix) to the KDL
# chain (KinematicModel.cpp root_correction), so its WORLD frame is Rx(90)*URDF: Z-up. All poses on
# the wire are world-frame — the first generator revision missed this and commanded raw URDF-frame
# poses (sideways, straddling the floor, KDL -5). FK here applies the same correction.
_cfg_early = json.load(io.open(CFG, encoding='utf-8-sig'))
ROOT_T = mat_mul(trans(_cfg_early.get('modelRootX', 0.0) / 1000.0,
                       _cfg_early.get('modelRootY', 0.0) / 1000.0,
                       _cfg_early.get('modelRootZ', 0.0) / 1000.0),
                 rot_rpy(math.radians(_cfg_early.get('modelRootRxDeg', 0.0)),
                         math.radians(_cfg_early.get('modelRootRyDeg', 0.0)),
                         math.radians(_cfg_early.get('modelRootRzDeg', 0.0))))

def fk(q_deg):
    T = [row[:] for row in ROOT_T]
    qi = 0
    for name, jtype, xyz, rpy, axis in joints:
        T = mat_mul(T, mat_mul(trans(*xyz), rot_rpy(*rpy)))
        if jtype != 'fixed':
            T = mat_mul(T, rot_axis(axis, math.radians(q_deg[qi])))
            qi += 1
    return T

def pose_of(T):  # -> (x,y,z mm, rx,ry,rz deg) in the project's Rz*Ry*Rx convention
    x, y, z = T[0][3]*1000, T[1][3]*1000, T[2][3]*1000
    sy = -T[2][0]
    sy = max(-1.0, min(1.0, sy))
    ry = math.asin(sy)
    rx = math.atan2(T[2][1], T[2][2])
    rz = math.atan2(T[1][0], T[0][0])
    return [x, y, z, math.degrees(rx), math.degrees(ry), math.degrees(rz)]

def rot_err_vec(Tc, Tt):  # rotation vector of R_t * R_c^T (world frame), radians
    R = [[sum(Tt[i][k]*Tc[j][k] for k in range(3)) for j in range(3)] for i in range(3)]
    tr = R[0][0]+R[1][1]+R[2][2]
    c = max(-1.0, min(1.0, (tr-1)/2))
    a = math.acos(c)
    if a < 1e-9: return [0.0, 0.0, 0.0]
    f = a/(2*math.sin(a))
    return [f*(R[2][1]-R[1][2]), f*(R[0][2]-R[2][0]), f*(R[1][0]-R[0][1])]

def ik(target_T, seed_deg, iters=400):
    q = list(seed_deg)
    for _ in range(iters):
        Tc = fk(q)
        e = [(target_T[i][3]-Tc[i][3]) for i in range(3)] + rot_err_vec(Tc, target_T)
        if math.sqrt(sum(v*v for v in e[:3])) < 1e-5 and math.sqrt(sum(v*v for v in e[3:])) < 2e-4:
            return q
        # finite-difference jacobian (6x6)
        J = [[0.0]*6 for _ in range(6)]
        h = 0.05  # deg
        for c in range(6):
            qp = list(q); qp[c] += h
            Tp = fk(qp)
            ep = [(Tp[i][3]-Tc[i][3])/math.radians(h) for i in range(3)]
            er = rot_err_vec(Tc, Tp)
            for r in range(3):
                J[r][c] = ep[r]
                J[3+r][c] = er[r]/math.radians(h)
        # damped least squares: dq = J^T (J J^T + l I)^-1 e   (solve 6x6 via gauss)
        lam = 1e-4
        JJT = [[sum(J[i][k]*J[j][k] for k in range(6)) + (lam if i == j else 0.0) for j in range(6)] for i in range(6)]
        # gauss solve JJT * y = e
        M = [row[:] + [e[i]] for i, row in enumerate(JJT)]
        for col in range(6):
            piv = max(range(col, 6), key=lambda r: abs(M[r][col]))
            M[col], M[piv] = M[piv], M[col]
            if abs(M[col][col]) < 1e-14: return None
            for r in range(6):
                if r != col:
                    f = M[r][col]/M[col][col]
                    for cc in range(col, 7):
                        M[r][cc] -= f*M[col][cc]
        yv = [M[i][6]/M[i][i] for i in range(6)]
        dq = [math.degrees(sum(J[r][c]*yv[r] for r in range(6))) for c in range(6)]
        # step clamp for stability
        mx = max(abs(v) for v in dq)
        if mx > 8.0:
            dq = [v*8.0/mx for v in dq]
        q = [q[i] + dq[i] for i in range(6)]
    return None

def pose_to_T(p):  # inverse of pose_of: mm+deg -> T
    T = mat_mul(rot_z(math.radians(p[5])), mat_mul(rot_y(math.radians(p[4])), rot_x(math.radians(p[3]))))
    T[0][3], T[1][3], T[2][3] = p[0]/1000, p[1]/1000, p[2]/1000
    return T

# ---------- limits ----------
cfg = json.load(io.open(CFG, encoding='utf-8-sig'))
limits = cfg['definition']['axisLimits']

def within_limits(q):
    return all(limits[i][0] + 2 <= q[i] <= limits[i][1] - 2 for i in range(6))

# ---------- find a comfortable reference configuration ----------
print('FK(zero):', [round(v, 1) for v in pose_of(fk([0]*6))])
# HexaArmMedium_Light is ~2.4x the Mini (upper arm 590 mm, forearm 448 mm, reach ~1.1 m), so the
# reference-config sweep is broader and denser than the Mini's.
candidates = []
for a2 in range(-70, 71, 10):
    for a3 in range(-110, 111, 15):
        for a5 in range(-90, 91, 15):
            candidates.append([0, a2, a3, 0, a5, 0])

def up_axis():
    # rotate A1: the invariant coordinate is the vertical axis
    p0 = pose_of(fk([0]*6)); p1 = pose_of(fk([40, 0, 0, 0, 0, 0]))
    d = [abs(p0[i]-p1[i]) for i in range(3)]
    return d.index(min(d))

UP = up_axis()
H = [i for i in range(3) if i != UP]
print('up axis index:', UP, '(0=x 1=y 2=z), horizontal:', H)

scored = []
for q_ref in candidates:
    p = pose_of(fk(q_ref))
    horiz = math.sqrt(p[H[0]]**2 + p[H[1]]**2)
    up = p[UP]
    # Working band for HexaArmMedium_Light (probed from FK): comfortably inside the ~1.1 m reach,
    # tool out in front at a natural table height. Target centre ~660 mm out, ~640 mm up.
    if horiz < 520 or horiz > 820:
        continue
    if up < 430 or up > 900:
        continue
    scored.append((abs(horiz-660) + abs(up-640), q_ref, p))
scored.sort(key=lambda t: t[0])
if not scored:
    sys.exit('no reference configuration found')
CENTER_CANDIDATES = scored[:12]
Q_REF, C_POSE = CENTER_CANDIDATES[0][1], CENTER_CANDIDATES[0][2]
print('center candidates:', len(CENTER_CANDIDATES), 'best:', Q_REF,
      [round(v, 1) for v in C_POSE])

# ---------- demo geometry ----------
# Boss request (2026-07-06): the pattern lies in the HORIZONTAL plane with only a small height
# amplitude between passes. Zigzag corners (h1, h2 offsets in mm) shared by every pass.
# Boss request v3: WIDER XY spread. The generator tries the full size first and shrinks in steps
# only if the dense IK verification cannot reach it with the tool pointing straight down.
# Boss request v5 (2026-07-06): SIX waypoints per pass (was four) on a BIGGER pattern - the corner
# spread grows to 300 x 210 mm (not a denser zigzag on the old footprint). The scale ladder below
# shrinks the footprint only if the dense IK verification cannot reach the full size.
# Boss request v6 (2026-07-06): the passes read as separate LAYERS.
# Boss request v7 (2026-07-06): layer distance 50 mm (12 -> 20 -> 30 -> 50 mm across revisions);
# no return-to-home at the end.
# Boss request v8 (2026-07-06): SERPENTINE layers - a pass ends, the tool drops pure-Z at THAT
# corner and the next pass runs in the OPPOSITE direction. No traverse back to the pattern start:
# the old TRAVERSE drew a line striking through the pattern.
# Scaled ~2.2x from the Mini footprint for HexaArmMedium_Light: the pattern spans ~660 x 460 mm
# and the four passes are 320 mm apart top-to-bottom (bold and readable across a room).
CORNERS_FULL = [(-330, -230), (-198, 230), (-66, -230), (66, 230), (198, -230), (330, 230)]
CORNERS = CORNERS_FULL  # rescaled by the search loop below
PASS_UP = {'PTP': 160.0, 'LIN': 55.0, 'CIRC': -55.0, 'SPLINE': -160.0}

# CIRC via: perpendicular offset from the chord midpoint of P0 -> P3
(a0, b0), (a3, b3) = CORNERS[0], CORNERS[-1]
mid = ((a0 + a3) / 2, (b0 + b3) / 2)
ch = (a3 - a0, b3 - b0); chn = math.hypot(*ch)
perp = (-ch[1] / chn, ch[0] / chn)
via_ab = (mid[0] + perp[0] * 45, mid[1] + perp[1] * 45)

# ---------- orientation: escape the RPY singularity ----------
# FK of this arm's natural configs sits at ry = +90 deg, the exact gimbal-lock point of the ZYX
# (Rz*Ry*Rx) representation every CartPose round trip uses. There the euler extraction is
# ill-conditioned and the KDL solver received a corrupted orientation (live failure: KDL -5 at the
# first LIN sample). Fix: tilt the tool by a world-frame rotation so abs(ry) <= 70 deg, and REQUIRE
# the euler round trip to reproduce the rotation exactly before accepting a candidate.
R0 = fk(Q_REF)

def rot_only(T):
    M = [row[:] for row in T]
    M[0][3] = M[1][3] = M[2][3] = 0.0
    return M

def orientation_candidates():
    # Boss request v3/v4: the TCP must point STRAIGHT DOWN (table work). Empirically confirmed on
    # the live robot: the (rx=180, ry=0) family points the tool UP on this flange, so DOWN is the
    # mirrored family (rx=0, ry=0) — the tool axis of this URDF tip runs along flange -Z.
    # ry=0 keeps the representation far from the ZYX gimbal lock; rz candidates give the wrist
    # different elbow-room around the pattern.
    for rz in (90.0, 0.0, 180.0, -90.0, 45.0, 135.0, -45.0, -135.0):
        yield pose_to_T([0.0, 0.0, 0.0, 0.0, 0.0, rz])

def euler_roundtrip_ok(R):
    p = pose_of(R)
    if abs(p[4]) > 70.0:
        return None
    T = pose_to_T(p)
    err = max(abs(T[i][j] - R[i][j]) for i in range(3) for j in range(3))
    return p[3:] if err < 1e-9 else None

def make_pose(a, b, up_off, orient):
    p = [0.0] * 6
    p[H[0]] = C_POSE[H[0]] + a
    p[H[1]] = C_POSE[H[1]] + b
    p[UP] = C_POSE[UP] + up_off
    p[3], p[4], p[5] = orient
    return p

# ---------- dense-path verification (mimics the runtime IK point stream) ----------
def circle_points(p0, pv, p1, n):
    A = [p0[i] for i in range(3)]; B = [pv[i] for i in range(3)]; C = [p1[i] for i in range(3)]
    a = [B[i] - A[i] for i in range(3)]; b = [C[i] - A[i] for i in range(3)]
    def cross(u, v): return [u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0]]
    def dot(u, v): return sum(u[i]*v[i] for i in range(3))
    def norm(u): return math.sqrt(dot(u, u))
    w = cross(a, b); w2 = dot(w, w)
    num = cross([dot(a,a)*b[i] - dot(b,b)*a[i] for i in range(3)], w)
    ctr = [A[i] + num[i] / (2*w2) for i in range(3)]
    r = norm([A[i]-ctr[i] for i in range(3)])
    u = [(A[i]-ctr[i])/r for i in range(3)]
    n_hat = [w[i]/norm(w) for i in range(3)]
    v = cross(n_hat, u)
    def ang(P):
        d = [P[i]-ctr[i] for i in range(3)]
        t = math.atan2(dot(d, v), dot(d, u))
        return t + 2*math.pi if t < 0 else t
    tv, te = ang(B), ang(C)
    if tv > te:
        v = [-x for x in v]
        te = 2*math.pi - te
    return [[ctr[i] + r*(math.cos(te*k/n)*u[i] + math.sin(te*k/n)*v[i]) for i in range(3)] for k in range(n+1)]

def verify_orientation(orient):
    # Execution-ordered pose sequence per pass, densely sampled (~8 mm), IK-chained like the runtime.
    # The FIRST pose may require folding the wrist ~180 deg (tool down), which the local DLS solver
    # cannot always reach from the elbow-up seed: try several wrist-flipped seed variants first.
    first_solved = False
    seed = list(Q_REF)
    def solve(p):
        nonlocal seed, first_solved
        seeds = [seed]
        if not first_solved:
            for d5 in (90, -90, 150, -150, 180):
                for d4 in (0, 180, -180):
                    v = list(seed); v[4] += d5; v[3] += d4
                    seeds.append(v)
        q = None
        for sd in seeds:
            q = ik(pose_to_T(p), sd)
            if q is not None and within_limits(q):
                break
            q = None
        if q is None:
            return False
        first_solved = True
        seed = q
        return True
    def line(pa, pb):
        d = math.sqrt(sum((pb[i]-pa[i])**2 for i in (0, 1, 2)))
        n = max(2, int(d/8))
        for k in range(1, n+1):
            p = [pa[i] + (pb[i]-pa[i])*k/n for i in range(3)] + list(orient)
            if not solve(p):
                return False
        return True
    prev = None
    forward = True   # serpentine: PTP forward, LIN reverse, CIRC forward, SPLINE reverse
    for name, up in PASS_UP.items():
        seq = CORNERS if forward else list(reversed(CORNERS))
        pts = [make_pose(a, b, up, orient) for a, b in seq]
        if prev is not None:
            # Boss v8 layer change: pure-Z drop at the corner the previous pass ended on (the next
            # pass starts right there and runs the other way) - verified exactly as it executes.
            if not line(prev, pts[0]):
                return False
        if name == 'CIRC':
            arc = circle_points(pts[0], make_pose(*via_ab, up, orient), pts[-1], 24)
            for xyz in arc:
                if not solve(list(xyz) + list(orient)):
                    return False
            prev = pts[-1]
        else:
            for j in range(len(pts) - 1):
                if not line(pts[j], pts[j+1]):
                    return False
            prev = pts[-1]
        forward = not forward
    return True

ORIENT = None
CHOSEN_SCALE = None
for scale in (1.0, 0.92, 0.85, 0.7):
    for _, q_ref_cand, c_pose_cand in CENTER_CANDIDATES:
        Q_REF, C_POSE = q_ref_cand, c_pose_cand
        CORNERS = [(a * scale, b * scale) for a, b in CORNERS_FULL]
        (a0, b0), (a3, b3) = CORNERS[0], CORNERS[-1]
        mid = ((a0 + a3) / 2, (b0 + b3) / 2)
        ch = (a3 - a0, b3 - b0); chn = math.hypot(*ch)
        perp = (-ch[1] / chn, ch[0] / chn)
        via_ab = (mid[0] + perp[0] * 45 * scale, mid[1] + perp[1] * 45 * scale)
        for cand_R in orientation_candidates():
            orient = euler_roundtrip_ok(cand_R)
            if orient is None:
                continue
            if verify_orientation(tuple(orient)):
                ORIENT = tuple(orient)
                CHOSEN_SCALE = scale
                break
        if ORIENT is not None:
            break
    if ORIENT is not None:
        break
if ORIENT is None:
    sys.exit('no orientation/scale/center combination passed dense-path IK verification')
print('orientation (rx, ry, rz):', [round(v, 2) for v in ORIENT], 'scale:', CHOSEN_SCALE,
      'center:', [round(v, 1) for v in C_POSE[:3]], 'Q_REF:', Q_REF)

# PTP corner joints (top pass), each solved from the previous corner's solution. The first corner
# needs the same wrist-flip seed variants as the verifier (tool-down is ~180 deg from the seed).
ptp_joints = []
seed = list(Q_REF)
for idx, (a, b) in enumerate(CORNERS):
    target = pose_to_T(make_pose(a, b, PASS_UP['PTP'], ORIENT))
    seeds = [seed]
    if idx == 0:
        for d5 in (90, -90, 150, -150, 180):
            for d4 in (0, 180, -180):
                v = list(seed); v[4] += d5; v[3] += d4
                seeds.append(v)
    q = None
    for sd in seeds:
        q = ik(target, sd)
        if q is not None and within_limits(q):
            break
        q = None
    if q is None:
        sys.exit('IK failed for a PTP corner')
    ptp_joints.append([round(v, 3) for v in q])
    seed = q

# ---------- build the program (controller wire format) ----------
steps = []

def add_step(step_type, comment_text, extra=None):
    # One wire ProgramStepStruct, exactly the fields RdtJson.h reads for this type. Sequential
    # ids (the studio assigns them the same way when authoring).
    step = {'id': len(steps), 'type': step_type, 'tool_id': 0, 'base_id': 0,
            'comment': comment_text}
    if extra:
        step.update(extra)
    steps.append(step)

r3 = lambda p: [round(v, 3) for v in p]

def cart(pose):  # [x,y,z,rx,ry,rz] -> wire CartPose object (mm / deg, plain numbers)
    p = r3(pose)
    return {'x': p[0], 'y': p[1], 'z': p[2], 'rx': p[3], 'ry': p[4], 'rz': p[5]}

SEED_CHAIN = {'q': None}  # advances step by step, mirroring the planner's chain order

def verify_reachable(pose):
    # Chained IK proves every Cartesian waypoint is reachable from the previous one (the same
    # seeding order the planner uses), so generation fails fast instead of shipping a demo that
    # faults at run time. The wire cart steps carry no joints; this is validation only.
    q = ik(pose_to_T(pose), SEED_CHAIN['q'])
    if q is None or not within_limits(q):
        sys.exit(f'IK failed for a Cartesian waypoint: {pose[:3]}')
    SEED_CHAIN['q'] = q

def comment(text):
    add_step('Comment', text)

def move_j(name, joints, speed):
    SEED_CHAIN['q'] = list(joints)
    add_step('MoveJ', name, {'joint_target': [round(v, 3) for v in joints],
                             'speed_ratio': float(speed), 'blending_radius_mm': 0.0})

def move_cart(step_type, name, pose, speed, via=None):
    if via is not None:
        verify_reachable(via)
    verify_reachable(pose)
    extra = {'cart_target': cart(pose), 'speed_ratio': float(speed), 'blending_radius_mm': 0.0}
    if via is not None:
        extra['cart_via'] = cart(via)
    add_step(step_type, name, extra)

def wait(sec):
    add_step('WaitTime', f'WAIT {sec}s', {'wait_duration_s': float(sec)})

SEED_CHAIN['q'] = list(Q_REF)

def add_layer_down(corner, next_up):
    # Boss v8: the layer change is ONE pure-Z LIN drop at the corner the previous pass ended on;
    # the next pass starts right there and runs the opposite direction (serpentine).
    move_cart('MoveL', 'LAYER DOWN', make_pose(*corner, next_up, ORIENT), 60)

comment('=== MOTION TYPES DEMO: serpentine zigzag, 50 mm layers ===')
comment('PASS 1 (top +75mm): PTP - joint motion, curved TCP path')
for i, ((a, b), qj) in enumerate(zip(CORNERS, ptp_joints)):
    move_j(f'PTP P{i+1}', qj, 40)
wait(0.7)

comment('PASS 2 (+25mm): LIN - straight lines, REVERSE direction')
add_layer_down(CORNERS[-1], PASS_UP['LIN'])
for idx in range(len(CORNERS) - 2, -1, -1):
    move_cart('MoveL', f'LIN P{idx+1}', make_pose(*CORNERS[idx], PASS_UP['LIN'], ORIENT), 60)
wait(0.7)

comment('PASS 3 (-25mm): CIRC - one arc through a via point')
add_layer_down(CORNERS[0], PASS_UP['CIRC'])
move_cart('MoveC', f'CIRC P1->P{len(CORNERS)}',
          make_pose(*CORNERS[-1], PASS_UP['CIRC'], ORIENT), 45,
          via=make_pose(*via_ab, PASS_UP['CIRC'], ORIENT))
wait(0.7)

comment('PASS 4 (bottom -75mm): SPLINE - smooth curve, REVERSE direction')
add_layer_down(CORNERS[-1], PASS_UP['SPLINE'])
for idx in range(len(CORNERS) - 2, -1, -1):
    move_cart('MoveS', f'SPL P{idx+1}', make_pose(*CORNERS[idx], PASS_UP['SPLINE'], ORIENT), 60)
wait(0.7)
# Boss v7: NO return-to-home - the demo ends on the last (serpentine) corner.

doc = {'formatVersion': 2, 'program': {'name': 'DEMO_MOTION_TYPES', 'steps': steps}}
import os
out = 'HexaMotion/programs/DEMO_MOTION_TYPES.json'
os.makedirs(os.path.dirname(out), exist_ok=True)
io.open(out, 'w', encoding='utf-8').write(json.dumps(doc, indent=1))
print('written:', out)
print('steps:', len(steps))
