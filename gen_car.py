#!/usr/bin/env python3
"""
E46 M3 GTR mesh -- NFS Most Wanted 2005 proportions.
Real dims: 4617mm long x 1784mm wide x 1346mm tall.
Scaled to 5 units long -> height = 1.46, half-width = 0.97 (race-widened to ~1.08).
Car local coords: X = front, Y = left (positive), Z = up.
Usage: python3 gen_car.py > car.obj
"""
import math, sys

verts = []
faces = []
current_group = "default"

def V(x, y, z):
    verts.append((round(x, 4), round(y, 4), round(z, 4)))
    return len(verts)

def Q(a, b, c, d): faces.append((current_group, a, b, c, d))
def T(a, b, c):    faces.append((current_group, a, b, c))

def group(name):
    global current_group
    current_group = name

# ----------------------------------------------------------------
# BODY SHELL
# 13 cross-sections x 8 vertices, connected by quads.
# Proportions corrected to real E46 M3 GTR dimensions.
# Ring vertex order (viewed from front/+X):
#   0: floor inner-R  1: sill outer-R  2: body outer-R  3: roof edge-R
#   4: roof edge-L    5: body outer-L  6: sill outer-L  7: floor inner-L
# Right Y negative, left Y positive (mirrored).
# ----------------------------------------------------------------
group("body")

def ring(x, yri, yro, yrs, z_sl, z_dr, z_rf):
    return [
        V(x,  yri, 0.0),
        V(x,  yro, z_sl),
        V(x,  yro, z_dr),
        V(x,  yrs, z_rf),
        V(x, -yrs, z_rf),
        V(x, -yro, z_dr),
        V(x, -yro, z_sl),
        V(x, -yri, 0.0),
    ]

# Corrected cross-section data:
# - height scaled up x1.37 from original (car was too flat)
# - width scaled down x0.87 (car was too wide)
# - wheel arch sections kept slightly wider for race widebody
#           x      yri     yro     yrs    z_sl   z_dr   z_rf
sdata = [
    ( 2.50, -0.48, -0.68,  -0.57,  0.06,  0.36,  0.39),  # 0  nose tip
    ( 2.15, -0.60, -0.84,  -0.70,  0.08,  0.44,  0.47),  # 1  front bumper
    ( 1.80, -0.70, -1.00,  -0.82,  0.10,  0.54,  0.60),  # 2  front fascia
    ( 1.40, -0.74, -1.12,  -0.90,  0.14,  0.62,  0.68),  # 3  front wheel arch
    ( 0.95, -0.72, -1.06,  -0.86,  0.14,  0.62,  1.17),  # 4  A-pillar base
    ( 0.50, -0.70, -1.01,  -0.82,  0.14,  0.62,  1.38),  # 5  windshield
    ( 0.00, -0.70, -1.01,  -0.82,  0.14,  0.62,  1.44),  # 6  roof peak
    (-0.50, -0.70, -1.01,  -0.82,  0.14,  0.62,  1.42),  # 7  roof rear
    (-0.95, -0.72, -1.04,  -0.84,  0.14,  0.62,  1.10),  # 8  C-pillar
    (-1.40, -0.76, -1.14,  -0.92,  0.14,  0.64,  0.72),  # 9  rear wheel arch
    (-1.80, -0.72, -1.02,  -0.83,  0.12,  0.62,  0.68),  # 10 rear deck
    (-2.15, -0.60, -0.84,  -0.70,  0.08,  0.52,  0.58),  # 11 rear bumper
    (-2.50, -0.50, -0.70,  -0.59,  0.06,  0.44,  0.50),  # 12 rear end
]

rings = [ring(*sd) for sd in sdata]

for i in range(len(rings) - 1):
    a, b = rings[i], rings[i + 1]
    for j in range(8):
        nj = (j + 1) % 8
        Q(a[j], a[nj], b[nj], b[j])

# ----------------------------------------------------------------
# HOOD CENTER RIDGE (M3 GTR hood bulge, nose to A-pillar)
# ----------------------------------------------------------------
ridge_z = [0.39, 0.48, 0.62, 0.70, 0.70]
ridge = [V(sdata[i][0], 0.0, rz) for i, rz in enumerate(ridge_z)]

for i in range(len(ridge) - 1):
    ra, rb   = ridge[i], ridge[i + 1]
    rr_a, rr_b = rings[i][3], rings[i + 1][3]
    rl_a, rl_b = rings[i][4], rings[i + 1][4]
    T(ra, rr_b, rr_a); T(ra, rb, rr_b)
    T(ra, rl_a, rl_b); T(ra, rl_b, rb)

# ----------------------------------------------------------------
# REAR WING  (tall race wing, NFS M3 GTR style)
# ----------------------------------------------------------------
group("wing")

wy   =  1.42   # half-span
wxf  = -1.72   # leading edge
wxr  = -2.18   # trailing edge
wzb  =  0.95   # bottom attachment (taller than before)
wzt  =  1.48   # top of wing
wzt2 =  1.43   # trailing edge top (angle of attack)

w = [
    V(wxf, -wy, wzb), V(wxr, -wy, wzb), V(wxr, -wy, wzt2), V(wxf, -wy, wzt),
    V(wxf,  wy, wzb), V(wxr,  wy, wzb), V(wxr,  wy, wzt2), V(wxf,  wy, wzt),
]
Q(w[3],w[2],w[1],w[0]); Q(w[4],w[5],w[6],w[7])
Q(w[0],w[1],w[5],w[4]); Q(w[7],w[6],w[2],w[3])
Q(w[3],w[0],w[4],w[7])

ep = 0.34
Q(w[0], w[1], V(wxr,-wy+ep,0), V(wxf,-wy+ep,0))
Q(w[4], w[5], V(wxr, wy-ep,0), V(wxf, wy-ep,0))

for wy_s in [-wy*0.55, wy*0.55]:
    T(V(wxf, wy_s, wzb), V(wxf, wy_s, 0.68), V(-1.65, wy_s, 0.68))

# ----------------------------------------------------------------
# FRONT SPLITTER  (wider, more aggressive -- GTR race splitter)
# ----------------------------------------------------------------
group("body")

sp = [V(2.82,-0.92,0.02), V(2.82,0.92,0.02), V(2.15,0.92,0.02), V(2.15,-0.92,0.02)]
Q(*sp)

# ----------------------------------------------------------------
# WHEELS  (BBS-style, axis along Y, circle in X-Z plane)
# Corrected positions to match narrower body
# ----------------------------------------------------------------
group("wheels")

def add_wheel(cx_ctr, cy, cz_ctr, hw, radius, segs=16):
    """Tire cylinder: axis along Y, circular cross-section in X-Z plane."""
    fa, fb = [], []
    for i in range(segs):
        a = 2 * math.pi * i / segs
        x = cx_ctr + radius * math.cos(a)
        z = cz_ctr + radius * math.sin(a)
        fa.append(V(x, cy - hw, z))
        fb.append(V(x, cy + hw, z))
    for i in range(segs):
        ni = (i + 1) % segs
        Q(fa[i], fa[ni], fb[ni], fb[i])
    for i in range(1, segs - 1):
        T(fa[0], fa[i], fa[i + 1])
        T(fb[0], fb[i + 1], fb[i])

# Wheel centers moved inward to match corrected body width
add_wheel( 1.40, -1.22, 0.28, 0.17, 0.28)   # front right
add_wheel( 1.40,  1.22, 0.28, 0.17, 0.28)   # front left
add_wheel(-1.40, -1.22, 0.28, 0.17, 0.28)   # rear right
add_wheel(-1.40,  1.22, 0.28, 0.17, 0.28)   # rear left

# ----------------------------------------------------------------
# OUTPUT
# ----------------------------------------------------------------
print("# E46 M3 GTR mesh -- NFS Most Wanted 2005")
print("# Proportions from real BMW M3 GTR E46 dimensions")
print("# For CS5330 Computer Vision AR project")
print(f"# {len(verts)} vertices, {len(faces)} faces")
print()
for x, y, z in verts:
    print(f"v {x} {y} {z}")
print()
last_grp = None
for face in faces:
    grp = face[0]
    if grp != last_grp:
        print(f"g {grp}")
        last_grp = grp
    print("f " + " ".join(str(i) for i in face[1:]))
