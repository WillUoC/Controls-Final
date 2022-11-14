import numpy as np
import control as cnt


def line_from_points(x1, y1, x2, y2):
    slope = (y2-y1)/(x2-x1)
    intercept = y1 - slope * x1

    return(slope, intercept)

# Inital Conditions
x0 = 0
y0 = 0
z0 = 0
theta0 = 0
alpha0 = 0
psi0 = 0
xdot0 = 0
ydot0 = 0
zdot0 = 0
thetadot0 = 0
alphadot0 = 0
psidot0 = 0

Ts = 0.01
t_start = 0.0
t_end = 130.0
t_plot = 0.1

T_max = 1
max_angle = 15*np.pi/180

## Plotting Parameters
rotor_radius = 0.087 # m
rotor_resolution = 100 # pts/circ

## Drone Parameters 
d = 0.2 # m
g = 9.81 # m/s^2
mm = 0.287
mc = 20.0 - 4*mm # kg
rc = 0.5
jc = 2/5*mc*rc**2

Fe = mc*g

## Aero 2 - System ID Parameters 
mu_r = 0.01
mu_lat = 0.0001
mu_lon = 0.0001

# Throttle -> Thrust data points from table (relatively linear)
# (0.4, 2.26)
# (1.0, 9.122)

# Throttle -> RPM (converted to rad/s) data points from table (also linear)
# (0.4, 1900*2*sym.pi/60)
# (1.0, 3673*2*sym.pi/60)

m_thrust, b_thrust = line_from_points(0.4, 2.26*g, 1.0, 9.122*g)
m_rot, b_rot = line_from_points(0.4, (1900*2*np.pi/60), 1.0, (3673*2*np.pi/60))

### PID Stuff

## H Loop Dynamics
Ah = np.array([
    [0.0, 1.0],
    [0.0, 0.0]
])

Bh = np.array([
    [0.0],
    [1/(mc + 4*mm)]
])

Crh = np.array([
    [1.0, 0.0]
])

Ahi = np.concatenate((
    np.concatenate((Ah, np.zeros((2, 1))), axis=1),
    np.concatenate((-Crh, np.array([[0.0]])), axis=1)),
    axis=0)

Bhi = np.concatenate((Bh, np.array([[0.0]])), axis=0)


## Yaw Loop Dynamics
Apsi = np.array([
    [0.0, 1.0],
    [0.0, 0.0]
])

Bpsi = np.array([
    [0.0],
    [1/(jc + 4*mm*d**2)]
])

Crpsi = np.array([
    [1.0, 0.0]
])

Aipsi = np.concatenate((
    np.concatenate((Apsi, np.zeros((2, 1))), axis=1),
    np.concatenate((-Crpsi, np.array([[0.0]])), axis=1)),
    axis=0)

Bipsi = np.concatenate((Bpsi, np.array([[0.0]])), axis=0)

## Pitch Loop Dynamics
Aal = np.array([
    [0.0, 1.0],
    [0.0, 0.0]
])

Bal = np.array([
    [0.0],
    [1/(jc + 4*mm*(d*np.sqrt(2)/2)**2)]
])

Cral = np.array([
    [1.0, 0.0]
])

Aial = np.concatenate((
    np.concatenate((Aal, np.zeros((2, 1))), axis=1),
    np.concatenate((-Cral, np.array([[0.0]])), axis=1)),
    axis=0)

Bial = np.concatenate((Bal, np.array([[0.0]])), axis=0)


## Roll Loop Dynamics
Ath = np.array([
    [0.0, 1.0],
    [0.0, 0.0]
])

Bth = np.array([
    [0.0],
    [1/(jc + 4*mm*(d*np.sqrt(2)/2)**2)]
])

Crth = np.array([
    [1.0, 0.0]
])

Aith = np.concatenate((
    np.concatenate((Ath, np.zeros((2, 1))), axis=1),
    np.concatenate((-Crth, np.array([[0.0]])), axis=1)),
    axis=0)

Bith = np.concatenate((Bth, np.array([[0.0]])), axis=0)

## x Loop Dynamics
Ax = np.array([
    [0.0, 1.0],
    [0.0, -mu_lat/(mc + 4*mm)]
])

Bx = np.array([
    [0.0],
    [g]
])

Crx = np.array([
    [1.0, 0.0]
])

Aix = np.concatenate((
    np.concatenate((Ax, np.zeros((2, 1))), axis=1),
    np.concatenate((-Crx, np.array([[0.0]])), axis=1)),
    axis=0)

Bix = np.concatenate((Bx, np.array([[0.0]])), axis=0)

## y Loop Dynamics
Ay = np.array([
    [0.0, 1.0],
    [0.0, -mu_lat/(mc + 4*mm)]
])

By = np.array([
    [0.0],
    [-g]
])

Cry = np.array([
    [1.0, 0.0]
])

Aiy = np.concatenate((
    np.concatenate((Ay, np.zeros((2, 1))), axis=1),
    np.concatenate((-Cry, np.array([[0.0]])), axis=1)),
    axis=0)

Biy = np.concatenate((By, np.array([[0.0]])), axis=0)

## gain calculation
tr_h = 0.1
zeta_h = 0.707

tr_psi = 2.0
zeta_psi = 0.707

tr_al = 0.1
zeta_al = 0.707

tr_th = 0.1
zeta_th = 0.707

tr_x = 1.0
zeta_x = 0.707

tr_y = 1.0
zeta_y = 0.707

h_integrator = -1.0
psi_integrator = -1.0
al_integrator = -1.0
th_integrator = -1.0
x_integrator = -1.0
y_integrator = -1.0


wn_h = 2.2/tr_h # natural frequency for position
wn_psi = 2.2/tr_psi
wn_al = 2.2/tr_al
wn_th = 2.2/tr_th
wn_x = 2.2/tr_x
wn_y = 2.2/tr_y


des_char_poly_h = np.convolve([1, 2*zeta_h*wn_h, wn_h**2], np.poly(np.array([h_integrator])))
des_poles_h = np.roots(des_char_poly_h)
des_poles_h2 = np.roots([1, 2*zeta_h*wn_h, wn_h**2])

des_char_poly_psi = np.convolve([1, 2*zeta_psi*wn_psi, wn_psi**2], np.poly(np.array([psi_integrator])))
des_poles_psi = np.roots(des_char_poly_psi)
des_poles_psi2 = np.roots([1, 2*zeta_psi*wn_psi, wn_psi**2])

des_char_poly_al = np.convolve([1, 2*zeta_al*wn_al, wn_al**2], np.poly(np.array([al_integrator])))
des_poles_al = np.roots(des_char_poly_al)
des_poles_al2 = np.roots([1, 2*zeta_al*wn_al, wn_al**2])

des_char_poly_th = np.convolve([1, 2*zeta_th*wn_th, wn_th**2], np.poly(np.array([th_integrator])))
des_poles_th = np.roots(des_char_poly_th)
des_poles_th2 = np.roots([1, 2*zeta_th*wn_th, wn_th**2])

des_char_poly_x = np.convolve([1, 2*zeta_x*wn_x, wn_x**2], np.poly(np.array([x_integrator])))
des_poles_x = np.roots(des_char_poly_x)

des_char_poly_y = np.convolve([1, 2*zeta_y*wn_y, wn_y**2], np.poly(np.array([y_integrator])))
des_poles_y = np.roots(des_char_poly_y)

# H loop
if np.linalg.matrix_rank(cnt.ctrb(Ahi, Bhi)) != np.size(Ahi, 1):
    print("The system is not controllable")
    
else:
    K_temp = cnt.place(Ahi, Bhi, des_poles_h)
    Kh = K_temp[0, 0:2]
    kih = K_temp[0, 2]
    
    Kh2 = cnt.place(Ah, Bh, des_poles_h2)
    krh = -1.0/(Crh @ np.linalg.inv(Ah - Bh @ np.reshape(Kh, (1, 2))) @ Bh)

print("\n=== H Gains ===")
print('K:')
print(Kh)
print('K2:')
print(Kh2)
print('Ki:')
print(kih)
print('Kr')
print(krh)


# Psi loop
if np.linalg.matrix_rank(cnt.ctrb(Aipsi, Bipsi)) != np.size(Aipsi, 1):
    print("The system is not controllable")
else:
    K_temp = cnt.place(Aipsi, Bipsi, des_poles_psi)
    Kpsi = K_temp[0, 0:2]
    kipsi = K_temp[0, 2]
    Kpsi2 = cnt.place(Apsi, Bpsi, des_poles_psi2)
    krpsi = -1.0/(Crpsi @ np.linalg.inv(Apsi - Bpsi @ np.reshape(Kpsi, (1, 2))) @ Bpsi)

print("\n=== Psi Gains ===")
print('K:')
print(Kpsi)
print('K2:')
print(Kpsi2)
print('Ki:')
print(kipsi)
print('Kr')
print(krpsi)

# Alpha loop
if np.linalg.matrix_rank(cnt.ctrb(Aial, Bial)) != np.size(Aial, 1):
    print("The system is not controllable")
else:
    K_temp = cnt.place(Aial, Bial, des_poles_al)
    Kal = K_temp[0, 0:2]
    kial = K_temp[0, 2]
    Kal2 = cnt.place(Aal, Bal, des_poles_al2)
    kral = -1.0/(Cral @ np.linalg.inv(Aal - Bal @ np.reshape(Kal, (1, 2))) @ Bal)

print("\n=== Alpha Gains ===")
print('K:')
print(Kal)
print('K2:')
print(Kal2)
print('Ki:')
print(kial)
print('Kr')
print(kral)

# Theta loop
if np.linalg.matrix_rank(cnt.ctrb(Aith, Bith)) != np.size(Aith, 1):
    print("The system is not controllable")
else:
    K_temp = cnt.place(Aith, Bith, des_poles_th)
    Kth = K_temp[0, 0:2]
    kith = K_temp[0, 2]
    Kth2 = cnt.place(Ath, Bth, des_poles_th2)
    krth = -1.0/(Crth @ np.linalg.inv(Ath - Bth @ np.reshape(Kth, (1, 2))) @ Bth)

print("\n=== Theta Gains ===")
print('K:')
print(Kth)
print('K2:')
print(Kth2)
print('Ki:')
print(kith)
print('Kr')
print(krth)

# x loop
if np.linalg.matrix_rank(cnt.ctrb(Aix, Bix)) != np.size(Aix, 1):
    print("The system is not controllable")
else:
    K_temp = cnt.place(Aix, Bix, des_poles_x)
    Kx = K_temp[0, 0:2]
    kix = K_temp[0, 2]
    krx = -1.0/(Crx @ np.linalg.inv(Ax - Bx @ np.reshape(Kx, (1, 2))) @ Bx)

print("\n=== X Gains ===")
print('K:')
print(Kx)
print('Ki:')
print(kix)
print('Kr')
print(krx)

# y loop
if np.linalg.matrix_rank(cnt.ctrb(Aiy, Biy)) != np.size(Aiy, 1):
    print("The system is not controllable")
else:
    K_temp = cnt.place(Aiy, Biy, des_poles_y)
    Ky = K_temp[0, 0:2]
    kiy = K_temp[0, 2]
    kry = -1.0/(Cry @ np.linalg.inv(Ay - By @ np.reshape(Ky, (1, 2))) @ By)

print("\n=== Y Gains ===")
print('K:')
print(Ky)
print('Ki:')
print(kiy)
print('Kr')
print(kry)