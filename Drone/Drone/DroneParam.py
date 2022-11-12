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
t_end = 60.0
t_plot = 0.1

T_max = 1

## Plotting Parameters
rotor_radius = 0.087 # m
rotor_resolution = 100 # pts/circ

## Drone Parameters 
d = 0.2 # m
g = 9.81 # m/s^2
mc = 20.0 # kg
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
    [1/mc]
])

Crh = np.array([
    [1.0, 0.0]
])

Ahi = np.concatenate((
    np.concatenate((Ah, np.zeros((2, 1))), axis=1),
    np.concatenate((-Crh, np.array([[0.0]])), axis=1)),
    axis=0)

Bhi = np.concatenate((Bh, np.array([[0.0]])), axis=0)



# gain calculation

tr_h = 1.0

zeta_h = 0.707

h_integrator = -1.0

wn_h = 2.2/tr_h # natural frequency for position

des_char_poly_h = np.convolve([1, 2*zeta_h*wn_h, wn_h**2], np.poly(np.array([h_integrator])))

des_poles_h = np.roots(des_char_poly_h)


# H loop
if np.linalg.matrix_rank(cnt.ctrb(Ahi, Bhi)) != np.size(Ahi, 1):
    print("The system is not controllable")
    
else:
    K_temp = cnt.place(Ahi, Bhi, des_poles_h)
    Kh = K_temp[0, 0:2]
    kih = K_temp[0, 2]
    # kr1 = -np.linalg.inv(Cr1 @ np.linalg.inv(A1 - B1 @ K1) @ B1)

print("\n=== H Gains ===")
print('K:')
print(Kh)
print('Ki:')
print(kih)