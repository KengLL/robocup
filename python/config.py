import math

# ZMQ Ports
VISION_PORT   = 5555   # SimNode   → RobotNode + VizNode  (world state)
STRATEGY_PORT = 5556   # Reserved for autonomous strategy node (not yet implemented)
COMMAND_PORT  = 5557   # RobotNode → SimNode  (wheel commands)
MANUAL_PORT   = 5558   # VizNode   → RobotNode (manual click targets)

# Field (SSL-EV, meters)
FIELD_W = 9.0
FIELD_H = 6.0

# Unit conversion from the Godot iterative scene (test_iterative.tscn):
# field size 1240x840 px and 6.0 m height ⇒ 140 px/m.
PX_PER_METER = 140.0

# Robot hardware (translated from robot.gd, scaled to meters)
NUM_ROBOTS      = 1
ROBOT_RADIUS    = 20.0 / PX_PER_METER   # robot.gd draw radius = 20 px
ROBOT_MASS      = 0.8                   # test_iterative.tscn Robot mass
WHEEL_DISTANCE  = 15.0 / PX_PER_METER   # robot.gd wheel_distance = 15 px
MOTOR_MAX_FORCE = 200.0 / PX_PER_METER  # robot.gd force in px-units → SI-scaled
WHEEL_ANGLES    = [0.0, 2*math.pi/3, 4*math.pi/3]

# Ball — SSL-standard specs (not currently simulated)
BALL_RADIUS = 0.043   # 43 mm diameter
BALL_MASS   = 0.046   # 46 g

# Physics damping (matches robot.gd linear_damp=3, angular_damp=3)
LINEAR_DAMP  = 3.0
ANGULAR_DAMP = 3.0

# Control gains (Dynamic Inversion — robot.gd kp=15, kd=5)
KP = 15.0
KD = 5.0

# MPC (robot.gd mpc_horizon=10, mpc_dt=0.016)
MPC_HORIZON = 10
MPC_DT      = 0.016

# Simulation timing
FPS = 60
DT  = 1.0 / FPS

# Display
DISPLAY_SCALE = 100.0
DISPLAY_W     = int(FIELD_W * DISPLAY_SCALE)   # 900
DISPLAY_H     = int(FIELD_H * DISPLAY_SCALE)   # 600

# Arrival threshold (meters)
ARRIVAL_THRESH = 5.0 / PX_PER_METER   # robot.gd arrival threshold = 5 px
