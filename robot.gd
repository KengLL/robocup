extends RigidBody2D

var target_pos = Vector2.ZERO
var start_pos = Vector2.ZERO
var control_mode = "2005_INVERSION"
var has_target = false

# --- Hardware Specs ---
var wheel_distance = 15.0
var motor_max_force = 80.0
var wheel_angles = [0.0, deg_to_rad(120.0), deg_to_rad(240.0)]

# --- 2005 Dynamic Inversion Gains ---
var kp = 15.0  # Proportional gain (Error correction)
var kd = 5.0   # Derivative gain (Damping/Velocity correction)

# --- MPC Parameters ---
var mpc_horizon = 10     # How many frames to look ahead
var mpc_dt = 0.016       # Simulated delta time per horizon step

func _ready():
	linear_damp = 3.0
	angular_damp = 3.0
	z_index = 10

func set_target_waypoint(new_target: Vector2, mode: String):
	target_pos = new_target
	start_pos = global_position  # Record the starting point!
	control_mode = mode
	has_target = true

func _physics_process(delta):
	if not has_target: return
	
	if global_position.distance_to(target_pos) < 10.0:
		has_target = false
		return

	var vx = 0.0
	var vy = 0.0
	var w = 0.0

	# ==========================================
	# ALGORITHM ROUTING
	# ==========================================
	if control_mode == "2005_INVERSION":
		var cmd = calculate_dynamic_inversion()
		vx = cmd[0]
		vy = cmd[1]
		w = cmd[2]
	elif control_mode == "MPC":
		var cmd = calculate_mpc_rollout()
		vx = cmd[0]
		vy = cmd[1]
		w = cmd[2]

	# ==========================================
	# INVERSE KINEMATICS (Apply to wheels)
	# ==========================================
	var commanded_wheel_speeds = [0.0, 0.0, 0.0]
	var max_calc_speed = 0.0
	
	for i in range(3):
		var alpha = wheel_angles[i]
		var speed = -sin(alpha) * vx + cos(alpha) * vy + (wheel_distance * w)
		commanded_wheel_speeds[i] = speed
		if abs(speed) > max_calc_speed: max_calc_speed = abs(speed)
			
	# Actuator Saturation limits control inputs to u_max
	if max_calc_speed > 1.0:
		for i in range(3): commanded_wheel_speeds[i] /= max_calc_speed

	# Forward Kinematics (Apply simulated Godot forces)
	var total_force_local = Vector2.ZERO
	var total_torque = 0.0
	for i in range(3):
		var force_mag = commanded_wheel_speeds[i] * motor_max_force
		var drive_angle = wheel_angles[i] + (PI / 2.0)
		total_force_local += Vector2(cos(drive_angle), sin(drive_angle)) * force_mag
		total_torque += force_mag * wheel_distance
		
	apply_central_force(total_force_local.rotated(rotation))
	apply_torque(total_torque)
	queue_redraw()


# ---------------------------------------------------------
# ALGORITHM 1: DYNAMIC INVERSION (From 2005/2016 Paper)
# ---------------------------------------------------------
func calculate_dynamic_inversion() -> Array:
	# 1. Calculate Error (e(t))
	var error = target_pos - global_position
	
	# 2. Imposed Error Dynamics: \ddot{e}(t) + C\dot{e}(t) + Ke(t) = 0
	# We want to find the desired acceleration to close the gap.
	var desired_accel = (error * kp) - (linear_velocity * kd)
	
	# Rotate desired global acceleration into the robot's local frame
	var local_accel = desired_accel.rotated(-rotation)
	
	# Desired rotational velocity (always try to face angle 0)
	var desired_omega = (0.0 - rotation) * 10.0
	
	return [local_accel.x, local_accel.y, desired_omega]


# ---------------------------------------------------------
# ALGORITHM 2: GREEDY MPC ROLLOUT
# ---------------------------------------------------------
func calculate_mpc_rollout() -> Array:
	# Define a discrete set of possible movement inputs (local space)
	# (Forward, Backward, Left, Right, Diagonals)
	var possible_inputs = [
		Vector2(1, 0), Vector2(-1, 0), Vector2(0, 1), Vector2(0, -1),
		Vector2(0.7, 0.7), Vector2(-0.7, 0.7), Vector2(0.7, -0.7), Vector2(-0.7, -0.7)
	]
	
	var best_input = Vector2.ZERO
	var lowest_cost = INF
	
	# Test every possible input to see which yields the best future state
	for input in possible_inputs:
		var simulated_pos = global_position
		var simulated_vel = linear_velocity
		
		# Rollout the physics model into the future
		for step in range(mpc_horizon):
			# Globalize the local input
			var global_force = input.rotated(rotation) * motor_max_force * 2.0
			
			# Basic Euler integration simulating Godot's physics
			var simulated_accel = (global_force / mass) - (simulated_vel * linear_damp)
			simulated_vel += simulated_accel * mpc_dt
			simulated_pos += simulated_vel * mpc_dt
			
		# The cost function: How far is the simulated future position from the target?
		var cost = simulated_pos.distance_squared_to(target_pos)
		
		if cost < lowest_cost:
			lowest_cost = cost
			best_input = input
			
	# Return the best local vector found
	return [best_input.x, best_input.y, (0.0 - rotation) * 10.0]

func _draw():
	# Draw the main chassis (circle)
	draw_circle(Vector2.ZERO, 20, Color.DARK_GRAY)
	
	# Draw a white line to indicate the "Front" of the robot
	draw_line(Vector2.ZERO, Vector2(25, 0), Color.WHITE, 3)
	
	# Draw the 3 wheels based on our wheel_angles
	for i in range(3):
		var angle = wheel_angles[i]
		# Find the position of the wheel at the edge of the robot
		var wheel_pos = Vector2(cos(angle), sin(angle)) * wheel_distance
		
		# We want to draw a small rectangle for the wheel. 
		# Omni wheels apply force perpendicular to their mount angle.
		var wheel_rect = Rect2(-4, -8, 8, 16) # Width 8, Height 16
		
		# Godot's draw_set_transform lets us easily rotate shapes
		# We rotate the wheel so it sits tangentially on the circular chassis
		draw_set_transform(wheel_pos, angle + (PI / 2.0), Vector2.ONE)
		draw_rect(wheel_rect, Color.RED)
		
	# Reset the transform so other things draw normally
	draw_set_transform(Vector2.ZERO, 0.0, Vector2.ONE)
	
	if has_target:
		# Convert global fixed points to the robot's local moving/rotating space
		var local_start = to_local(start_pos)
		var local_target = to_local(target_pos)
		
		# Draw a dashed line from start to target (Ideal Path)
		# Arguments: Start, End, Color, Width, Dash Length
		draw_dashed_line(local_start, local_target, Color.YELLOW, 2.0, 10.0)
		
		# Draw a "Pin" at the destination (An 'X' marker and a circle)
		draw_circle(local_target, 5.0, Color.GREEN)
		
		# Draw an 'X' over the target to make it highly visible
		var pin_size = 10.0
		draw_line(local_target + Vector2(-pin_size, -pin_size), local_target + Vector2(pin_size, pin_size), Color.GREEN, 2.0)
		draw_line(local_target + Vector2(pin_size, -pin_size), local_target + Vector2(-pin_size, pin_size), Color.GREEN, 2.0)

	# Reset the transform (just in case)
	draw_set_transform(Vector2.ZERO, 0.0, Vector2.ONE)

	
