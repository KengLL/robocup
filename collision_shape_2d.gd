# robot.gd
extends RigidBody2D
var speed = 1000
var robot_color = Color.BLUE
var ball : RigidBody2D
var team = ""
var teammates = []
var is_chasing = false
var pass_cooldown = 0.0
var role_offset = Vector2.ZERO  # Unique offset per robot to avoid stacking

func _ready():
	lock_rotation = true
	linear_damp = 15.0
	collision_layer = 2   # Robots on layer 2
	collision_mask = 3    # Robots collide with layer 1 (ball+walls) AND layer 2 (other robots)

func _draw():
	draw_circle(Vector2.ZERO, 20, robot_color)
	var direction_vector = Vector2(25, 0).rotated(visual_rotation)
	draw_line(Vector2.ZERO, direction_vector, Color.WHITE, 3)

var visual_rotation = 0.0

func _physics_process(delta):
	if not ball: return
	if pass_cooldown > 0: pass_cooldown -= delta

	var target_pos: Vector2
	var distance_to_ball = global_position.distance_to(ball.global_position)

	if is_chasing:
		target_pos = ball.global_position
		if distance_to_ball < 40:
			if pass_cooldown <= 0:
				kick_ball()
			return
	else:
		var home_x = 250 if team == "blue" else 950
		# Apply unique role_offset so supporters spread out, not stack
		target_pos = Vector2(home_x, ball.global_position.y) + role_offset

		if global_position.distance_to(target_pos) < 20:
			linear_velocity = linear_velocity.lerp(Vector2.ZERO, 0.3)
			return

	# --- Separation force: push away from nearby teammates ---
	var separation = Vector2.ZERO
	for mate in teammates:
		if mate == self: continue
		var to_me = global_position - mate.global_position
		var dist = to_me.length()
		if dist < 60 and dist > 0:  # 60px = 3x robot radius, tweak as needed
			separation += to_me.normalized() * (60.0 - dist) * 8.0

	var dir = global_position.direction_to(target_pos)
	apply_central_force(dir * speed + separation)

	visual_rotation = dir.angle()
	queue_redraw()

func kick_ball():
	var target_teammate = null
	for friend in teammates:
		if friend != self:
			target_teammate = friend
			break

	if target_teammate:
		var kick_dir = global_position.direction_to(target_teammate.global_position)
		ball.apply_central_impulse(kick_dir * 200)
		pass_cooldown = 2.0
