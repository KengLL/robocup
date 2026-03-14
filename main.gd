# game.gd
extends Node2D
@export var robot_scene : PackedScene
var robot_count = 6
var blue_team = []
var red_team = []
var red_score = 0
var blue_score = 0
@onready var ball = $Ball
@onready var score_label = $CanvasLayer/Score

# Spread supporters so they don't target the same spot
var support_offsets = [Vector2(0, -120), Vector2(0, 120)]

#Testing purposes only
func _input(event):
	if Input.is_key_pressed(KEY_1):
		ball.linear_velocity = Vector2.ZERO
		ball.position = Vector2(620, 420)
		ball.apply_impulse(Vector2(-800, 0))  # shoot left toward left goal
	if Input.is_key_pressed(KEY_2):
		ball.linear_velocity = Vector2.ZERO
		ball.position = Vector2(620, 420)
		ball.apply_impulse(Vector2(800, 0))   # shoot right toward right goal


func _ready():
	randomize()
	score_label.text = "Blue: 0  Red: 0" 
	ball.position = Vector2(620, 420)
	var blue_support_idx = 0
	var red_support_idx = 0

	for i in robot_count:
		var robot = robot_scene.instantiate()
		robot.ball = ball

		if i < 3:
			robot.robot_color = Color.BLUE
			robot.team = "blue"
			blue_team.append(robot)
			# Assign offset BEFORE adding to scene
			if blue_support_idx < support_offsets.size():
				robot.role_offset = support_offsets[blue_support_idx]
				blue_support_idx += 1
		else:
			robot.robot_color = Color.RED
			robot.team = "red"
			red_team.append(robot)
			if red_support_idx < support_offsets.size():
				robot.role_offset = support_offsets[red_support_idx]
				red_support_idx += 1

		robot.position = Vector2(randf_range(200, 1000), randf_range(200, 600))
		add_child(robot)

func _physics_process(_delta):
	update_team_roles(blue_team)
	update_team_roles(red_team)

func update_team_roles(team_array):
	if team_array.is_empty(): return

	var closest_robot = team_array[0]
	var min_dist = closest_robot.global_position.distance_to(ball.global_position)

	for robot in team_array:
		var dist = robot.global_position.distance_to(ball.global_position)
		if dist < min_dist:
			min_dist = dist
			closest_robot = robot
		robot.is_chasing = false
		robot.teammates = team_array

	closest_robot.is_chasing = true



func scored_goal_left(body: Node2D) -> void:
	if body != ball:return
	blue_score += 1
	score_label.text = "Blue: " + str(blue_score) + "  Red: " + str(red_score)
	await reset_ball()
	await get_tree().create_timer(0.2).timeout

func scored_goal_right(body: Node2D) -> void:
	if body != ball:return
	red_score += 1
	score_label.text = "Blue: " + str(blue_score) + "  Red: " + str(red_score)
	await reset_ball()
	await get_tree().create_timer(0.2).timeout
	
func reset_ball():
	print("before reset:", ball.global_position) #leave here, doesn't work without it
	ball.call_deferred("set", "freeze", true)
	ball.linear_velocity = Vector2.ZERO
	ball.angular_velocity = 0.0
	ball.global_position = Vector2(620, 420)
	print("after reset:", ball.global_position) #timing issues?
	await get_tree().physics_frame
	print("one frame later:", ball.global_position)
	ball.call_deferred("set", "freeze", false)
	
