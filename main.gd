# game.gd
extends Node2D
@export var robot_scene : PackedScene
var robot_count = 6
var blue_team = []
var red_team = []
@onready var ball = $Ball

# Spread supporters so they don't target the same spot
var support_offsets = [Vector2(0, -120), Vector2(0, 120)]

func _ready():
	randomize()
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
