extends CharacterBody2D

var speed = 200
var direction = Vector2.ZERO
var change_time = 0

func _ready():
	randomize()
	pick_new_direction()

func pick_new_direction():
	direction = Vector2(
		randf_range(-1,1),
		randf_range(-1,1)
	).normalized()

	change_time = randf_range(1,3)

func _physics_process(delta):

	change_time -= delta

	if change_time <= 0:
		pick_new_direction()

	velocity = direction * speed
	move_and_slide()
