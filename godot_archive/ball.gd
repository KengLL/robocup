# ball.gd
extends RigidBody2D
func _ready():
	linear_damp = 1.5
	angular_damp = 2.0
	gravity_scale = 0.0
	collision_layer = 1   # Ball is on layer 1
	collision_mask = 3    # Ball collides with layer 1 (walls) AND layer 2 (robots)
