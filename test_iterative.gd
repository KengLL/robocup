extends Node2D

@onready var robot = $Robot

func _input(event):
	if event is InputEventMouseButton and event.pressed and event.button_index == MOUSE_BUTTON_LEFT:
		var click_pos = get_global_mouse_position()
		
		# Check if Shift is held down
		var mode = "MPC" if Input.is_key_pressed(KEY_SHIFT) else "2005_INVERSION"
		
		robot.set_target_waypoint(click_pos, mode)
		print("Commanding robot to ", click_pos, " using ", mode)
