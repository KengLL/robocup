extends Node2D

@onready var robot = $Robot

func _input(event):
	if event is InputEventMouseButton and event.pressed and event.button_index == MOUSE_BUTTON_LEFT:
		var click_pos = get_global_mouse_position()
		
		var mode = "2005_INVERSION"
		# Check if Shift is held down
		if Input.is_key_pressed(KEY_SHIFT):
			mode = "MPC"
		elif Input.is_key_pressed(KEY_META):
			mode = "2005_TIME_OPTIMAL"
			
		robot.set_target_waypoint(click_pos, mode)
		print("Commanding robot to ", click_pos, " using ", mode)
