extends CharacterBody2D


const SPEED = 300.0
const JUMP_VELOCITY = -400.0

func _physics_process(_delta: float) -> void:
  var mouse_position: Vector2 = get_global_mouse_position()
  var direction: Vector2 = (mouse_position - global_position).normalized()
  $LineOfSight2D.look_at(mouse_position)
