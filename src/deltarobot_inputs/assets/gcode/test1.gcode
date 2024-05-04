G28 ; call function input_cmds__homing__publish(True)

; pick object
G0 X100 Y100 Z-200 ; call function input_cmds__move__task_space__ptp__publish(100, 100, -200, -1)
G0 X100 Y100 Z-220 ; call function input_cmds__move__task_space__ptp__publish(100, 100, -220, -1)
G1 ON ; call function input_cmds__gripper__em__publish(True)

; place object
G0 X-100 Y-100 Z-190 ; call function input_cmds__move__task_space__ptp__publish(-100, -100, -200, -1)
G1 OFF ; call function input_cmds__gripper__em__publish(False)