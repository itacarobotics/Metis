; G28 homing
; G0 X... Y... Z... T... (T-1 best effort)
; M5 1 gripper open
; M5 0 gripper closed

G28
G0 X0 Y0 Z-150 T3

; loop 1
G0 X100 Y50 Z-220 T-1
G0 X100 Y50 Z-242 T-1
M5 0
G0 X100 Y50 Z-220 T-1
G0 X-100 Y-50 Z-220 T-1
G0 X-100 Y-50 Z-242 T-1
M5 1
G0 X-100 Y-50 Z-220 T-1

G0 X100 Y0 Z-220 T-1
G0 X100 Y0 Z-242 T-1
M5 0
G0 X100 Y0 Z-220 T-1
G0 X-100 Y0 Z-220 T-1
G0 X-100 Y0 Z-242 T-1
M5 1
G0 X-100 Y0 Z-220 T-1

G0 X100 Y-50 Z-220 T-1
G0 X100 Y-50 Z-242 T-1
M5 0
G0 X100 Y-50 Z-220 T-1
G0 X-100 Y50 Z-220 T-1
G0 X-100 Y50 Z-242 T-1
M5 1
G0 X-100 Y50 Z-220 T-1


; loop 2
G0 X-100 Y-50 Z-220 T-1
G0 X-100 Y-50 Z-242 T-1
M5 0
G0 X-100 Y-50 Z-220 T-1
G0 X100 Y50 Z-220 T-1
G0 X100 Y50 Z-242 T-1
M5 1
G0 X100 Y50 Z-220 T-1

G0 X-100 Y0 Z-220 T-1
G0 X-100 Y0 Z-242 T-1
M5 0
G0 X-100 Y0 Z-220 T-1
G0 X100 Y0 Z-220 T-1
G0 X100 Y0 Z-242 T-1
M5 1
G0 X100 Y0 Z-220 T-1

G0 X-100 Y50 Z-220 T-1
G0 X-100 Y50 Z-242 T-1
M5 0
G0 X-100 Y50 Z-220 T-1
G0 X100 Y-50 Z-220 T-1
G0 X100 Y-50 Z-242 T-1
M5 1
G0 X100 Y-50 Z-220 T-1


; place
G0 X100 Y50 Z-220 T-1
G0 X100 Y50 Z-242 T-1
M5 0
G0 X100 Y50 Z-200 T-1
G0 X0 Y150 Z-180 T-1
M5 1

G0 X100 Y0 Z-220 T-1
G0 X100 Y0 Z-242 T-1
M5 0
G0 X100 Y0 Z-220 T-1
G0 X0 Y150 Z-180 T-1
M5 1

G0 X100 Y-50 Z-220 T-1
G0 X100 Y-50 Z-242 T-1
M5 0
G0 X100 Y-50 Z-220 T-1
G0 X0 Y150 Z-180 T-1
M5 1


G0 X0 Y0 Z-100 T3
