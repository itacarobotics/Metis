; G28 homing
; G0 X... Y... Z... T... (T-1 best effort)
; M5 1gripper open
; M5 0gripper closed

G28
G0 X0 Y0 Z-200 T6


; ******* loop 1 *******
; pick object
G0 X60 Y120 Z-232 T-1
G0 X60 Y120 Z-242 T-1
M5 0
G0 X60 Y120 Z-232 T-1

; place object
G0 X60 Y-120 Z-232 T-1
G0 X60 Y-120 Z-242 T-1
M5 1
G0 X60 Y-120 Z-232 T-1


; pick object
G0 X0 Y120 Z-232 T-1
G0 X0 Y120 Z-242 T-1
M5 0
G0 X0 Y120 Z-232 T-1

; place object
G0 X0 Y-120 Z-232 T-1
G0 X0 Y-120 Z-242 T-1
M5 1
G0 X0 Y-120 Z-232 T-1


; pick object
G0 X-60 Y120 Z-232 T-1
G0 X-60 Y120 Z-242 T-1
M5 0
G0 X-60 Y120 Z-232 T-1

; place object
G0 X-60 Y-120 Z-232 T-1
G0 X-60 Y-120 Z-242 T-1
M5 1
G0 X-60 Y-120 Z-232 T-1



; ******* loop 2 *******
; pick object
G0 X60 Y-120 Z-232 T-1
G0 X60 Y-120 Z-242 T-1
M5 0
G0 X60 Y-120 Z-232 T-1

; place object
G0 X60 Y120 Z-232 T-1
G0 X60 Y120 Z-242 T-1
M5 1
G0 X60 Y120 Z-232 T-1


; pick object
G0 X0 Y-120 Z-232 T-1
G0 X0 Y-120 Z-242 T-1
M5 0
G0 X0 Y-120 Z-232 T-1

; place object
G0 X0 Y120 Z-232 T-1
G0 X0 Y120 Z-242 T-1
M5 1
G0 X0 Y120 Z-232 T-1


; pick object
G0 X-60 Y-120 Z-232 T-1
G0 X-60 Y-120 Z-242 T-1
M5 0
G0 X-60 Y-120 Z-232 T-1

; place object
G0 X-60 Y120 Z-232 T-1
G0 X-60 Y120 Z-242 T-1
M5 1
G0 X-60 Y120 Z-232 T-1







; ******* loop 1 *******
; pick object
G0 X60 Y120 Z-232 T-1
G0 X60 Y120 Z-242 T-1
M5 0
G0 X60 Y120 Z-232 T-1

; place object
G0 X60 Y-120 Z-232 T-1
G0 X60 Y-120 Z-242 T-1
M5 1
G0 X60 Y-120 Z-232 T-1


; pick object
G0 X0 Y120 Z-232 T-1
G0 X0 Y120 Z-242 T-1
M5 0
G0 X0 Y120 Z-232 T-1

; place object
G0 X0 Y-120 Z-232 T-1
G0 X0 Y-120 Z-242 T-1
M5 1
G0 X0 Y-120 Z-232 T-1


; pick object
G0 X-60 Y120 Z-232 T-1
G0 X-60 Y120 Z-242 T-1
M5 0
G0 X-60 Y120 Z-232 T-1

; place object
G0 X-60 Y-120 Z-232 T-1
G0 X-60 Y-120 Z-242 T-1
M5 1
G0 X-60 Y-120 Z-232 T-1



; ******* loop 2 *******
; pick object
G0 X60 Y-120 Z-232 T-1
G0 X60 Y-120 Z-242 T-1
M5 0
G0 X60 Y-120 Z-232 T-1

; place object
G0 X60 Y120 Z-232 T-1
G0 X60 Y120 Z-242 T-1
M5 1
G0 X60 Y120 Z-232 T-1


; pick object
G0 X0 Y-120 Z-232 T-1
G0 X0 Y-120 Z-242 T-1
M5 0
G0 X0 Y-120 Z-232 T-1

; place object
G0 X0 Y120 Z-232 T-1
G0 X0 Y120 Z-242 T-1
M5 1
G0 X0 Y120 Z-232 T-1


; pick object
G0 X-60 Y-120 Z-232 T-1
G0 X-60 Y-120 Z-242 T-1
M5 0
G0 X-60 Y-120 Z-232 T-1

; place object
G0 X-60 Y120 Z-232 T-1
G0 X-60 Y120 Z-242 T-1
M5 1
G0 X-60 Y120 Z-232 T-1






; ******* loop 1 *******
; pick object
G0 X60 Y120 Z-232 T-1
G0 X60 Y120 Z-242 T-1
M5 0
G0 X60 Y120 Z-232 T-1

; place object
G0 X60 Y-120 Z-232 T-1
G0 X60 Y-120 Z-242 T-1
M5 1
G0 X60 Y-120 Z-232 T-1


; pick object
G0 X0 Y120 Z-232 T-1
G0 X0 Y120 Z-242 T-1
M5 0
G0 X0 Y120 Z-232 T-1

; place object
G0 X0 Y-120 Z-232 T-1
G0 X0 Y-120 Z-242 T-1
M5 1
G0 X0 Y-120 Z-232 T-1


; pick object
G0 X-60 Y120 Z-232 T-1
G0 X-60 Y120 Z-242 T-1
M5 0
G0 X-60 Y120 Z-232 T-1

; place object
G0 X-60 Y-120 Z-232 T-1
G0 X-60 Y-120 Z-242 T-1
M5 1
G0 X-60 Y-120 Z-232 T-1



; ******* loop 2 *******
; pick object
G0 X60 Y-120 Z-232 T-1
G0 X60 Y-120 Z-242 T-1
M5 0
G0 X60 Y-120 Z-232 T-1

; place object
G0 X60 Y120 Z-232 T-1
G0 X60 Y120 Z-242 T-1
M5 1
G0 X60 Y120 Z-232 T-1


; pick object
G0 X0 Y-120 Z-232 T-1
G0 X0 Y-120 Z-242 T-1
M5 0
G0 X0 Y-120 Z-232 T-1

; place object
G0 X0 Y120 Z-232 T-1
G0 X0 Y120 Z-242 T-1
M5 1
G0 X0 Y120 Z-232 T-1


; pick object
G0 X-60 Y-120 Z-232 T-1
G0 X-60 Y-120 Z-242 T-1
M5 0
G0 X-60 Y-120 Z-232 T-1

; place object
G0 X-60 Y120 Z-232 T-1
G0 X-60 Y120 Z-242 T-1
M5 1
G0 X-60 Y120 Z-232 T-1






; ******* loop 1 *******
; pick object
G0 X60 Y120 Z-232 T-1
G0 X60 Y120 Z-242 T-1
M5 0
G0 X60 Y120 Z-232 T-1

; place object
G0 X60 Y-120 Z-232 T-1
G0 X60 Y-120 Z-242 T-1
M5 1
G0 X60 Y-120 Z-232 T-1


; pick object
G0 X0 Y120 Z-232 T-1
G0 X0 Y120 Z-242 T-1
M5 0
G0 X0 Y120 Z-232 T-1

; place object
G0 X0 Y-120 Z-232 T-1
G0 X0 Y-120 Z-242 T-1
M5 1
G0 X0 Y-120 Z-232 T-1


; pick object
G0 X-60 Y120 Z-232 T-1
G0 X-60 Y120 Z-242 T-1
M5 0
G0 X-60 Y120 Z-232 T-1

; place object
G0 X-60 Y-120 Z-232 T-1
G0 X-60 Y-120 Z-242 T-1
M5 1
G0 X-60 Y-120 Z-232 T-1



; ******* loop 2 *******
; pick object
G0 X60 Y-120 Z-232 T-1
G0 X60 Y-120 Z-242 T-1
M5 0
G0 X60 Y-120 Z-232 T-1

; place object
G0 X60 Y120 Z-232 T-1
G0 X60 Y120 Z-242 T-1
M5 1
G0 X60 Y120 Z-232 T-1


; pick object
G0 X0 Y-120 Z-232 T-1
G0 X0 Y-120 Z-242 T-1
M5 0
G0 X0 Y-120 Z-232 T-1

; place object
G0 X0 Y120 Z-232 T-1
G0 X0 Y120 Z-242 T-1
M5 1
G0 X0 Y120 Z-232 T-1


; pick object
G0 X-60 Y-120 Z-232 T-1
G0 X-60 Y-120 Z-242 T-1
M5 0
G0 X-60 Y-120 Z-232 T-1

; place object
G0 X-60 Y120 Z-232 T-1
G0 X-60 Y120 Z-242 T-1
M5 1
G0 X-60 Y120 Z-232 T-1
