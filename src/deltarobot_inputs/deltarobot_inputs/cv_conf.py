
SHOW_FRAME      = True
SHOW_MARKERS    = True
SHOW_EDGES      = True
SHOW_OBJECTS    = True

## markers position on base
img_dst_prescaler = 5   # use prescaler to increase image resolution
WS_HEIGHT = int(100*img_dst_prescaler)  # [ mm ]
WS_LENGTH = int(150*img_dst_prescaler)  # [ mm ]

## workplane
z_offset = 10
z_work_plane = -209

washer_thickness = 2

## marker position
marker0_x = 75
marker0_y = -25

## boxes
box0_x = 37.5
box0_y = 75
box0_z = -170

box1_x = -37.5
box1_y = 75
box1_z = -170


## idle position
idle_pos_x = 0
idle_pos_y = 50
idle_pos_z = -150