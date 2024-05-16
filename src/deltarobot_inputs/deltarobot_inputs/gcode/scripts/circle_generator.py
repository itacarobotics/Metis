import math
from deltarobot import configuration as conf
from os.path import join

def generate_gcode_circle(radius, resolution):
    gcode = []
    # Start position
    gcode.append(f"G28")
    gcode.append(f"G0 X0 Y0 Z-200 T-1")
    # Generate circle points
    for i in range(0, resolution+1):
        theta = i * (2 * math.pi / resolution)
        x = radius * math.cos(theta)
        y = radius * math.sin(theta)
        gcode.append(f"G0 X{x} Y{y} Z-200 T-1")
    # End position (back to starting point)
    gcode.append(f"G0 X0 Y0 Z-200 T-1")
    return gcode


def write_gcode_to_file(gcode, filepath):
    gcode_file = open(filepath, 'w')
    for line in gcode:
        gcode_file.write(line + "\n")
    gcode_file.close()


if __name__ == "__main__":
    radius = 50  # Radius of the circle
    resolution = 100  # Number of points to approximate the circle
    gcode = generate_gcode_circle(radius, resolution)
    
    file_name = "circle.gcode"
    file_path = join(conf.ws_path, "src/deltarobot_inputs/assets/gcode")
    file_path = join(file_path, file_name)
    
    write_gcode_to_file(gcode, file_path)
