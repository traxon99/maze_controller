from controller import Robot, Camera, Motor
from pathlib import Path
from maze_solver import bfs
import math


# ------------------- Run / Maze Constants -------------------

MAPPED = False
file_path = Path("path.txt")
if file_path.exists():
    MAPPED = True

TIME_STEP = 64
MAX_SPEED = 3.0
WHEEL_RADIUS = 0.0205
AXLE_LENGTH = 0.05822365
TILE_SIZE = 0.25
GRID_ROWS = 6
GRID_COLS = 6


# ------------------- Robot Setup -------------------

robot = Robot()

rotation = 0
robot_row = 0
robot_col = 0
inverted = False
global_rotation = 0

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

left_sensor = left_motor.getPositionSensor()
right_sensor = right_motor.getPositionSensor()
left_sensor.enable(TIME_STEP)
right_sensor.enable(TIME_STEP)

camera = robot.getDevice('camera')
camera.enable(TIME_STEP)
camera.setFov(.84)

robot.step(TIME_STEP)


# ------------------- Maze Data Structures -------------------

# visited keeps track of explored tiles during mapping.
visited = [[False for _ in range(GRID_COLS)] for _ in range(GRID_ROWS)]

# graph stores the maze as an adjacency matrix for BFS.
graph = [[0] * (GRID_ROWS * GRID_COLS) for _ in range(GRID_ROWS * GRID_COLS)]

# end stores the green goal tile once it is found.
end = []


# ------------------- Coordinate / Rotation Helpers -------------------

# Return the tile directly in front of the robot based on its current global heading.
def check_coords():
    global global_rotation
    global robot_row
    global robot_col

    row = robot_row
    col = robot_col

    if global_rotation == 0:
        row += 1
    elif global_rotation == 90:
        col += 1
    elif global_rotation == 180:
        row -= 1
    elif global_rotation == 270:
        col -= 1

    return row, col


# Keep the heading normalized to 0, 90, 180, 270 style angles.
def rotation_tracker(angle):
    global global_rotation
    global_rotation = (global_rotation + angle) % 360


# Update the robot's tracked tile after a forward move.
def update_coords():
    global global_rotation
    global robot_row
    global robot_col
    global inverted

    if global_rotation == 0:
        robot_row += 1
    elif global_rotation == 90:
        robot_col += 1
    elif global_rotation == 180:
        robot_row -= 1
    elif global_rotation == 270:
        robot_col -= 1

    if robot_col < 0:
        inverted = True


# Flatten a row/column pair into a graph node index.
def coord_to_index(r, c):
    return r * GRID_COLS + c


# ------------------- Motion Helpers -------------------

# Turn the robot in place by the requested number of degrees.
def turn(angle):
    rotation_tracker(angle)

    theta = math.radians(abs(angle))
    distance = (AXLE_LENGTH / 2) * theta
    wheel_rotation = distance / WHEEL_RADIUS

    left_start = left_sensor.getValue()
    right_start = right_sensor.getValue()

    if angle > 0:
        left_target = left_start + wheel_rotation
        right_target = right_start - wheel_rotation
    else:
        left_target = left_start - wheel_rotation
        right_target = right_start + wheel_rotation

    left_motor.setPosition(left_target)
    right_motor.setPosition(right_target)
    left_motor.setVelocity(MAX_SPEED)
    right_motor.setVelocity(MAX_SPEED)

    while robot.step(TIME_STEP) != -1:
        if abs(left_sensor.getValue() - left_target) < 0.01 and abs(right_sensor.getValue() - right_target) < 0.01:
            break

    left_motor.setVelocity(0)
    right_motor.setVelocity(0)


# Drive forward a given distance. In mapping mode it stops at the end of the move.
# In racing mode it keeps the move smooth so it does not pause tile-by-tile.
def move_dist(dist, racing=False):
    start_left = left_sensor.getValue()
    start_right = right_sensor.getValue()
    target_rotation = dist / WHEEL_RADIUS

    left_target = start_left + target_rotation
    right_target = start_right + target_rotation

    left_motor.setPosition(left_target)
    right_motor.setPosition(right_target)

    if racing:
        left_motor.setVelocity((MAX_SPEED * 2) + 0.28)
        right_motor.setVelocity((MAX_SPEED * 2) + 0.28)
    else:
        left_motor.setVelocity(MAX_SPEED)
        right_motor.setVelocity(MAX_SPEED)

    while robot.step(TIME_STEP) != -1:
        if abs(left_sensor.getValue() - left_target) < 0.01 and abs(right_sensor.getValue() - right_target) < 0.01:
            break

    if not racing:
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)


# ------------------- Camera / Wall Detection -------------------

# Detect whether a white wall is in front of the robot.
def front_wall(camera):
    img_bytes = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()

    def count_white_pixels(x):
        white_count = 0

        for y in range(height):
            r = camera.imageGetRed(img_bytes, width, x, y)
            g = camera.imageGetGreen(img_bytes, width, x, y)
            b = camera.imageGetBlue(img_bytes, width, x, y)

            green_val = (g - r) / (r + g + b + 1e-8)

            if abs(green_val) <= 0.05 or green_val > 0.6 or green_val < -0.6:
                white_count += 1

        return white_count

    pixels = [count_white_pixels(width // 2)]
    print("Front wall pixels:", pixels)
    return not sum(pixels) <= 25


# Detect whether the green goal wall is in front of the robot.
def green_wall(camera):
    img_bytes = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()

    def count_green_pixels(x):
        green_count = 0

        for y in range(height):
            r = camera.imageGetRed(img_bytes, width, x, y)
            g = camera.imageGetGreen(img_bytes, width, x, y)
            b = camera.imageGetBlue(img_bytes, width, x, y)

            green_val = (g - r) / (r + g + b + 1e-8)

            if green_val > 0.6:
                green_count += 1

        return green_count

    pixels = [count_green_pixels(width // 2)]
    print(sum(pixels))
    return not sum(pixels) <= 25


# ------------------- Startup Reorientation -------------------

# If the robot starts facing a wall, rotate in 90-degree steps until an open path is found.
def reorient_if_facing_wall():
    robot.step(TIME_STEP)

    if not front_wall(camera):
        return

    print("Robot started facing a wall. Reorienting...")

    for _ in range(4):
        turn(90)
        robot.step(TIME_STEP)

        if not front_wall(camera):
            print("Open direction found. Continuing.")
            return

    print("Warning: robot could not find an open direction.")


# ------------------- DFS Mapping -------------------

# DFS explores the maze, builds the graph, and records the green goal tile.
def dfs(start=True, invert=False):
    global robot_row
    global robot_col
    global end

    curr = coord_to_index(robot_row, abs(robot_col))

    if green_wall(camera) and not end:
        end.append(robot_row)
        end.append(abs(robot_col))

    visited[robot_row][abs(robot_col)] = True

    if invert:
        angles = [0, -270, -180]
    else:
        angles = [0, 90, 180]

    for dir in angles:
        turn(dir)

        next_row, next_col = check_coords()

        if not front_wall(camera):
            if not visited[next_row][abs(next_col)]:
                move_dist(TILE_SIZE)
                update_coords()

                enter_angle = global_rotation
                dfs(start=False, invert=(not invert))

                turn(global_rotation - enter_angle)
                move_dist(TILE_SIZE)
                update_coords()

                if invert:
                    turn(180)
                else:
                    turn(-180)

            next_idx = coord_to_index(next_row, abs(next_col))
            graph[curr][next_idx] = graph[next_idx][curr] = 1


# ------------------- Race Path Helpers -------------------

# Convert coordinate path into per-step row/column differences.
def build_differences(coord_path):
    differences = []

    for i in range(len(coord_path) - 1):
        x1, y1 = coord_path[i]
        x2, y2 = coord_path[i + 1]

        dx = x2 - x1
        dy = y2 - y1
        differences.append((dx, dy))

    return differences


# Compress repeated same-direction steps so the robot can drive straight through them.
def compress_differences(differences):
    if not differences:
        return []

    compressed = []
    current = differences[0]
    count = 1

    for diff in differences[1:]:
        if diff == current:
            count += 1
        else:
            compressed.append((current[0], current[1], count))
            current = diff
            count = 1

    compressed.append((current[0], current[1], count))
    return compressed


# Turn the robot to match the direction required for the next race segment.
def orient_for_segment(dx, dy, mult):
    if dx == 1:
        if global_rotation == 0:
            return
        elif global_rotation == 90:
            turn(-90)
        elif global_rotation == 180:
            turn(180)
        elif global_rotation == 270:
            turn(90)

    elif dx == -1:
        if global_rotation == 180:
            return
        elif global_rotation == 0:
            turn(180)
        elif global_rotation == 90:
            turn(90)
        elif global_rotation == 270:
            turn(-90)

    elif dy == 1 * mult:
        if global_rotation == 0:
            turn(90)
        elif global_rotation == 90:
            return
        elif global_rotation == 180:
            turn(-90)
        elif global_rotation == 270:
            turn(180)

    elif dy == -1 * mult:
        if global_rotation == 0:
            turn(-90)
        elif global_rotation == 90:
            turn(180)
        elif global_rotation == 180:
            turn(90)
        elif global_rotation == 270:
            return


# Execute the shortest path as fast as possible by merging straight segments.
def run_race_path(coord_path, mult):
    differences = build_differences(coord_path)
    segments = compress_differences(differences)

    for dx, dy, count in segments:
        orient_for_segment(dx, dy, mult)
        move_dist(TILE_SIZE * count, True)


# ------------------- Main Controller Logic -------------------

robot.step(TIME_STEP)
reorient_if_facing_wall()

if not MAPPED:
    dfs()

    if not end:
        end.append(robot_row)
        end.append(abs(robot_col))

    with open("adj_matrix.txt", "w") as file:
        for row in graph:
            line = ",".join(str(x) for x in row)
            file.write(line + "\n")

        file.write(f"{0}, {0}\n")
        file.write(f"{end[0]}, {end[1]}\n")

        if inverted:
            file.write("T")
        else:
            file.write("F")

    search = bfs()
    search.run()

else:
    mult = 1
    coord_path = []

    with open("path.txt", "r") as file:
        lines = file.readlines()

    var = lines.pop().strip()

    if var == "T":
        mult = -1

    for line in lines:
        line = line.strip()

        if line:
            cleaned = line.strip("()")
            x_str, y_str = cleaned.split(",")
            coord_path.append((int(x_str), int(y_str)))

    run_race_path(coord_path, mult)

    left_motor.setVelocity(0)
    right_motor.setVelocity(0)

    if file_path.exists():
        file_path.unlink()
