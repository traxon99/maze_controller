from controller import Robot, Camera, Motor, LED
from pathlib import Path
from maze_solver import bfs
import math


MAPPED = False
file_path = Path("path.txt")
if file_path.exists():
    MAPPED = True
    
TIME_STEP = 64
MAX_SPEED = 3.0
WHEEL_RADIUS = 0.0205
#I just messed with the numbers until it was a decent right angle   
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

led = robot.getDevice('led8')
  
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))


left_sensor = left_motor.getPositionSensor()
right_sensor = right_motor.getPositionSensor()
left_sensor.enable(TIME_STEP)
right_sensor.enable(TIME_STEP)
robot.step(TIME_STEP)


camera = robot.getDevice('camera')
camera.enable(TIME_STEP)
camera.setFov(.84)

#Array to tell which tiles we visited 
visited = [[False for _ in range(GRID_COLS)] for _ in range(GRID_ROWS)]
#Adj matrix to store graph 
graph = [[0]* (GRID_ROWS*GRID_COLS) for _ in range(GRID_ROWS*GRID_COLS)]

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
 
def rotation_tracker(angle):
    global global_rotation
    global_rotation = (global_rotation + angle) % 360
    
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
    robot_col = robot_col
        
#Turns a given amount of degrees
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
        if abs(left_sensor.getValue()-left_target)<0.01 and abs(right_sensor.getValue()-right_target)<0.01:
            break

    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    
#Move distance in meters 
def move_dist(dist,racing = False):
    start_left = left_sensor.getValue()
    start_right = right_sensor.getValue()
    target_rotation = dist / WHEEL_RADIUS

    left_motor.setPosition(start_left + target_rotation)
    right_motor.setPosition(start_right + target_rotation)
    left_motor.setVelocity((MAX_SPEED*2)+.28)
    right_motor.setVelocity((MAX_SPEED*2)+.28)

    while robot.step(TIME_STEP) != -1:
        if abs(left_sensor.getValue()-(start_left+target_rotation))<0.01 and abs(right_sensor.getValue()-(start_right+target_rotation))<0.01:
            break
    if not racing:
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)



#Detect white front wall 
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

            if abs(green_val) <= .05 or green_val > .6 or green_val < -.6:
                white_count += 1
        return white_count

    pixels = [count_white_pixels(width//2)]
    print("Front wall pixels:", pixels)
    return not sum(pixels) <= 25

        
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

            if green_val > .6:
                green_count += 1
        return green_count

    pixels = [count_green_pixels(width//2)]
    print(sum(pixels))
    return not sum(pixels) <= 25
 

def coord_to_index(r,c):
    return r*GRID_COLS + c
  
end = []

def dfs(start=True, invert= False):
    global robot_row, robot_col
    curr = coord_to_index(robot_row, abs(robot_col))
    if green_wall(camera):
        end.append(robot_row)
        end.append(abs(robot_col))
        

    visited[robot_row][abs(robot_col)] = True 
    if invert:
        angles = [0, -270, -180]
    else:
        angles = [0,90,180]  
    for dir in angles: 

        turn(dir)

        # Check front
        next_row, next_col = check_coords()
        if not front_wall(camera):
            if not visited[next_row][abs(next_col)]:
                move_dist(TILE_SIZE)
                update_coords()
                enter_angle = global_rotation
                dfs(start=False,invert = (not invert)) 
                turn(global_rotation-enter_angle)
                move_dist(TILE_SIZE)
                update_coords()
                if invert:
                    turn(180)
                else:
                    turn(-180)
            next_idx = coord_to_index(next_row, abs(next_col))
            graph[curr][next_idx] = graph[next_idx][curr] = 1

robot.step(TIME_STEP)
if not MAPPED:

    dfs()
    #Write out the adj matrix to a file
    with open("adj_matrix.txt", "w") as file:
        for row in graph:
            line = ",".join(str(x) for x in row) 
            file.write(line + "\n")
    #Pass the start and end coords into bfs as tuples
    #Also pass in if we need to invert
    search = bfs((0,0),(end[0], end[1]), inverted)
    #Run the optimization algo 
    search.run()

else:
    #Tracks if we need to invert the y coords 
    mult = 1
    coord_path = []
    #Read in the path 
    with open("path.txt", "r") as file:
        lines = file.readlines()
    #Pop the inversion off the end of the text file 
    var = lines.pop()
    #Check if we need inversion 
    if var == "True\n":
        mult = -1
    #Read the rest of the lines, removing some of the formating 
    for line in lines:
        line = line.strip() 
        if line:
            x_str, y_str = line.strip("()").split(",")
            coord_path.append((int(x_str), int(y_str)))
                
    
    #Find of next tile offset from where we are           
    differences = []
    for i in range(len(coord_path) - 1):
        x1, y1 = coord_path[i]
        x2, y2 = coord_path[i + 1]
        dx = x2 - x1
        dy = y2 - y1
        differences.append((dx, dy))
    #Default distance is one tile 
    dist = .25
    #Hard coded turning
    for i in range(len(differences)):
        if i == len(differences) - 1:
            dist = 2
        x, y = differences[i]
        if x == 1:
            if global_rotation == 0:
                move_dist(dist,True)
            elif global_rotation == 90:
                turn(-90)
                move_dist(dist,True)
            elif global_rotation == 180:
                turn(180)
                move_dist(dist,True)
            elif global_rotation == 270:
                turn(90)
                move_dist(dist,True)
        if x == -1:
            if global_rotation == 180:
                move_dist(dist,True)
            elif global_rotation == 0:
                turn(180)
                move_dist(dist,True)
            elif global_rotation == 90:
                turn(90)
                move_dist(dist,True)
            elif global_rotation == 270:
                turn(-90)
                move_dist(dist,True)  
        if y == 1 * mult:
            if global_rotation == 0:
                turn(90)
                move_dist(dist,True)
            elif global_rotation == 90:
                move_dist(dist,True)
            elif global_rotation == 180:
                turn(-90)
                move_dist(dist,True)
            elif global_rotation == 270:
                turn(180)
                move_dist(dist,True)
                
        if y == -1 * mult:
            if global_rotation == 0:
                turn(-90)
                move_dist(dist,True)
            elif global_rotation == 90:
                turn(180)
                move_dist(dist,True)
            elif global_rotation == 180:
                turn(90)
                move_dist(dist,True)
            elif global_rotation == 270:
                move_dist(dist,True)
    #Delete the path file   
    if file_path.exists():
        file_path.unlink()
        
    
    