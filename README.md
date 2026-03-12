Team RonBot – e-puck Maze Controller

This guide explains how to run the Team RonBot e-puck controller in Webots for the maze competition.

Requirements

Before running the project, make sure you have:

Webots installed

The maze competition world file

The Team_RonBot folder containing the updated controller

Setup Instructions
1. Replace the Controller Folder

Open the Team_RonBot folder.

Extract the controllers folder.

Navigate to your Webots project directory.

Replace the existing controllers folder with the new controllers folder from Team_RonBot.

This ensures Webots uses the correct robot control code.

2. Open the Maze World

Launch Webots.

Open the maze competition world file.

3. Select the Controller

Click on the e-puck robot in the scene tree.

In the left sidebar, locate the Controller field.

Make sure the controller is set to:

epuck_go_forward
Running the Simulation
1. Run the Mapping Phase

Press the Play button.

Once the epuck_go_forward controller loads, the robot will begin mapping the maze.

During this phase the robot will:

Explore the maze

Map all 36 tiles

Return back to the starting position

2. Reset the Simulation

After mapping finishes:

Click the Reset Simulation button (two icons to the left of the Play button).

Press Play again.

3. Run the Solving Phase

After resetting and pressing play again, the robot will:

Use the generated map

Compute the optimal route

Solve the maze from start to finish

Backup Controller (If Problems Occur)

If the epuck_go_forward controller fails to load or the robot does not move correctly, a backup controller can be used.

Steps

Click the e-puck robot in the scene tree.

In the Controller field on the left sidebar, change the controller from:

epuck_go_forward

to the backup controller:

epuck_backup

Press Play to start the simulation again.

The backup controller provides a simplified navigation behavior and can be used for testing or troubleshooting if the main controller encounters issues.

Expected Behavior

First Run: Robot explores and maps the maze.

Second Run: Robot uses the map to solve the maze efficiently.

Notes

Ensure the controller is set correctly before starting the simulation.

If the robot does not move, confirm that the controllers folder was replaced properly.

Always reset the simulation before running the solving phase.
