# Team RonBot | e-puck Maze Controller

This guide explains how to run the **Team RonBot e-puck controller** in **Webots** for the maze competition.

## Requirements

Before running the project, make sure you have:

- **Webots** installed
- The **maze competition world file**
- The **Team_RonBot** folder containing the updated controller

## Setup Instructions

### 1. Replace the Controllers Folder

1. Open the **Team_RonBot** folder.
2. Extract the `controllers` folder.
3. Navigate to your **Webots project directory**.
4. Replace the existing `controllers` folder with the new `controllers` folder from **Team_RonBot**.

This ensures Webots uses the correct robot control code.

### 2. Open the Maze World

1. Launch **Webots**.
2. Open the **maze competition world file**.

### 3. Select the Controller

1. Click on the **e-puck** robot in the scene tree.
2. In the left sidebar, locate the **Controller** field.
3. Make sure the controller is set to:

```text
epuck_go_forward
```

## Running the Simulation

### Mapping Phase

1. Press the **Play** button.
2. Once the `epuck_go_forward` controller loads, the robot will begin mapping the maze.

During this phase, the robot will:

- Explore the maze
- Map all **36 tiles**
- Return to the starting position

### Reset the Simulation

After mapping finishes:

1. Click the **Reset Simulation** button  
   *(two icons to the left of the Play button)*.
2. Press **Play** again.

### Solving Phase

After resetting and pressing **Play** again, the robot will:

- Use the generated map
- Compute the optimal route
- Solve the maze from start to finish

## Backup Controller (**ONLY** If Problems Occur)

If the `epuck_go_forward` controller fails to load or the robot does not move correctly, a backup controller can be used.

### Steps

1. Click the **e-puck** robot in the scene tree.
2. In the **Controller** field on the left sidebar, change the controller from:

```text
epuck_go_forward
```

to the backup controller:

```text
epuck_backup
```

3. Press **Play** to start the simulation again.

The backup controller provides a simplified navigation behavior and can be used for testing or troubleshooting if the main controller encounters issues.

## Expected Behavior

- **First Run:** Robot explores and maps the maze
- **Second Run:** Robot uses the map to solve the maze efficiently
- **Backup:** Wall-Follow Oversimplified Backup to be used if all else fails

## Notes

- Ensure the controller is set correctly before starting the simulation
- If the robot does not move, confirm that the `controllers` folder was replaced properly
- Always reset the simulation before running the solving phase
