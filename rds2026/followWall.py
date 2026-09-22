#!/usr/bin/python
# encoding: utf-8

import rds2026simulation, rds2026environment, rds2026machines
from random import randint

COOULDOWN_FRAMES = 1

# init system
robot = rds2026machines.vacuum_zero(position = (21, 21), orientation = 0)
simulation = rds2026simulation.simulation(
    size = (700, 700),
    fps = 16,
    environment = rds2026environment.floorplan("cfg_0.py"),
    machine = robot)

simulation.start()

# Estados
FOLLOW_WALL_RIGHT = "wall right"
SEARCH_WALL = "search wall"
cd = 0
state = SEARCH_WALL

## Funciones auxiliares

# Front sensor
def front_s():
    return robot.sensor["proximity"]["front"]

# Right sensor
def right_s():
    return robot.sensor["proximity"]["right"]

# Left sensor
def left_s():
    return robot.sensor["proximity"]["left"]

# Rotate alpha degrees and cd of frames
def rotate(alpha):
    global cd
    robot.stop()
    robot.rotate(alpha)
    cd = COOULDOWN_FRAMES
    robot.start()
# Main loop
while simulation.is_running:
    ## Machine State ##
    # FOLLOW_WALL_RIGHT -> advance and turn if object infront
    # SEARCH_WALL -> advance until there is a wall infront
    # 

    # Update the world
    simulation.update()

    # No hacemos nada si se ha aplicado un CD
    if cd > 0:
        cd =-1

        continue

    if front_s() and right_s() and left_s():
        rotate(180)

    # Search
    elif state == SEARCH_WALL:
        if front_s():
            rotate(90)
            state = FOLLOW_WALL_RIGHT
        else:
            if not robot.is_running:
                robot.start()

    # Follow
    elif state == FOLLOW_WALL_RIGHT:
        if front_s():
            rotate(90)
        elif not right_s():
            rotate(-90)
        else:
            if not robot.is_running:
                robot.start()
# end
simulation.stop()

