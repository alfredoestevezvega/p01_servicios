#!/usr/bin/python
# encoding: utf-8

import rds2026simulation, rds2026environment, rds2026machines
from random import randint
from Comportamientos import FollowWall

COOULDOWN_FRAMES = 1

# init system
robot = rds2026machines.vacuum_zero(position = (9, 9), orientation = 0)
simulation = rds2026simulation.simulation(
    size = (700, 700),
    fps = 200,
    environment = rds2026environment.floorplan("cfg_2.py"),
    machine = robot)

simulation.start()

# Rotate alpha degrees and cd of frames

cd = 0
def rotate(alpha):
    global cd
    robot.stop()
    robot.rotate(alpha)
    cd = COOULDOWN_FRAMES
    robot.start()
# Main loop

comp = FollowWall(robot)
action = "move"
alpha = 0
#while simulation.is_running:
while True: 
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
    action, alpha = comp.get_action_()

    if action == "rotate":
        rotate(alpha)

    elif action == "move":
        if not robot.is_running:
            robot.start() 
    elif action == "stop":
        robot.stop()  
    else:
        break
print(comp.imprimir_celdas())
# end
simulation.stop()

