#!/usr/bin/python
# encoding: utf-8

import rds2026simulation, rds2026environment, rds2026machines
from random import randint
f = open("instrucciones.txt")
#
# robot básico con rotación aleatoria ante colisiones
#

# init system
robot = rds2026machines.vacuum_zero(position = (21, 21), orientation = 0)
simulation = rds2026simulation.simulation(
    size = (700, 700),
    fps = 1,
    environment = rds2026environment.floorplan("cfg_0.py"),
    machine = robot)

simulation.start()

# Estados
state = {
    "Follow_wall" : False,
    "Avoid_obstacle" : False,
    "Look_for_wall" : True

}

# A la hora de avanzar va de 2 en dos (ocupa 2x2)
steps = 0
while simulation.is_running:
    #
    # update world
    # press Q to quit, SPACE to stop/run and D to show/hide tiles
    #
    if state["Look_for_wall"]:
        if robot.sensor['proximity']['front']:
            robot.rotate(90)

            state["Look_for_wall"] = False
            state["Avoid_obstacle"] = True

            print("Change state Look_for_wall -> Follow_wall")

    elif state["Avoid_obstacle"]:
        robot.rotate(90)
        
        state["Avoid_obstacle"] = False
        state["Follow_wall"] = True
        
        print("Change state Avoid_obstacle -> Follow_wall")

    elif state["Follow_wall"]:
        if robot.sensor['proximity']['front']:
            robot.rotate(90)

            print("No wall -> turn")
        elif not robot.sensor['proximity']['right']:

            robot.rotate(-90)

            print("No wall -> turn")


    simulation.update()



# end
simulation.stop()

