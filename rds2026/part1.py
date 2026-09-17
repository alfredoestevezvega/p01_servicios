#!/usr/bin/python
# encoding: utf-8

import rds2026simulation, rds2026environment, rds2026machines
from random import randint

#
# robot básico con rotación aleatoria ante colisiones
#

# init system
robot = rds2026machines.vacuum_zero(position = (21, 21), orientation = 0)
simulation = rds2026simulation.simulation(
    size = (700, 700),
    fps = 15,
    environment = rds2026environment.floorplan("cfg_0.py"),
    machine = robot)

simulation.start()
n = 1
m = 0
while simulation.is_running:
    #
    # update world
    # press Q to quit, SPACE to stop/run and D to show/hide tiles
    #

    # Me he movido m passos de n

    # El robot se mueve
    simulation.update()
    if m == n:
        robot.rotate(90)
        n += 1
        m = 0
    m += 1

# end
simulation.stop()

