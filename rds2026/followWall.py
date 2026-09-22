#!/usr/bin/python
# encoding: utf-8

from mapeado import MapaOcupacion
import rds2026simulation
import rds2026environment
import rds2026machines

COOULDOWN_FRAMES = 1

# init system
robot = rds2026machines.vacuum_zero(
    position=(21, 21),
    orientation=0
)

mapeado = MapaOcupacion()

simulation = rds2026simulation.simulation(
    size=(700, 700),
    fps=16,
    environment=rds2026environment.floorplan("cfg_0.py"),
    machine=robot
)

simulation.start()

# Estados
FOLLOW_WALL_RIGHT = "wall right"
SEARCH_WALL = "search wall"

cd = 0
state = SEARCH_WALL


# Funciones auxiliares

def front_s():
    return robot.sensor["proximity"]["front"]


def right_s():
    return robot.sensor["proximity"]["right"]


def left_s():
    return robot.sensor["proximity"]["left"]


def rotate(alpha):
    global cd

    robot.stop()
    robot.rotate(alpha)

    cd = COOULDOWN_FRAMES

    robot.start()


# Main loop
while simulation.is_running:

    # Update the world
    simulation.update()

    # Actualizar el mapa
    mapeado.Actualizar(robot)

    # No hacemos nada si se ha aplicado un CD
    if cd > 0:
        cd -= 1
        continue

    if front_s() and right_s() and left_s():
        rotate(180)

    elif state == SEARCH_WALL:

        if front_s():
            rotate(90)
            state = FOLLOW_WALL_RIGHT

        else:
            if not robot.is_running:
                robot.start()

    elif state == FOLLOW_WALL_RIGHT:

        if front_s():
            rotate(90)

        elif not right_s():
            rotate(-90)

        else:
            if not robot.is_running:
                robot.start()

mapeado.Mostrar_mapa(robot)
mapeado.Estadisticas()
simulation.stop()
