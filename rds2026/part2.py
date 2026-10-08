#!/usr/bin/python
# encoding: utf-8


import rds2026simulation
import rds2026environment
import rds2026machines

from DJKSITRA import DJKSITRA


COOULDOWN_FRAMES = 1


# ================================================================
# ROBOT
# ================================================================

robot = rds2026machines.vacuum_zero(
    position=(9, 9),
    orientation=0
)


# ================================================================
# SIMULACIÓN
# ================================================================

simulation = rds2026simulation.simulation(
    size=(700, 700),
    fps=15,
    environment=rds2026environment.floorplan(
        "cfg_1.py"
    ),
    machine=robot
)


simulation.start()


# ================================================================
# ROTACIÓN
# ================================================================

cd = 0


def rotate(alpha):

    global cd

    robot.stop()

    robot.rotate(alpha)

    cd = COOULDOWN_FRAMES

    robot.start()


# ================================================================
# OBJETIVO
# ================================================================

# Cambia esto por la celda a la que quieras llegar.
#
# IMPORTANTE:
# tiene que utilizar las mismas coordenadas que mapa.json

OBJETIVO = (14, 14)


# ================================================================
# CREAR DIJKSTRA
# ================================================================

comp = DJKSITRA(
    robot=robot,
    mapa_json="mapa.json",
    objetivo=OBJETIVO
)


# ================================================================
# BUCLE PRINCIPAL
# ================================================================

action = "move"
alpha = 0


while True:

    # ------------------------------------------------------------
    # ACTUALIZAR SIMULACIÓN
    # ------------------------------------------------------------

    simulation.update()


    # ------------------------------------------------------------
    # COOLDOWN
    # ------------------------------------------------------------

    if cd > 0:

        cd -= 1

        continue


    # ------------------------------------------------------------
    # PEDIR ACCIÓN AL COMPORTAMIENTO
    # ------------------------------------------------------------

    action, alpha = comp.get_action_()


    # ------------------------------------------------------------
    # GIRAR
    # ------------------------------------------------------------

    if action == "rotate":

        rotate(alpha)


    # ------------------------------------------------------------
    # MOVER
    # ------------------------------------------------------------

    elif action == "move":

        if not robot.is_running:
            robot.start()


    # ------------------------------------------------------------
    # STOP
    # ------------------------------------------------------------

    elif action == "stop":

        robot.stop()

        print("Simulación terminada")

        break


    # ------------------------------------------------------------
    # ACCIÓN DESCONOCIDA
    # ------------------------------------------------------------

    else:

        print(
            "Acción desconocida:",
            action
        )

        break


# ================================================================
# FIN
# ================================================================

simulation.stop()
