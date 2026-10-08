#!/usr/bin/python
# encoding: utf-8

import json
import pygame

import rds2026simulation
import rds2026environment
import rds2026machines

from DJKSITRA import DJKSITRA


# ================================================================
# CONFIGURACIÓN
# ================================================================

ARCHIVO_WAYPOINTS = "waypoints.json"

COOLDOWN_FRAMES = 1


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
    fps=60,
    environment=rds2026environment.floorplan(
        "cfg_2.py"
    ),
    machine=robot
)


simulation.start()

# empezamos parados
robot.stop()


# ================================================================
# VARIABLES
# ================================================================

# lista de puntos guardados por el usuario
waypoints = []


# modos:
#
# "manual"       -> controla el usuario
# "reproduccion" -> el robot sigue los waypoints automáticamente

modo = "manual"


# waypoint que estamos recorriendo
indice_waypoint = 0


# comportamiento Dijkstra actual
comp = None


# cooldown para los giros
cd = 0


# ================================================================
# OBTENER POSICIÓN LÓGICA DEL ROBOT
# ================================================================

def posicion_robot():
    """
    Devuelve la posición lógica del robot.

    Como el robot ocupa 2x2, utilizamos la esquina
    superior izquierda de las cuatro celdas ocupadas.
    """

    cells = robot.sensor["cells"]

    min_x = min(x for x, y in cells)
    min_y = min(y for x, y in cells)

    return (min_x, min_y)


# ================================================================
# GUARDAR WAYPOINT
# ================================================================

def guardar_waypoint():

    posicion = posicion_robot()

    # evitamos guardar dos veces seguidas el mismo punto
    if len(waypoints) > 0:

        if waypoints[-1] == posicion:

            print(
                "Waypoint repetido:",
                posicion
            )

            return

    waypoints.append(posicion)

    print()
    print("==============================")
    print("WAYPOINT GUARDADO")
    print("Número:", len(waypoints))
    print("Posición:", posicion)
    print("==============================")
    print()


# ================================================================
# GUARDAR RUTA EN FICHERO
# ================================================================

def guardar_ruta():

    datos = {
        "waypoints": []
    }

    for x, y in waypoints:

        datos["waypoints"].append(
            [x, y]
        )

    with open(
        ARCHIVO_WAYPOINTS,
        "w",
        encoding="utf-8"
    ) as f:

        json.dump(
            datos,
            f,
            indent=4
        )

    print()
    print("==============================")
    print("RUTA GUARDADA")
    print("Archivo:", ARCHIVO_WAYPOINTS)
    print("Waypoints:", len(waypoints))
    print("==============================")
    print()


# ================================================================
# CARGAR RUTA
# ================================================================

def cargar_ruta():

    global waypoints

    try:

        with open(
            ARCHIVO_WAYPOINTS,
            "r",
            encoding="utf-8"
        ) as f:

            datos = json.load(f)

    except FileNotFoundError:

        print()
        print(
            "No existe",
            ARCHIVO_WAYPOINTS
        )
        print()

        return False


    waypoints = []

    for punto in datos["waypoints"]:

        waypoints.append(
            (
                punto[0],
                punto[1]
            )
        )

    print()
    print("==============================")
    print("RUTA CARGADA")
    print("Waypoints:", len(waypoints))
    print("==============================")

    for i, punto in enumerate(waypoints):

        print(
            i + 1,
            "->",
            punto
        )

    print("==============================")
    print()

    return True


# ================================================================
# ROTACIÓN AUTOMÁTICA
# ================================================================

def rotate(alpha):

    global cd

    robot.stop()

    robot.rotate(alpha)

    cd = COOLDOWN_FRAMES

    robot.start()


# ================================================================
# EMPEZAR REPRODUCCIÓN
# ================================================================

def empezar_reproduccion():

    global modo
    global indice_waypoint
    global comp


    # cargamos primero la ruta guardada
    if not cargar_ruta():

        return


    if len(waypoints) == 0:

        print("No hay waypoints guardados")

        return


    robot.stop()

    indice_waypoint = 0


    print()
    print("==============================")
    print("INICIANDO REPRODUCCIÓN")
    print("==============================")
    print()


    comp = DJKSITRA(
        robot=robot,
        mapa_json="mapa.json",
        objetivo=waypoints[
            indice_waypoint
        ]
    )


    modo = "reproduccion"


# ================================================================
# SIGUIENTE WAYPOINT
# ================================================================

def siguiente_waypoint():

    global indice_waypoint
    global comp
    global modo


    indice_waypoint += 1


    # ------------------------------------------------------------
    # TODOS LOS WAYPOINTS COMPLETADOS
    # ------------------------------------------------------------

    if indice_waypoint >= len(waypoints):

        robot.stop()

        modo = "manual"

        comp = None

        print()
        print("==============================")
        print("RUTA COMPLETADA")
        print("==============================")
        print()

        return


    # ------------------------------------------------------------
    # NUEVO DIJKSTRA
    # ------------------------------------------------------------

    objetivo = waypoints[
        indice_waypoint
    ]


    print()
    print("==============================")
    print(
        "SIGUIENTE WAYPOINT:",
        indice_waypoint + 1,
        "/",
        len(waypoints)
    )
    print("Objetivo:", objetivo)
    print("==============================")
    print()


    comp = DJKSITRA(
        robot=robot,
        mapa_json="mapa.json",
        objetivo=objetivo
    )


# ================================================================
# INFORMACIÓN
# ================================================================

print()
print("======================================")
print("       PARTE 3 - TELEOPERACIÓN")
print("======================================")
print()
print("FLECHA ARRIBA : avanzar")
print("FLECHA IZQ.   : girar izquierda")
print("FLECHA DER.   : girar derecha")
print()
print("W             : guardar waypoint")
print("S             : guardar ruta")
print("R             : reproducir ruta")
print("Q             : salir")
print()
print("======================================")
print()


# ================================================================
# BUCLE PRINCIPAL
# ================================================================

while simulation.is_running:


    # ============================================================
    # MODO MANUAL
    # ============================================================

    if modo == "manual":


        # --------------------------------------------------------
        # LEER TECLADO
        # --------------------------------------------------------

        for event in simulation.read_keyboard():


            # ----------------------------------------------------
            # TECLA PULSADA
            # ----------------------------------------------------

            if event.type == pygame.KEYDOWN:


                # -----------------------------------------------
                # GIRAR IZQUIERDA
                # -----------------------------------------------

                if event.key == pygame.K_LEFT:

                    robot.rotate(15)


                # -----------------------------------------------
                # GIRAR DERECHA
                # -----------------------------------------------

                elif event.key == pygame.K_RIGHT:

                    robot.rotate(-15)


                # -----------------------------------------------
                # AVANZAR
                # -----------------------------------------------

                elif event.key == pygame.K_UP:

                    robot.start()


                # -----------------------------------------------
                # GUARDAR WAYPOINT
                # -----------------------------------------------

                elif event.key == pygame.K_w:

                    guardar_waypoint()


                # -----------------------------------------------
                # GUARDAR RUTA
                # -----------------------------------------------

                elif event.key == pygame.K_s:

                    robot.stop()

                    guardar_ruta()


                # -----------------------------------------------
                # REPRODUCIR
                # -----------------------------------------------

                elif event.key == pygame.K_r:

                    robot.stop()

                    empezar_reproduccion()


                # -----------------------------------------------
                # DEBUG
                # -----------------------------------------------

                elif event.key == pygame.K_d:

                    simulation.screen[
                        "debugging"
                    ] = not simulation.screen[
                        "debugging"
                    ]


                # -----------------------------------------------
                # SALIR
                # -----------------------------------------------

                elif event.key == pygame.K_q:

                    simulation.is_running = False

                    break


            # ----------------------------------------------------
            # TECLA LIBERADA
            # ----------------------------------------------------

            elif event.type == pygame.KEYUP:


                # al soltar arriba paramos
                if event.key == pygame.K_UP:

                    robot.stop()


            # ----------------------------------------------------
            # CERRAR VENTANA
            # ----------------------------------------------------

            elif event.type == pygame.QUIT:

                simulation.is_running = False


        # --------------------------------------------------------
        # ACTUALIZAR SIMULACIÓN
        # --------------------------------------------------------

        simulation.update()


    # ============================================================
    # MODO REPRODUCCIÓN
    # ============================================================

    elif modo == "reproduccion":


        # --------------------------------------------------------
        # ACTUALIZAR SIMULACIÓN
        # --------------------------------------------------------

        simulation.update()


        # --------------------------------------------------------
        # COOLDOWN
        # --------------------------------------------------------

        if cd > 0:

            cd -= 1

            continue


        # --------------------------------------------------------
        # OBTENER ACCIÓN DE DIJKSTRA
        # --------------------------------------------------------

        action, alpha = comp.get_action_()


        # --------------------------------------------------------
        # GIRAR
        # --------------------------------------------------------

        if action == "rotate":

            rotate(alpha)


        # --------------------------------------------------------
        # AVANZAR
        # --------------------------------------------------------

        elif action == "move":

            if not robot.is_running:

                robot.start()


        # --------------------------------------------------------
        # WAYPOINT ALCANZADO
        # --------------------------------------------------------

        elif action == "stop":

            robot.stop()

            print(
                "Waypoint",
                indice_waypoint + 1,
                "alcanzado:",
                waypoints[
                    indice_waypoint
                ]
            )

            siguiente_waypoint()


        # --------------------------------------------------------
        # ACCIÓN DESCONOCIDA
        # --------------------------------------------------------

        else:

            robot.stop()

            print(
                "Acción desconocida:",
                action
            )

            modo = "manual"


# ================================================================
# FIN
# ================================================================

simulation.stop()
