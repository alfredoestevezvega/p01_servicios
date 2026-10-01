#!/usr/bin/python
# encoding: utf-8

import json
import heapq


class DJKSITRA:

    def __init__(self, robot, mapa_json, objetivo):
        self.robot = robot
        self.objetivo = objetivo

        # Mapa:
        # 0 -> obstáculo
        # 1 -> celda visitada/libre
        # 2 -> celda libre pendiente de explorar
        self.map = {}

        self.ruta = None
        self.indice_ruta = 1

        self.cargar_mapa_(mapa_json)


    # ============================================================
    # CARGAR MAPA JSON
    # ============================================================

    def cargar_mapa_(self, archivo):

        with open(archivo, "r", encoding="utf-8") as f:
            datos = json.load(f)

        for posicion in datos["posiciones"]:

            x = posicion["x"]
            y = posicion["y"]
            valor = posicion["valor"]

            self.map[(x, y)] = valor


    # ============================================================
    # POSICIÓN LÓGICA DEL ROBOT
    # ============================================================

    def posicion_robot_(self):
        """
        El robot ocupa 2x2.

        Usamos como posición lógica la esquina superior izquierda.
        """

        cells = self.robot.sensor["cells"]

        min_x = min(x for x, y in cells)
        min_y = min(y for x, y in cells)

        return (min_x, min_y)


    # ============================================================
    # CELDAS QUE OCUPARÍA EL ROBOT
    # ============================================================

    def celdas_robot_en_(self, x, y):

        return {
            (x, y),
            (x + 1, y),
            (x, y + 1),
            (x + 1, y + 1)
        }


    # ============================================================
    # POSICIÓN VÁLIDA
    # ============================================================

    def posicion_valida_(self, x, y):

        for cell in self.celdas_robot_en_(x, y):

            # No conocemos esa celda
            if cell not in self.map:
                return False

            # Obstáculo
            if self.map[cell] == 0:
                return False

        return True


    # ============================================================
    # VECINOS
    # ============================================================

    def vecinos_(self, posicion):

        x, y = posicion

        movimientos = [
            (1, 0),      # derecha
            (-1, 0),     # izquierda
            (0, -1),     # arriba
            (0, 1)       # abajo
        ]

        vecinos = []

        for dx, dy in movimientos:

            nx = x + dx
            ny = y + dy

            if self.posicion_valida_(nx, ny):
                vecinos.append((nx, ny))

        return vecinos


    # ============================================================
    # DIJKSTRA
    # ============================================================

    def calcular_ruta_(self, inicio, objetivo):

        print()
        print("========== DIJKSTRA ==========")
        print("Inicio:", inicio)
        print("Objetivo:", objetivo)

        # Comprobamos que la posición inicial sea válida
        if not self.posicion_valida_(*inicio):
            print("La posición inicial no es válida")
            return None

        cola = []

        heapq.heappush(
            cola,
            (0, inicio)
        )

        distancias = {
            inicio: 0
        }

        anterior = {
            inicio: None
        }

        final = None

        while cola:

            distancia_actual, actual = heapq.heappop(cola)

            # Puede haber entradas antiguas en el heap
            if distancia_actual != distancias[actual]:
                continue

            # ----------------------------------------------------
            # OBJETIVO
            #
            # Basta con que una de las cuatro celdas ocupadas
            # por el robot sea la celda objetivo.
            # ----------------------------------------------------

            if objetivo in self.celdas_robot_en_(*actual):

                final = actual
                break

            # ----------------------------------------------------
            # EXPANDIMOS
            # ----------------------------------------------------

            for vecino in self.vecinos_(actual):

                coste = 1

                nueva_distancia = (
                    distancia_actual
                    + coste
                )

                if (
                    vecino not in distancias
                    or
                    nueva_distancia < distancias[vecino]
                ):

                    distancias[vecino] = nueva_distancia

                    anterior[vecino] = actual

                    heapq.heappush(
                        cola,
                        (
                            nueva_distancia,
                            vecino
                        )
                    )

        # --------------------------------------------------------
        # NO EXISTE RUTA
        # --------------------------------------------------------

        if final is None:

            print(
                "No existe ruta hasta",
                objetivo
            )

            return None

        # --------------------------------------------------------
        # RECONSTRUIR RUTA
        # --------------------------------------------------------

        ruta = []

        actual = final

        while actual is not None:

            ruta.append(actual)

            actual = anterior[actual]

        ruta.reverse()

        print(
            "Ruta encontrada con",
            len(ruta) - 1,
            "movimientos"
        )

        print(ruta)

        print("==============================")
        print()

        return ruta


    # ============================================================
    # ORIENTACIÓN NECESARIA PARA UN PASO
    # ============================================================

    def orientacion_hacia_(self, actual, siguiente):

        dx = siguiente[0] - actual[0]
        dy = siguiente[1] - actual[1]

        # Tu sistema:
        #
        #        90
        #         ↑
        # 180 ← robot → 0
        #         ↓
        #        270

        if dx > 0:
            return 0

        if dx < 0:
            return 180

        if dy < 0:
            return 90

        if dy > 0:
            return 270

        return self.robot.sensor["orientation"]


    # ============================================================
    # NORMALIZAR GIRO
    # ============================================================

    def normalizar_giro_(self, giro):

        giro = (
            (giro + 180) % 360
        ) - 180

        # Para mantener giros de 180 positivos
        if giro == -180:
            giro = 180

        return giro


    # ============================================================
    # COMPORTAMIENTO PRINCIPAL
    # ============================================================

    def get_action_(self):

        # --------------------------------------------------------
        # CALCULAMOS RUTA SOLO LA PRIMERA VEZ
        # --------------------------------------------------------

        if self.ruta is None:

            inicio = self.posicion_robot_()

            self.ruta = self.calcular_ruta_(
                inicio,
                self.objetivo
            )

            if self.ruta is None:

                print("No se puede alcanzar el objetivo")

                return "stop", 0

            # ruta[0] es donde estamos
            self.indice_ruta = 1


        # --------------------------------------------------------
        # YA HEMOS TERMINADO
        # --------------------------------------------------------

        if self.indice_ruta >= len(self.ruta):

            print("OBJETIVO ALCANZADO")

            return "stop", 0


        # --------------------------------------------------------
        # POSICIÓN ACTUAL REAL
        # --------------------------------------------------------

        actual = self.posicion_robot_()

        siguiente = self.ruta[
            self.indice_ruta
        ]


        # --------------------------------------------------------
        # Si ya hemos alcanzado el siguiente nodo, avanzamos índice
        # --------------------------------------------------------

        if actual == siguiente:

            self.indice_ruta += 1

            if self.indice_ruta >= len(self.ruta):

                print("OBJETIVO ALCANZADO")

                return "stop", 0

            siguiente = self.ruta[
                self.indice_ruta
            ]


        # --------------------------------------------------------
        # ORIENTACIÓN
        # --------------------------------------------------------

        orientacion_deseada = (
            self.orientacion_hacia_(
                actual,
                siguiente
            )
        )

        orientacion_actual = (
            self.robot.sensor[
                "orientation"
            ]
        )

        giro = (
            orientacion_deseada
            - orientacion_actual
        )

        giro = self.normalizar_giro_(
            giro
        )


        print(
            "Robot:",
            actual,
            "->",
            siguiente,
            "| ori:",
            orientacion_actual,
            "->",
            orientacion_deseada,
            "| giro:",
            giro
        )


        # --------------------------------------------------------
        # PRIMERO GIRAR
        # --------------------------------------------------------

        if giro != 0:

            return "rotate", giro


        # --------------------------------------------------------
        # SEGURIDAD EXTRA
        #
        # Si el mapa dice que está libre pero el sensor ve algo,
        # paramos para evitar colisión.
        # --------------------------------------------------------

        front = self.robot.sensor[
            "proximity"
        ]["front"]

        if front:

            print(
                "OBSTÁCULO DETECTADO "
                "DELANTE DURANTE LA RUTA"
            )

            return "stop", 0


        # --------------------------------------------------------
        # AVANZAR
        # --------------------------------------------------------

        return "move", 0