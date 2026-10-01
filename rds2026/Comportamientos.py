from collections import deque
import json
import heapq

class FollowWall:

    def __init__(self, robot):
        self.robot = robot

        self.front = self.robot.sensor["proximity"]["front"]
        self.left = self.robot.sensor["proximity"]["left"]
        self.right = self.robot.sensor["proximity"]["right"]

        self.state = "search wall"

        # 1 -> celda visitada
        # 2 -> celda interior todavía no visitada
        self.map = {}

        self.a_visitar = set()
        self.cercana = None

        # Movimientos bloqueados conocidos.
        #
        # Ejemplo:
        # ((2,3), (3,3))
        #
        # significa que el robot no puede mover su posición
        # lógica de (2,3) a (3,3).
        self.blocked_moves = set()

        # Para saber cuándo hemos terminado de seguir la pared
        self.pto_inicio = None
        self.wall_steps = 0


    # ============================================================
    # ACCIÓN PRINCIPAL
    # ============================================================

    def get_action_(self):

        self._update()

        # Marcamos siempre las 4 celdas actuales como visitadas
        for cell in self.robot.sensor["cells"]:
            self.map[cell] = 1

        # Guardamos las paredes/obstáculos que estamos viendo
        self.registrar_obstaculos_()

        # --------------------------------------------------------
        # BUSCAR PARED
        # --------------------------------------------------------

        if self.state == "search wall":

            if self.front:
                self.state = "wall right"

                self.pto_inicio = self.posicion_robot_()
                self.wall_steps = 0

                # Giramos a la izquierda
                return "rotate", 90

            return "move", 0


        # --------------------------------------------------------
        # SEGUIR PARED POR LA DERECHA
        # --------------------------------------------------------

        elif self.state == "wall right":

            posicion_actual = self.posicion_robot_()

            # Hemos vuelto al punto donde empezamos
            if (
                self.wall_steps > 0
                and posicion_actual == self.pto_inicio
            ):
                print("Perímetro cerrado")

                self.update_map_()

                self.state = "EXPLORE"
                self.cercana = None

                self.imprimir_celdas()

                return "stop", 0

            # Si ya no tenemos pared a la derecha,
            # giramos hacia ella.
            if not self.right:
                return "rotate", -90

            # Si tenemos algo delante, giramos a la izquierda.
            if self.front:
                return "rotate", 90

            # Podemos avanzar
            self.wall_steps += 1
            return "move", 0


        # --------------------------------------------------------
        # EXPLORAR INTERIOR
        # --------------------------------------------------------

        elif self.state == "EXPLORE":

            # Las posiciones actuales pasan a ser visitadas
            for cell in self.robot.sensor["cells"]:
                self.map[cell] = 1

            self.get_unexplored_()

            if not self.a_visitar:
                print("Exploración terminada")
                return "ACABO", 0

            # En vez de coger simplemente la más cercana por Manhattan,
            # buscamos directamente la más cercana QUE TENGA RUTA.
            ruta, objetivo = self.buscar_objetivo_()

            if ruta is None:
                print("No existe ninguna ruta conocida hacia los 2")

                # Marcamos como obstáculo los 2 que no son alcanzables
                for celda in list(self.a_visitar):
                    self.map[celda] = 0

                return "ACABE", 0
            self.cercana = objetivo

            print("Objetivo:", objetivo)
            print("Ruta:", ruta)

            # Ya estamos ocupando la celda objetivo
            if len(ruta) <= 1:

                for cell in self.robot.sensor["cells"]:
                    self.map[cell] = 1

                self.cercana = None

                return "stop", 0

            # Solo nos interesa EL SIGUIENTE paso del camino
            actual = ruta[0]
            siguiente = ruta[1]

            orientacion_deseada = self.orientacion_hacia_posicion(
                actual,
                siguiente
            )

            orientacion_actual = self.robot.sensor["orientation"]

            giro = orientacion_deseada - orientacion_actual

            # Normalizar a [-180, 180]
            giro = ((giro + 180) % 360) - 180

            # Preferimos 180 antes que -180
            if giro == -180:
                giro = 180

            print(
                "Robot:",
                actual,
                "->",
                siguiente,
                "| orientación:",
                orientacion_actual,
                "->",
                orientacion_deseada,
                "| giro:",
                giro
            )

            # Primero giramos
            if giro != 0:
                return "rotate", giro

            # ----------------------------------------------------
            # SEGURIDAD:
            # aunque BFS diga que podemos ir, comprobamos el sensor
            # actual antes de movernos.
            # ----------------------------------------------------

            if self.front:
                print("BFS quería avanzar, pero hay pared delante")

                dx = siguiente[0] - actual[0]
                dy = siguiente[1] - actual[1]

                self.bloquear_movimiento_(
                    actual,
                    (actual[0] + dx, actual[1] + dy)
                )

                # En la siguiente iteración BFS calculará otra ruta
                return "stop", 0

            return "move", 0

        return "stop", 0


    # ============================================================
    # BFS
    # ============================================================

    def buscar_objetivo_(self):
        """
        BFS desde la posición actual del robot.

        Devuelve:
            ruta, objetivo

        Busca directamente el 2 alcanzable más cercano.
        """

        inicio = self.posicion_robot_()

        cola = deque([inicio])

        anterior = {
            inicio: None
        }

        movimientos = [
            (1, 0),    # derecha
            (-1, 0),   # izquierda
            (0, -1),   # arriba
            (0, 1)     # abajo
        ]

        posicion_final = None
        objetivo_final = None

        while cola:

            actual = cola.popleft()

            x, y = actual

            # ----------------------------------------------------
            # ¿Alguna de las 4 celdas del robot está sobre un 2?
            # ----------------------------------------------------

            for cell in self.celdas_robot_en(x, y):

                if cell in self.a_visitar:
                    posicion_final = actual
                    objetivo_final = cell
                    break

            if posicion_final is not None:
                break

            # ----------------------------------------------------
            # Expandir vecinos
            # ----------------------------------------------------

            for dx, dy in movimientos:

                nx = x + dx
                ny = y + dy

                siguiente = (nx, ny)

                if siguiente in anterior:
                    continue

                if not self.puede_mover(x, y, dx, dy):
                    continue

                anterior[siguiente] = actual
                cola.append(siguiente)

        # No hemos encontrado ningún 2 alcanzable
        if posicion_final is None:
            return None, None

        # --------------------------------------------------------
        # RECONSTRUIR RUTA
        # --------------------------------------------------------

        ruta = []

        actual = posicion_final

        while actual is not None:

            ruta.append(actual)

            actual = anterior[actual]

        ruta.reverse()

        return ruta, objetivo_final


    # ============================================================
    # COMPROBAR MOVIMIENTO
    # ============================================================
    def puede_mover(self, x, y, dx, dy):

        actual = (x, y)

        siguiente = (
            x + dx,
            y + dy
        )

        # Si sabemos que existe una pared entre ambas posiciones
        if (actual, siguiente) in self.blocked_moves:
            return False

        nuevas_celdas = self.celdas_robot_en(
            siguiente[0],
            siguiente[1]
        )

        for cell in nuevas_celdas:

            # No permitimos entrar en una zona que no conocemos
            if cell not in self.map:
                return False

            # No puede ocupar una celda con obstáculo
            if self.map[cell] == 0:
                return False

        # MUY IMPORTANTE
        return True
    def celdas_delante_(self, orientacion):
        x, y = self.posicion_robot_()

        if orientacion == 0:      # derecha
            return {
                (x + 2, y),
                (x + 2, y + 1)
            }

        elif orientacion == 90:   # arriba
            return {
                (x, y - 1),
                (x + 1, y - 1)
            }

        elif orientacion == 180:  # izquierda
            return {
                (x - 1, y),
                (x - 1, y + 1)
            }

        elif orientacion == 270:  # abajo
            return {
                (x, y + 2),
                (x + 1, y + 2)
            }

        return set()
    # ============================================================
    # GUARDAR OBSTÁCULOS
    # ============================================================
    def registrar_obstaculos_(self):
        orientacion = self.robot.sensor["orientation"]
        posicion = self.posicion_robot_()

        sensores = [
            (self.front, orientacion),
            (self.left, (orientacion + 90) % 360),
            (self.right, (orientacion - 90) % 360)
        ]

        for detectado, ori in sensores:

            if not detectado:
                continue

            # Marcamos las celdas físicas como obstáculos
            for cell in self.celdas_delante_(ori):
                self.map[cell] = 0

            # Marcamos también ese movimiento como bloqueado
            dx, dy = self.vector_orientacion_(ori)

            siguiente = (
                posicion[0] + dx,
                posicion[1] + dy
            )

            self.bloquear_movimiento_(
                posicion,
                siguiente
            )
    def bloquear_direccion_(self, posicion, direccion):

        x, y = posicion
        dx, dy = direccion

        siguiente = (
            x + dx,
            y + dy
        )

        self.bloquear_movimiento_(
            posicion,
            siguiente
        )


    def bloquear_movimiento_(self, a, b):
        """
        Si no puedo pasar A -> B, tampoco puedo B -> A.
        """

        self.blocked_moves.add((a, b))
        self.blocked_moves.add((b, a))


    # ============================================================
    # DIRECCIONES / ORIENTACIONES
    # ============================================================

    def vector_orientacion_(self, orientacion):

        orientacion %= 360

        if orientacion == 0:
            return (1, 0)       # derecha

        if orientacion == 90:
            return (0, -1)      # arriba

        if orientacion == 180:
            return (-1, 0)      # izquierda

        if orientacion == 270:
            return (0, 1)       # abajo

        raise ValueError(
            f"Orientación desconocida: {orientacion}"
        )


    def orientacion_hacia_posicion(self, actual, siguiente):

        dx = siguiente[0] - actual[0]
        dy = siguiente[1] - actual[1]

        if dx > 0:
            return 0

        if dx < 0:
            return 180

        if dy < 0:
            return 90

        if dy > 0:
            return 270

        # Estamos en la misma posición
        return self.robot.sensor["orientation"]


    # ============================================================
    # POSICIÓN DEL ROBOT
    # ============================================================

    def posicion_robot_(self):
        """
        Representamos el robot 2x2 mediante la esquina
        superior izquierda de las cuatro celdas que ocupa.
        """

        cells = self.robot.sensor["cells"]

        min_x = min(x for x, y in cells)
        min_y = min(y for x, y in cells)

        return (min_x, min_y)


    def celdas_robot_en(self, x, y):
        """
        Celdas ocupadas por un robot 2x2 si su esquina
        superior izquierda está en (x,y).
        """

        return {
            (x, y),
            (x + 1, y),
            (x, y + 1),
            (x + 1, y + 1)
        }


    def centro_robot_(self):

        cells = self.robot.sensor["cells"]

        suma_x = sum(x for x, y in cells)
        suma_y = sum(y for x, y in cells)

        n = len(cells)

        return (
            suma_x / n,
            suma_y / n
        )


    # ============================================================
    # MAPA
    # ============================================================

    def get_unexplored_(self):

        self.a_visitar = {
            posicion
            for posicion, valor in self.map.items()
            if valor == 2
        }

    def update_map_(self):
        """
        Valores del mapa:
            0 -> obstáculo
            1 -> celda visitada por el robot
            2 -> celda interior pendiente de explorar

        Las posiciones visitadas con valor 1 forman el perímetro.
        Las posiciones encerradas por ese perímetro pasan a valor 2,
        excepto las que ya sean obstáculos (0).
        """

        visited = {
            pos
            for pos, value in self.map.items()
            if value == 1
        }

        if not visited:
            return

        xs = [x for x, y in visited]
        ys = [y for x, y in visited]

        # Dejamos un margen alrededor para poder recorrer el exterior
        min_x = min(xs) - 1
        max_x = max(xs) + 1

        min_y = min(ys) - 1
        max_y = max(ys) + 1

        exterior = set()
        stack = [(min_x, min_y)]

        # --------------------------------------------------------
        # FLOOD FILL DESDE EL EXTERIOR
        # --------------------------------------------------------
        while stack:
            x, y = stack.pop()
            pos = (x, y)

            if pos in exterior:
                continue

            if not (min_x <= x <= max_x):
                continue

            if not (min_y <= y <= max_y):
                continue

            # Las posiciones visitadas forman la frontera
            if pos in visited:
                continue

            exterior.add(pos)

            stack.extend([
                (x + 1, y),
                (x - 1, y),
                (x, y + 1),
                (x, y - 1)
            ])

        # --------------------------------------------------------
        # TODO LO QUE NO ES EXTERIOR ESTÁ DENTRO
        # --------------------------------------------------------
        for y in range(min_y + 1, max_y):
            for x in range(min_x + 1, max_x):

                pos = (x, y)

                # Visitada -> se queda como 1
                if pos in visited:
                    continue

                # Exterior -> no pertenece a la habitación
                if pos in exterior:
                    continue

                # Obstáculo conocido -> se queda como 0
                if self.map.get(pos) == 0:
                    continue

                # El resto es interior aún no explorado
                self.map[pos] = 2
    # ============================================================
    # OTROS MÉTODOS
    # ============================================================

    def manhatam(self, a0, b0, a1, b1):

        return (
            abs(a0 - a1)
            + abs(b0 - b1)
        )


    def next_pose(self):

        orientacion = self.robot.sensor["orientation"]

        if orientacion == 0:

            pose = self.robot.sensor["cells"][1]

            return (
                pose[0] + 1,
                pose[1]
            )

        elif orientacion == 90:

            pose = self.robot.sensor["cells"][0]

            return (
                pose[0],
                pose[1] - 1
            )

        elif orientacion == 180:

            pose = self.robot.sensor["cells"][2]

            return (
                pose[0] - 1,
                pose[1]
            )

        elif orientacion == 270:

            pose = self.robot.sensor["cells"][3]

            return (
                pose[0],
                pose[1] + 1
            )

        raise ValueError(
            f"Orientación desconocida: {orientacion}"
        )


    def _update(self):

        self.front = self.robot.sensor["proximity"]["front"]
        self.left = self.robot.sensor["proximity"]["left"]
        self.right = self.robot.sensor["proximity"]["right"]


    # ============================================================
    # DEBUG DEL MAPA
    # ============================================================

    def imprimir_celdas(self):
        """
        Genera un JSON con todas las posiciones conocidas del mapa.

        Valores:
            0 -> obstáculo / pared
            1 -> posición visitada por el robot
            2 -> posición interior pendiente de explorar
        """

        datos = {
            "leyenda": {
                "0": "obstaculo",
                "1": "visitado",
                "2": "por_explorar"
            },
            "posiciones": []
        }

        for (x, y), valor in self.map.items():
            datos["posiciones"].append({
                "x": x,
                "y": y,
                "valor": valor
            })

        # Ordenamos para que el JSON sea más fácil de leer
        datos["posiciones"].sort(
            key=lambda p: (p["y"], p["x"])
        )

        with open("mapa.json", "w", encoding="utf-8") as archivo:
            json.dump(
                datos,
                archivo,
                indent=4,
                ensure_ascii=False
            )

        print("Mapa guardado en mapa.json")


