# mapeado.py

Desconocido = 0
Libre = 1
Ocupado = 2


class MapaOcupacion:

    def __init__(self):
        self.grid = {}

    def Marca_libre(self, celda):
        # Las casillas donde está el robot son libres obligatoriamente
        self.grid[celda] = Libre

    def Marca_ocupado(self, celda):
        # Si una casilla ya se comprobó como libre porque el robot pasó
        # por ella, no la sobrescribimos como ocupada
        if self.grid.get(celda, Desconocido) != Libre:
            self.grid[celda] = Ocupado

    def Direccion(self, orientacion):
        direcciones = {
            0:   (1, 0),    # Este
            90:  (0, -1),   # Norte
            180: (-1, 0),   # Oeste
            270: (0, 1)     # Sur
        }

        return direcciones[orientacion]

    def Girar_izquierda(self, direccion):
        dx, dy = direccion
        return (dy, -dx)

    def Girar_derecha(self, direccion):
        dx, dy = direccion
        return (-dy, dx)

    def Celdas_adyacentes(self, celdas_robot, direccion):
        celdas_robot = set(celdas_robot)

        dx, dy = direccion

        final = set()

        for x, y in celdas_robot:
            vecino = (x + dx, y + dy)

            if vecino not in celdas_robot:
                final.add(vecino)

        return final

    def Actualizar(self, robot):
        celdas_robot = set(robot.sensor["cells"])

        # Por donde haya pasado el robot es una celda libre
        for celda in celdas_robot:
            self.Marca_libre(celda)

        frente = self.Direccion(robot.sensor["orientation"])
        izquierda = self.Girar_izquierda(frente)
        derecha = self.Girar_derecha(frente)

        sensores = {
            "front": frente,
            "left": izquierda,
            "right": derecha
        }

        for nombre_sensor, direccion in sensores.items():

            celdas = self.Celdas_adyacentes(
                celdas_robot,
                direccion
            )

            if robot.sensor["proximity"][nombre_sensor]:
                for celda in celdas:
                    self.Marca_ocupado(celda)

            else:
                for celda in celdas:
                    self.Marca_libre(celda)

    def Mostrar_mapa(self, robot=None):

        if not self.grid:
            print("Mapa vacío")
            return

        # Celdas ocupadas actualmente por el robot
        celdas_robot = set()

        if robot is not None:
            celdas_robot = set(robot.sensor["cells"])

        # Límites del mapa conocido
        xs = [x for x, y in self.grid.keys()]
        ys = [y for x, y in self.grid.keys()]

        min_x = min(xs)
        max_x = max(xs)

        min_y = min(ys)
        max_y = max(ys)

        print()

        for y in range(min_y, max_y + 1):

            fila = ""

            for x in range(min_x, max_x + 1):

                celda = (x, y)

                if celda in celdas_robot:
                    fila += "R"

                else:
                    estado = self.grid.get(celda, Desconocido)

                    if estado == Libre:
                        fila += "."

                    elif estado == Ocupado:
                        fila += "#"

                    else:
                        fila += "?"

            print(fila)

        print()

    def Estadisticas(self):

        libres = 0
        ocupadas = 0

        for estado in self.grid.values():

            if estado == Libre:
                libres += 1

            elif estado == Ocupado:
                ocupadas += 1

        print("Celdas libres:", libres)
        print("Celdas ocupadas:", ocupadas)
        print("Total conocidas:", libres + ocupadas)
