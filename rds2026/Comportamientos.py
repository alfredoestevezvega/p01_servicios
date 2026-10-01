import math


class FollowWall:
    def __init__(self, robot):
        self.robot = robot

        self.front = self.robot.sensor["proximity"]["front"]
        self.left = self.robot.sensor["proximity"]["left"]
        self.right = self.robot.sensor["proximity"]["right"]

        self.state = "search wall"

        self.visitadas = []

    def next_pose(self):
        orientacion = self.robot.sensor["orientation"]
        if orientacion == 0:
            pose = (self.robot.sensor["cells"][1])
            new_pose = (pose[0] + 1, pose[1])

        elif orientacion == 90:
            pose = (self.robot.sensor["cells"][0]) 
            new_pose = (pose[0], pose[1] - 1)

        elif orientacion == 180:
            pose = (self.robot.sensor["cells"][2]) 
            new_pose = (pose[0] - 1, pose[1])
        elif orientacion == 270:
            pose = (self.robot.sensor["cells"][3]) 
            new_pose = (pose[0], pose[1] + 1)
        return new_pose

    def _update(self):
        self.front = self.robot.sensor["proximity"]["front"]
        self.left = self.robot.sensor["proximity"]["left"]
        self.right = self.robot.sensor["proximity"]["right"]


    def get_action_(self):
        self._update()

        for cell in self.robot.sensor["cells"]:
            if cell not in self.visitadas:
                self.visitadas.append(cell)

        if self.front and self.right and self.left:
            return "rotate", 180
        
        elif self.state == "search wall":
            if self.front:
                self.state = "wall right"
                self.pto_inicio = self.robot.sensor["cells"][0]
                return "rotate", 90
            else:
                return "move", 0

        elif self.state == "wall right":
            
            new_pose = self.next_pose()
            if new_pose == self.pto_inicio:
                return "stop", 0
            elif not self.right:
                return "rotate", -90
            if self.front:
                return "rotate", 90

            else:

                return "move", 0
            


    def imprimir_celdas(self):
        celdas = self.visitadas

        if not celdas:
            print("No hay celdas visitadas")
            return

        robot_pos = self.robot.sensor["position"]
        orientacion = self.robot.sensor["orientation"]

        simbolos = {
            0: ">",      # Este
            90: "v",     # En tu mapa, norte se ve hacia abajo
            180: "<",    # Oeste
            270: "^"     # Sur se ve hacia arriba
        }

        xs = [x for x, y in celdas]
        ys = [y for x, y in celdas]

        xs.append(robot_pos[0])
        ys.append(robot_pos[1])

        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

        # Y invertida
        for y in range(min_y, max_y + 1):
            fila = ""

            for x in range(min_x, max_x + 1):

                if (x, y) == robot_pos:
                    fila += simbolos.get(orientacion, "R") + " "

                elif (x, y) in celdas:
                    fila += "# "

                else:
                    fila += "  "

            print(fila)













class DFT:
    def __init__(self, robot):
        self.robot = robot

        self.mapa = {
            robot.sensor["position"] : 0
        }

        self.orientations = {
            0 : "east",
            90 : "north",
            180 : "west",
            270 : "south"
        }

        self.pose_orientations_= {
            "north" : (0, -0.5),
            "south" : (0, 0.5),
            "west" : (-0.5, 0),
            "east" : (0.5, 0)
        }

        # Casillas ya visitadas
        self.visitados = [robot.sensor["position"]]
        self.camino = [robot.sensor["position"]]

        
    def sumar_tuplas_(self,tup1, tup2):
        return tuple((tup1[0] + tup2[0] , tup1[1] + tup2[1]))

    def get_vecinos_(self):
        pose = self.robot.sensor["position"]
        ori = self.robot.sensor["orientation"]
        vecinos = []

        for i in [0, 90, 270]:
            new_ori = (ori + i) % 360

            new_pose = self.sumar_tuplas_(
                pose, 
                self.pose_orientations_[self.orientations[new_ori]]    
            )

            if i == 0:
                vecinos.append([new_pose, "front"])
            elif i == 90:
                vecinos.append([new_pose, "left"])
            elif i == 270:
                vecinos.append([new_pose, "right"])
           
        return vecinos

    def update_map(self):
        self.mapa[self.robot.sensor["position"]] = 0
        vecinos = self.get_vecinos_()

        for vec in vecinos:
            if self.robot.sensor["proximity"][vec[1]]:
                self.mapa[vec[0]] = 1
            
            else:
                self.mapa[vec[0]] = 0

    def get_alpha_(self, new_pose):
        pose = self.robot.sensor["position"]
        orientation = self.robot.sensor["orientation"]

        dx = new_pose[0] - pose[0]
        dy = new_pose[1] - pose[1]

        objetivo = math.degrees(math.atan2(dy, dx))

        objetivo %= 360

        giro = (objetivo - orientation) % 360

        return giro

    def get_action(self):
        print(
            "POSE:", self.robot.sensor["position"],
            "ORI:", self.robot.sensor["orientation"],
            "CAMINO:", self.camino
        )
        self.update_map()
        pose = self.robot.sensor["position"]

        if pose not in self.visitados:
            self.visitados.append(pose)
            if self.camino[-1] != pose:

                self.camino.append(pose)

        vecinos = self.get_vecinos_()
        # Intentar avanzar a un vecino libre
        for vec in vecinos:
            posicion = vec[0]
            orientacion = vec[1]

            if self.mapa.get(posicion) == 0 and posicion not in self.visitados:
                if orientacion == "front":
                    return "move", 0
                
                elif orientacion == "left":
                    return "rotate", 90
                
                else:
                    return "rotate", -90

        if len(self.camino) > 1:
            if self.camino[-1] == pose:
                self.camino.pop()

            padre    = self.camino[-1]
            alpha = self.get_alpha_(padre)

            if alpha == 0:
                return "move", 0
            return "rotate", alpha 
        return "stop", 0            

                
    def imprimir_mapa(self):
        xs = sorted(set(x for x, y in self.mapa.keys()))
        ys = sorted(set(y for x, y in self.mapa.keys()), reverse=True)

        for y in ys:
            fila = ""

            for x in xs:
                valor = self.mapa.get((x, y), None)

                if valor == 0:
                    fila += ". "
                elif valor == 1:
                    fila += "# "
                else:
                    fila += "  "

            print(fila)