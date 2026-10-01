#!/usr/bin/env python3
"""Exploración con sensores; ejecutar: python3 -m P01.part1.

El mapa representa posiciones transitables del centro del robot, no muros
puntuales: una detección significa que su cuerpo de 2x2 no cabe allí.
"""
import argparse
from collections import deque
from copy import deepcopy
import json
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
os.environ.setdefault('PYGAME_HIDE_SUPPORT_PROMPT', '1')
import rds2026machines as machines
import rds2026environment as environments
import rds2026simulation as simulations

DIRECTIONS = {0: (1, 0), 90: (0, -1), 180: (-1, 0), 270: (0, 1)}


def key(position):
    return tuple(round(v * 2) for v in position)


def cells(point):
    # Equivalente a check_cells(), en coordenadas enteras de medio metro.
    x, y = point
    return {(a // 2, b // 2) for a in (x-1, x+1)
            for b in (y-1, y+1)}


class Explorer:
    def __init__(self, robot, strategy='nearest'):
        self.robot = robot
        self.strategy = strategy
        self.free = set()
        self.blocked = set()
        self.visited = set()
        self.covered = set()
        self.graph = {}
        self.turns = 0

    def observe(self):
        r = self.robot
        p = key(r.sensor['position'])
        self.visited.add(p)
        self.free.add(p)
        self.covered.update(cells(p))
        self.graph.setdefault(p, set())
        for name, offset in (('front', 0), ('left', 90), ('right', -90)):
            angle = (round(r.sensor['orientation']) + offset) % 360
            dx, dy = DIRECTIONS[angle]
            q = (p[0]+dx, p[1]+dy)
            if r.sensor['proximity'][name]:
                self.blocked.add(q)
            else:
                self.free.add(q)
                self.graph[p].add(q)
                self.graph.setdefault(q, set()).add(p)
        return p

    def action(self):
        r = self.robot
        r.check_proximity()
        p = self.observe()
        # Una lectura tras media vuelta descubre también el sector trasero.
        # Girar es una acción instantánea en este simulador, sin traslación.
        neighbors = {(p[0]+dx, p[1]+dy) for dx, dy in DIRECTIONS.values()}
        if any(q not in self.free and q not in self.blocked for q in neighbors):
            r.rotate(180)
            self.turns += 1
            r.check_proximity()
            self.observe()
        pending = self.free - self.visited
        if not pending:
            return False
        orientation = round(r.sensor['orientation']) % 360
        dx, dy = DIRECTIONS[orientation]
        forward = (p[0]+dx, p[1]+dy)
        # Búsqueda por niveles: mínimo trayecto por el mapa ya observado.
        queue = deque([p])
        parent = {p: None}
        targets = []
        while queue and not targets:
            for _ in range(len(queue)):
                node = queue.popleft()
                if node in pending:
                    targets.append(node)
                    continue
                for q in sorted(self.graph.get(node, ())):
                    if q not in parent:
                        parent[q] = node
                        queue.append(q)
        if not targets:
            raise RuntimeError('Frontera sin camino conocido')
        def first_step(q):
            while parent[q] != p:
                q = parent[q]
            return q
        def score(q):
            return (first_step(q) == forward, -q[1], -q[0])
        target = first_step(max(targets, key=score))
        delta = (target[0]-p[0], target[1]-p[1])
        angle = next(a for a, d in DIRECTIONS.items() if d == delta)
        turn = (angle - round(r.sensor['orientation']) + 180) % 360 - 180
        if turn:
            r.rotate(turn)
            self.turns += 1
        r.check_proximity()
        if r.sensor['proximity']['front']:
            raise RuntimeError('El camino observado dejó de ser transitable')
        return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--fps', type=int, default=32)
    parser.add_argument('--max-steps', type=int, default=30000)
    parser.add_argument('--strategy', choices=['nearest', 'original'], default='nearest')
    parser.add_argument('--output', type=Path, default=ROOT/'P01'/'resultado.json')
    args = parser.parse_args()
    args.output = args.output.resolve()
    if args.headless:
        os.environ['SDL_VIDEODRIVER'] = 'dummy'
        os.environ['SDL_AUDIODRIVER'] = 'dummy'
    os.chdir(ROOT)  # Los recursos del simulador usan rutas relativas.
    robot = machines.vacuum_zero(position=(21, 21), orientation=0)
    # El simulador define sensores mutables de clase y no inicializa odometría.
    robot.sensor = deepcopy(robot.sensor)
    robot.sensor['position'] = robot.position
    robot.sensor['orientation'] = robot.orientation
    sim = simulations.simulation(size=(700, 700), fps=0 if args.headless else args.fps,
                                 environment=environments.floorplan('cfg_0.py'), machine=robot)
    sim.start()
    explorer = Explorer(robot, args.strategy)
    steps = 0
    milestones = {}
    complete = False
    start = time.perf_counter()
    if args.strategy == 'original':
        from Comportamientos import FollowWall
        original = FollowWall(robot)
    try:
        while sim.is_running and steps < args.max_steps:
            if robot.is_running or args.strategy == 'original':
                if args.strategy == 'original':
                    import contextlib
                    with open(os.devnull, 'w') as sink, contextlib.redirect_stdout(sink):
                        action, turn = original.get_action()
                    if action == 'move':
                        robot.start()
                    elif action == 'rotate':
                        robot.stop()
                        robot.rotate(turn)
                    else:
                        break
                elif not explorer.action():
                    complete = True
                    break
                steps += 1
            sim.update()
            explorer.covered.update(cells(key(robot.sensor['position'])))
            for threshold in (100, 200, 300, 400, 416, 450, 500):
                if len(explorer.covered) >= threshold:
                    milestones.setdefault(str(threshold), steps)
            if not args.headless:
                import pygame
                pygame.display.set_caption(f'Exploración: {len(explorer.covered)} celdas | {steps} pasos | {robot.stats_collisions} colisiones')
    finally:
        elapsed = time.perf_counter() - start
        result = dict(strategy=args.strategy, complete=complete, steps=steps,
                      seconds_at_32fps=steps/32, elapsed_seconds=elapsed,
                      collisions=robot.stats_collisions, turns=explorer.turns,
                      covered_cells=len(explorer.covered), visited_positions=len(explorer.visited),
                      milestones=milestones,
                      free_positions=sorted(explorer.free), blocked_positions=sorted(explorer.blocked),
                      covered_tiles=sorted(explorer.covered))
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2)+'\n')
        # Imagen de las celdas recorridas; el mapa JSON conserva la resolución fina.
        import pygame
        sim.environment.update()
        overlay = pygame.Surface((700, 700), pygame.SRCALPHA)
        density = sim.screen['window']['density']
        for x, y in explorer.covered:
            pygame.draw.rect(overlay, (40, 190, 90, 95),
                             (x*density, y*density, density, density))
        sim.screen['display'].blit(overlay, (0, 0))
        sim.environment.update_extra()
        robot.stop()
        robot.update()
        pygame.image.save(sim.screen['display'], str(args.output.with_suffix('.png')))
        print(json.dumps({k: v for k, v in result.items() if not isinstance(v, list)}, indent=2))
        sim.stop()


if __name__ == '__main__':
    main()
