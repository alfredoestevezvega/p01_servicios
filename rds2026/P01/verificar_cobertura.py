import os, sys, json
os.environ['SDL_VIDEODRIVER']='dummy'
os.environ['SDL_AUDIODRIVER']='dummy'
sys.path.insert(0,str(__import__('pathlib').Path(__file__).resolve().parents[1]))
os.chdir(str(__import__('pathlib').Path(__file__).resolve().parents[1]))
import pygame
from P01.part1 import cells
import rds2026environment as e
import rds2026machines as m
import rds2026simulation as s
r=m.vacuum_zero((21,21),0)
sim=s.simulation((700,700),0,e.floorplan('cfg_0.py'),r)
d=sim.screen['window']['density']
valid=set()
for x in range(1,56):
 for y in range(1,56):
  rect=pygame.Rect(int((x/2-1)*d),int((y/2-1)*d),2*d,2*d)
  if not any(rect.colliderect(o['bbox']) for o in sim.environment.objects): valid.add((x,y))
seen={(42,42)}; todo=list(seen)
while todo:
 x,y=todo.pop()
 for p in ((x+1,y),(x-1,y),(x,y+1),(x,y-1)):
  if p in valid and p not in seen: seen.add(p);todo.append(p)
expected=set().union(*(cells(p) for p in seen))
for name in ('resultado',):
 data=json.load(open(str(__import__('pathlib').Path(__file__).resolve().parent / (name+'.json'))))
 actual=set(map(tuple,data['free_positions']))
 covered=set(map(tuple,data['covered_tiles']))
 print(name, 'reachable',len(seen),'expected_tiles',len(expected),'missing_positions',len(seen-actual),'extra_positions',len(actual-seen),'missing_tiles',len(expected-covered),'extra_tiles',len(covered-expected))

 assert expected == covered, 'Cobertura incompleta'
 assert data['complete'] and data['collisions'] == 0
