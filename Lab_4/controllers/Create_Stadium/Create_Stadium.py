from controller import Supervisor
import random

TIME_STEP = 32
BALL_RADIUS = 0.025
BALL_DENSITY = 1
AREA_SIZE = 2

supervisor = Supervisor()
root = supervisor.getRoot()
children_field = root.getField("children")

def create_ball(def_name, color, position):
    r, g, b = color
    x, y, z = position
    proto = f"""
    Solid {{
      translation {x:.3f} {y:.3f} {z:.3f}
      rotation 0 1 0 0
      children [
        DEF {def_name}_SHAPE Shape {{
          appearance Appearance {{
            material Material {{
              diffuseColor {r} {g} {b}
            }}
          }}
          geometry Sphere {{
            radius {BALL_RADIUS}
          }}
        }}
      ]
      boundingObject USE {def_name}_SHAPE
      physics Physics {{
        density {BALL_DENSITY}
      }}
    }}"""
    children_field.importMFNodeFromString(-1, proto)



def generate_positions(n, min_dist=0.07):
    positions = []
    tries = 0
    while len(positions) < n and tries < 2000:
        x = random.uniform(-AREA_SIZE + BALL_RADIUS, AREA_SIZE - BALL_RADIUS)
        y = random.uniform(AREA_SIZE*-1, AREA_SIZE - BALL_RADIUS)
        z = random.uniform(-AREA_SIZE + BALL_RADIUS, AREA_SIZE - BALL_RADIUS)
        pos = (x, y, z)
        if all(((px - x)**2 + (py - y)**2 + (pz - z)**2)**0.5 >= min_dist for px, py, pz in positions):
            positions.append(pos)
        tries += 1
    return positions

import math

elapsed = 0
while supervisor.step(TIME_STEP) != -1:
    elapsed += TIME_STEP / 1000.0
    if elapsed:
       # create_arena_outline()
        positions = generate_positions(20)
        for i in range(10):
            create_ball(f"BALL_BLACK_{i}", (1, 0, 0), positions[i])
            create_ball(f"BALL_WHITE_{i}", (1, 1, 1), positions[i + 10])
        break
