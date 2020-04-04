import math
from math import pi
from solid import *
from solid.utils import *
from cycloneArray import cycloneArray
# TODO: fix openscad-freecad plugin
#      create qt widget menu for entering [:3] params then drafting cube constraint [4:] params
#      make mod plugins for freecad, start with different fill primitives e.g.: cylinder, possible sphere and eventually any convex polygons in R3

# POSITIONAL LEGEND:
# initParams=[intakeSlitHeight=10, intakeSlitWidth=2,
#     intakeLeft=True, vortexSearcherDepth=5, collectorDepth=75,
#     cylinderRadius=10, cylinderHeight=15, wallWidth=0.05]

# reduction methods
def geometric_reduction_slit_height(x): return x/2  # remove floor bound
def geometric_reduction_slit_width(x): return x/2  # remove floor bound
def geometric_reduction_radius(x): return x/2  # remove floor bound
def geometric_increase_height(x): return x + x/4  # remove ceiling bound
# null is special case for no-op


def null(x): return x

init_params = [50000, 15000,
               True, 10000, 30000,
               40000, 100000, 1000]

delta_params = [geometric_reduction_slit_width, geometric_reduction_slit_height,
                null, null, null,
                geometric_reduction_radius, null, null]

# -1 is special case for unbounded
final_params = [500, 1000,
                -1, -1, -1,
                pi*1000, 20000, -1]

# cycloneArray = cycloneArray(init_params, delta_params, final_params, 500000, 500000, 500000, 20000)
builder = cycloneArray()
cycloneArray = builder.cycloneArray(init_params, delta_params, final_params, 450000, 450000, 300000, 20000)

print('done', cycloneArray)
scad_render_to_file(cycloneArray, 'cycloneArray.scad')