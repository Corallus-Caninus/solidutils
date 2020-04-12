from solid import *

from cycloneArray import cycloneArray

#TODO: make a unittest of 3 sets of params covering 
#      most important positions in parameter space

# TODO: fix openscad-freecad plugin
#      create qt widget menu for entering [:3] params then drafting cube constraint [4:] params
#      make mod plugins for freecad, start with different fill primitives e.g.: cylinder, possible sphere and eventually any convex polygons in R3


# reduction methods
def geometric_reduction_slit_height(x): return x - x/3  # remove floor bound
def geometric_reduction_slit_width(x): return x - x/3  # remove floor bound
def geometric_reduction_radius(x): return x - x/2.5  # remove floor bound
def geometric_increase_height(x): return x + x/4  # remove ceiling bound
# null is special case for no-op

def null(x): return x
#TODO: standardize upon a dimension e.g.: centimeters
#      this allows for easier scaling when transferring to 
#      manufacturing (3d print slicing)
#NOTE: params must be valid.
#      do not rely on assertions.
init_params = [300, 150,
               True, 100, 300,
               500, 1000, 25]

#delta_params = [geometric_reduction_slit_width, geometric_reduction_slit_height,
delta_params = [geometric_reduction_slit_height, geometric_reduction_slit_width,
                null, null, null,
                geometric_reduction_radius, null, null]

#NOTE: -1 is special case for unbounded
final_params = [5, 1,
                -1, -1, -1,
                100, 2000, -1]

# POSITIONAL LEGEND:
# initParams=[intakeSlitHeight=10, intakeSlitWidth=2,
#     intakeLeft=True, vortexSearcherDepth=5, collectorDepth=75
#     cylinderRadius=10, cylinderHeight=15, wallWidth=0.05]
builder = cycloneArray()
cycloneArray = builder.cycloneArray(init_params, delta_params, final_params, \
    8000, 10000, 4000, 300)

print('done', cycloneArray)
scad_render_to_file(cycloneArray, 'cycloneArray.scad')
