from solid import *
from solid.utils import *
from math import sin, cos, radians, degrees
from intakeManifold import extrude_intake_manifold

# TODO: set segments on all cylindric primitives for smooth models

# TODO: ensure the intakeRadius holes are consistent wrt wallWidth
# TODO: reducing r^2 - wallWidth^2  =? (l-wallwidth)*(w-wallWidth)
# (l-wallwidth)*(w-wallWidth) =  lw - lWall - wWall + wallWidth^2
#  =lw - wall(l+w) + wallWidth^2
# therefore: wallWidth of rectangle constrains linearly faster than wallWidth of radius
# Corr: reduce wallWidth of rectangle by wallWidth but add back wall(l+w)
# OR: consider wallWidth added to hole defined by intake

# TODO: verify wallWidth addition to solids works


# DESIGN GOAL:
# create an object that will possibly be used for:
# 1. design optimization for wetScrubber and/or vortex filterArray
# 2. grid array iteration (for instance: iterating for effective paricle diameter in
#  sequential-parallel array given a volume/manifold geometry)


def ConeFilter(
        intakeSlitLength,
        intakeSlitWidth,
        intakeSlitSize,
        intakeLeft,
        vortexSearcherDepth,
        collectorDepth,
        cylinderRadius,
        cylinderHeight,
        wallWidth
):
    # TODO: pir^2 must be greater than intake cross section where r is cylinderRadius.
    # need more safety constraints.
    '''
    Creates a Cone filter hull
    Parameters:
        intakeSlitLength: air intake nozzle height
        intakeSlitWidth: air intake nozzle width, should be minimized to particle/blob diameter
        intakeSlitSize: length of intake nozzle tube connecting and transforming the intake tube to a slit
        intakeLeft: if True, put intake on left side of filter, else place on right. 
        useful for symmetric arrays in rectangular manifolds.
        vortexSearcherDepth: sets depth of vortex (considered only past intakeSlitLength)
        collectorDepth: depth of collector cone after cylinder
        cylinderRadius: radius of cylinder that makes up the hull
        cylinderHeight: height of cylinder that makes up the hull
        wallWidth: width of wall for all parts
    '''
    ############# Build solids. #############
    intakeRadius = sqrt((intakeSlitLength*intakeSlitWidth+wallWidth)/pi)
    # build each part
    mainBodySolid = cylinder(r=(cylinderRadius + wallWidth),
                             h=(cylinderHeight+wallWidth))
    collectorConeSolid = cylinder(
        r1=(cylinderRadius + wallWidth), r2=(intakeRadius + wallWidth), h=collectorDepth)
    vortexTubeSolid = cylinder(
        r=(intakeRadius + wallWidth), h=(vortexSearcherDepth + intakeSlitLength + wallWidth), segments=100)
    # currently using a slit of unit width.
    # set width to expected particle/blob size
    # TODO:  move these into parameters for filter
    intakeSolid = extrude_intake_manifold(
        # TODO: should this be parameterized or just as much as possible? \
        # constants have no place in parametric models
        intake_resolution=100,
        exhaust_slit=intakeSlitLength + wallWidth,
        exhaust_width=intakeSlitWidth + wallWidth,
        exhaust_length=intakeSlitSize)  # TODO: consider extracting this

    ############# Open holes inside solids#############
    # becuase we parameterized by radius, wall width can be subtracted directly
    mainBody = mainBodySolid - \
        cylinder(r=cylinderRadius-wallWidth, h=cylinderHeight-wallWidth)
    collectorCone = collectorConeSolid - \
        hole()(cylinder(r1=cylinderRadius-wallWidth,
                        r2=intakeRadius-wallWidth, h=collectorDepth))
    vortexTube = vortexTubeSolid - \
        hole()(cylinder(r=(intakeRadius - wallWidth),
                        h=(vortexSearcherDepth + intakeSlitLength + wallWidth), segments=100))
    intake = intakeSolid - hole()(extrude_intake_manifold(
        intake_resolution=100,
        exhaust_slit=intakeSlitLength-wallWidth,
        exhaust_width=intakeSlitWidth-wallWidth,
        exhaust_length=intakeSlitSize))

    ############# Assemble filter: #############
    # mainBody
    if(intakeLeft is True):
        filter = mainBody + \
            rotate([180, 0, 0])(collectorCone) + \
            up(cylinderHeight - (vortexSearcherDepth + intakeSlitLength))(vortexTube) + \
            left(cylinderRadius - intakeSlitWidth)(up(cylinderHeight - intakeSlitLength/2 - wallWidth)
                                                   (rotate([90, 90, 0])(intake)))
    else:
        filter = mainBody + \
            rotate([180, 0, 0])(collectorCone) + \
            up(cylinderHeight - (vortexSearcherDepth + intakeSlitLength))(vortexTube) + \
            right(cylinderRadius - intakeSlitWidth)(up(cylinderHeight - intakeSlitLength/2 - wallWidth)
                                                    (rotate([90, 90, 0])(intake)))
    # TODO this is bad! need to parameterize width. only allowed because
    # currently only supporting slit with 1 width!
    # TODO: cylHeight-crossSection should be seekerOffset var

    return filter


############# Build Filter #############
solution = ConeFilter(
    intakeSlitLength=10, intakeSlitWidth=2, intakeSlitSize=20,
    intakeLeft=True, vortexSearcherDepth=5, collectorDepth=100,
    cylinderRadius=10, cylinderHeight=15, wallWidth=0.75)
# Optimization Considerations:
# minimize: cylinderHeight constrained by intakeSlitLength
# minimize: vortexSearcherDepth constrained by cylinderHeight (only increased based on low pressure aerodynamics)
# minimize: intakeSlitWidth: by particle diameter
# minimize: cylinderRadius for volume, otherwise this would be maximized (design constraint)
# minimize: wallWidth wrt pressure and material compressive strength
# maximize: intakeSlitLength
# maximize: collectorDepth ideally to infinity if pressure is infinity (perfect vacuum)
# TODO: consider a higher level, safer parameterization for these considerations:
#               remove: cylinderHeight, cylinderRadius by intakeArea, vortexSearcherDepth(after CFD analysis wrt pressure and intake model),
#                             |_need to be constrained by pressure models

# FUTURE DEVELOPMENT:
#   set minimum and maximum values for parameters
#  (so they dont create unsustainable models e.g.: intakeSlit>cylinderHeight)
#  then run optimization algorithm on it wrt CFD (consider FreeCAD)
#  also constrain the parameters to reduce testing (discretize and limit parameter space)

############# Writeout Filter #############
scad_render_to_file(
    solution,
    "cycloneFilter.scad",
    "PUT THE PATH TO YOUR OPENSCAD .EXE HERE",
)
