from solid import *
from solid.utils import *
from math import sin, cos, radians, degrees
from . import extrude_intake_manifold

#############Static Constants#############
CircleResolution = 100

# TODO: verify wallWidth addition to solids works
#               (area is no less than intakeRadius throughout the model)
#               only pressure lose should be from friction and filtering no venturi effect


# DESIGN GOAL:
# create an object that will possibly be used for:
# 1. design optimization for wetScrubber and/or vortex filterArray
# 2. grid array iteration (for instance: iterating for effective paricle diameter in
#  sequential-parallel array given a volume/manifold geometry)


def cycloneFilter(
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
    ############# Build Solids: #############
    # WAS: +wallWidth
    intakeRadius = sqrt((intakeSlitLength*intakeSlitWidth)/pi)
    # build each part
    mainBodySolid = cylinder(r=(cylinderRadius + wallWidth),
                             h=(cylinderHeight+wallWidth), segments=CircleResolution)
    collectorConeSolid = cylinder(
        r1=(cylinderRadius + wallWidth), r2=(intakeRadius + wallWidth), h=collectorDepth, segments=CircleResolution)
    vortexTubeSolid = cylinder(
        r=(intakeRadius + wallWidth), h=(vortexSearcherDepth + intakeSlitLength), segments=CircleResolution)
    intakeSolid = extrude_intake_manifold(
        # TODO: constants have no place in parametric models
        intake_resolution=100,
        exhaust_slit=intakeSlitLength + wallWidth,
        exhaust_width=intakeSlitWidth + wallWidth,
        exhaust_length=intakeSlitSize)

    ############# Open holes inside solids: #############
    # becuase we parameterized by radius, wall width can be subtracted directly
    mainBody = mainBodySolid - \
        cylinder(r=cylinderRadius, h=cylinderHeight, segments=CircleResolution)
    collectorCone = collectorConeSolid - \
        hole()(cylinder(r1=cylinderRadius,
                        r2=intakeRadius, h=collectorDepth, segments=CircleResolution))
    vortexTube = vortexTubeSolid - \
        hole()(cylinder(r=(intakeRadius),
                        h=(vortexSearcherDepth + intakeSlitLength), segments=CircleResolution))
    intake = intakeSolid - hole()(extrude_intake_manifold(
        intake_resolution=100,
        exhaust_slit=intakeSlitLength,
        exhaust_width=intakeSlitWidth,
        exhaust_length=intakeSlitSize))

    ############# Assemble filter: #############
    # mainBody
    # TODO: remove rotates where appropriate
    if(intakeLeft is True):
        filter = mainBody + \
            rotate([180, 0, 0])(collectorCone) + \
            up(cylinderHeight - (vortexSearcherDepth + intakeSlitLength-wallWidth))(vortexTube) + \
            left(cylinderRadius - intakeSlitWidth)(up(cylinderHeight - intakeSlitLength/2 - wallWidth)
                                                   (rotate([90, 90, 0])(intake)))
    else:
        filter = mainBody + \
            rotate([180, 0, 0])(collectorCone) + \
            up(cylinderHeight - (vortexSearcherDepth + intakeSlitLength-wallWidth))(vortexTube) + \
            right(cylinderRadius - intakeSlitWidth)(up(cylinderHeight - intakeSlitLength/2 - wallWidth)
                                                    (rotate([90, 90, 0])(intake)))

    return filter


############# Build Filter: #############
solution = cycloneFilter(
    intakeSlitLength=10, intakeSlitWidth=2, intakeSlitSize=20,
    intakeLeft=True, vortexSearcherDepth=5, collectorDepth=75,
    cylinderRadius=10, cylinderHeight=15, wallWidth=0.05)


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

############# Writeout Filter Model #############
scad_render_to_file(
    solution,
    "cycloneFilter.scad",
    # "PUT THE PATH TO YOUR OPENSCAD .EXE HERE",

    # REMOVE THIS WHEN COMMITING
    #    "C:/Users/jw.local/AppData/Roaming/Microsoft/Windows/Start Menu/Programs/OpenSCAD.exe",
)
