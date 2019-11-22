from solid import *
from solid.utils import *
from math import sin, cos, radians, degrees
# Encapsulated seperate from cyclone hull due to complexity and reuse of code for vacuumManifold

# TODO: put this back into camelCase because snakes are for dummies
# TODO: should constraint safety be implemented here? in extrude_intake_manifold?
# r = viewscad.Renderer(
#     openscad_exec="C:\Program Files\OpenSCAD\openscad.exe"
# )  # Point this to your openscad executable


def construct_polygon(sides, radius_major, radius_minor, offset):
    """
    Builds a planar 2D regular polygon with a given offset

    Parameters:
        polyPoints: number of points for the reglar polygon
        radius: the radius of a circle sweeping the polygon points
        offset: shift along z axis, useful for extrusions
    Returns:
        a list of Point3 entries defining the regular polygon
    """
    geometry = []
    for i in range(0, sides):
        angle = i * 360 / sides
        x = radius_major * sin(radians(angle))
        y = radius_minor * cos(radians(angle))

        geometry.append(Point3(x, y, offset))
    return geometry


def extrude_intake_manifold(intake_resolution, exhaust_slit, exhaust_width, exhaust_length):
    '''
    Extrudes an intake manifold for cyclone filtration.
    Parameters:
        intake_resolution: how many segments to be used for the final polygon (circle theroetically has infinite)
        exhaust_slit: length of slit that enters the vortex
        exhaust_width: width of slit that enters the vortex and also acts as cyclone guide (minimize)
        exhaust_length: entire length of intake manifold
    Returns:
        a Hull() consisting of the segments created with construct_polygon
    '''
    inlet_radius = sqrt(exhaust_slit*exhaust_width/pi)

    segmentedIntakeManifold = []  # return value
    # constants are okay here as this module is meant to be transformed from origin
    segmentedIntakeManifold.append(
        down(1)(scale([exhaust_slit, exhaust_width, 1])(cube(center=True)))
    )  # exhaust square (initial object)
    # raderator is the radius iterator for
    # rectangle to circle transform
    raderator_major = (exhaust_slit - inlet_radius)/intake_resolution
    raderator_minor = (exhaust_width - inlet_radius)/intake_resolution

    inlet_radius_major = exhaust_slit/2
    inlet_radius_minor = exhaust_width/2

    length_iterator = exhaust_length/intake_resolution

    for seg in range(1, intake_resolution):
        inlet_radius_major -= raderator_major/2
        inlet_radius_minor -= raderator_minor/2
        length_iterator += exhaust_length/intake_resolution
        segmentedIntakeManifold.append(
            extrude_along_path(
                construct_polygon(seg + 1,
                                  inlet_radius_major, inlet_radius_minor, length_iterator),
                [[0, 0, length_iterator], [0, 0, length_iterator + length_iterator]],
            )
        )
    return hull()(segmentedIntakeManifold)

############# Write to File (for logging) #############
# solution = extrude_intake_manifold()
# scad_render_to_file(
#     solution,
#     "intakeManifold.scad",
#     "PUT THE PATH TO YOUR OPENSCAD .EXE HERE",
# )
################################################
