#!/usr/bin/env python
# coding: utf-8

# In[1]:


from solid import *
from solid.utils import *
from math import sin, cos, radians, degrees
# Encapsulated seperate from cyclone hull due to complexity and reuse of code for vacuumManifold

# TODO: should constraint safety be implemented here? in extrude_intake_manifold?
# r = viewscad.Renderer(
#     openscad_exec="C:\Program Files\OpenSCAD\openscad.exe"
# )  # Point this to your openscad executable


# In[2]:


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


# In[47]:


def extrude_intake_manifold(intake_resolution, exhaust_slit, exhaust_width, exhaust_length):
    '''
    Extrudes an intake manifold for cyclone filtration.
    Parameters:
        intake_resolution: how many segments to be used for the final polygon (circle theroetically has infinite)
        exhaust_slit: length of slit that enters the vortex
        exhaust_width: width of slit that enters the vortex and also acts as cyclone guide (minimize)
        exhaust_length: extruded distance of intake manifold
    Returns:
        a Hull() consisting of the segments created with construct_polygon
    '''
    inlet_radius = sqrt(exhaust_slit*exhaust_width/pi)
    intakeManifold = cube([exhaust_slit, exhaust_width, 1], center=True)
    #intakeManifold = down(1)(cube([exhaust_slit, exhaust_width, 1], center=True))

#     segmentedIntakeManifold = []  # return value
    # constants are okay here as this module is meant to be transformed from origin
#     segmentedIntakeManifold.append(
# #         down(1)(scale([exhaust_slit, exhaust_width, 1])(cube(center=True)))
#         down(1)(cube([exhaust_slit, exhaust_width, 1], center=True))
#     )  # exhaust square (initial object)
    # raderator is the radius iterator for
    # rectangle to circle transform
    
    #TODO: this is the problem
    raderator_major = (exhaust_slit/2 - inlet_radius)/intake_resolution
    raderator_minor = (exhaust_width/2 - inlet_radius)/intake_resolution

    inlet_radius_major = exhaust_slit/2
    inlet_radius_minor = exhaust_width/2

    length_iterator = exhaust_length/intake_resolution
    for seg in range(1, intake_resolution):
        inlet_radius_major -= raderator_major
        inlet_radius_minor -= raderator_minor
        
        #TODO: these should equate at all iterations-- iterating incorrectly. 
        #      shouldnt decrement so much figure out
        #      derivative of elipse to circle in terms of arc/curvature.
        #Model as 0, pi/2 pi and 3pi/4 and 2pi points of which 
        #intakeWidth and intakeHeight need to converge to
        #polygon also loses some precision to circle. 
        #should find nonlinear way to extrude
        
        
        length_iterator += exhaust_length/intake_resolution
        
        intakeManifold += up(length_iterator)(scale([inlet_radius_major, inlet_radius_minor])(cylinder(center=True)))
        #TODO: need to iterate to this, raderator is broken
#         segmentedIntakeManifold.append(
#             extrude_along_path(
# #                 construct_polygon(seg + 1,
#                 construct_polygon(intake_resolution,
#                                   inlet_radius_major, inlet_radius_minor, length_iterator),
#                 [[0, 0, length_iterator], [0, 0, length_iterator + length_iterator]],
#             )
#         )
    intakeManifold+=up(length_iterator+length_iterator)(cylinder(r=inlet_radius, center=True))
    return hull()(intakeManifold)
#     return hull()(segmentedIntakeManifold)


# In[48]:


############# Write to File (for logging) #############
intakeSlitHeight = 7
intakeSlitWidth = 2
wallWidth = 1
intakeSlitLength = 20
solution = extrude_intake_manifold(             intake_resolution=100,             exhaust_slit=intakeSlitHeight + wallWidth,             exhaust_width=intakeSlitWidth + wallWidth,            exhaust_length=intakeSlitLength)
scad_render_to_file(
    solution,
    "intakeManifold.scad"
)
################################################


# In[ ]:




