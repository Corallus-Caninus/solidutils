# Cyclone Filtration with OpenSCAD Python library SolidPython
#### Welcome!

This package contains: 
- a parameterized cyclone filter.
- a parameterization of a sequential-parallel array of cyclone filters.

## Cyclone Filter
This module contains a cyclone filter, allowing for empirical/simulated optimization of geometries.
### Future Development
- threading along wall to act as vane and increase surface area.
- different methods for recycling dirty gas back into the cyclone using a slipstream recycler.
- different forced gas methods for enhancing vortex stability or particulate collision chance using one or more additional gas inlets. 


## Cyclone Array
This module parameterizes different ways of combining cyclone filters to create a filter solution that is clog resistant due to gradually decreasing the optimal particle size being filtered and efficient due to having equivalent cross-sectional area throughout the array, placing all energy loss and pressure differentials into filtering the particulate-gas stream.
### Future Development
- different methods for generating cyclone arrays, possibly using different infill geometries (cylinder, sphere, and general closed surfaces).
- a collector bin for dust/particulate collection mirroring the cycloneArray, parametrized in such a way that the user can define wet-scrubber or hydrocyclone rows which can have water proofed partitions of the collector bin for seperate dust-water sequences. Also post-hydrocyclone/wet-scrubber collector bin rows for drying gas before reaching the compressor, dripping back into previous row's reservoire but not being submerged themselves.
- recycling "dirty" but still pressurized gas from later sequences to initial sequences using secondary forced gas inlets, creating a sequence-sequence recycling slipstream loop.
