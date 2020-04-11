from solid import *
from solid.utils import *
from copy import deepcopy
from progress.bar import ShadyBar

from cycloneFilter import cycloneFilter


# TODO: build a grating on each intakeManifold that only allows particles smaller than intakeDiameter through
#       this is mostly important in the initial intake and allows for cleaning surfaces if model is detachable 
#       per sequence and fastened together. 

class cycloneArray:
    def __init__(self):
        return None

    def crossSectionalArea(self, *params):
        '''
        calculate the cross sectional area of the given cycloneFilter.

        Parameters:
            cycloneFilterParams: splat list of parameters that will define a cycloneFilter
        Returns:
            rectangular area of intakeWidth*intakeHeight
        '''
        return params[0] * params[1]

    def cycloneArray(self, init_params, delta_params, final_params, length, width, height, manifoldCeiling):
        '''
        build sequential/parallel array of cyclone filters in a cartesian grid pattern.
        starts by fitting width with init_params cycloneFilter objects, then alters geometry by delta_params 
        (currently can only functionally decrease parameters)and adds the equivalent airflow; 
        through each sequence of delta_params the total cross sectional area of filter intakes/outlets is kept constant. 
        (no bernoulli effect, all energy/pressure delta going to filtering friction/collision) 
        solution of the new cycloneFilter objects to the described rectangular prism.
        Stops when the rectangular prism is filled.

        Parameters:
                init_params: list of cycloneFilter positional parameters to start from
                delta_params: list of values that will change each respective positional init_params
                length: length (y+) of the rectangular prism fill constraint
                width: width (x+) of the rectangular prism fill constraint
                height: array height (z+) of the rectangular prism fill constraint
                manifoldCeiling: height (z++) of manifold above cyclone filters
        '''
        # TODO: integrate manifoldCeiling with height z axis constraints

        # TODO: add option or functional interface to take in vacuum_intake_manifold total cross sectional area and solve init_params intake_radius
        #       vacuum_intake_area = intake_radius*boxX_dimension/cylinder_radius #now solve for intake_radius..
        #                                                       ^ how many filters can fit in first row (reimman-sum sub-intervals of intake_radius)
        #
        #       intake_radius = vacuum_intake_area*cylinder_radius/boxX_dimension #solved for intake radius
        # eventually this need to be interface and constraints between radial compressor and vacuum inlet should be established

        # TODO: assert initial cylinder can fit in the first place (init cylinderRadius) and other bounding box min constraints
        # TODO: curtains cause low pressure regions due to square loss across large volume. this increases latent clog chance greatly
        #      compared to high pressure pipelines minimized for cross section of intake/outlet where constant high velocity fields
        #      keep particle friction dynamic. This is too complicated, would require kernel stride from one sequence to the next. but is a great feature for next iteration if worth it

        # TODO: set positional parameters as dictionary for keyword reference and unpack with ** operator
        #       double splat causes huge number of keyword arguments, consider passing in dictionary and dereferencing each entry here
        #       with locals().update(final_params) (NOTE: DO THIS IN FIRST FULLY FUNCTIONAL REFACTOR-- PRACTICE REGEX no rope) 
        #       can also use finals = SimpleNamespace(**final_params) then finals.cylinderRadius etc.

        # TODO: change sequential up right etc. commands to translate()() (again regex this post-aplha feature functionality

        # NOTE: in the future attempt inheritance and polymorphism when creating heirarchy of parametrized abstraction
        #  (e.g.: cyclone filter->cyclone array)

        # TODO: suspend in a box using infill between box and cycloneArray solution. should actually be easier here than in freecad.

        ### CYCLONE VARIABLES ###
        params = init_params
        # current position of cyclones being written
        curPosition = [0, 0]
        array = None
        # spacing distance between filters dynamically configured
        xDistance, yDistance = 0, 0
        print(params)

        # deiterate cross-sectional area in this loop.
        arrayBuffer = []  # holds a cell of filters
        areaSolution = 0  # static total sum of cross sectional area needed in parallel
        # for sequence pressure equivalence (not considering pressure loss in filter)
        areaBuffer = 0  # buffer for deiterating areaSolution per row in a sequence

        ########### INITIALIZE ###########
        # first filter sequence sets areaSolution (currently just one row is initial sequence of filters)
        # reset x axis with carriage return
        sequenceCounter = 0
        xDistance = 3*params[5] + 2*params[7]
        parallelFilters = width//xDistance

        areaSolution = parallelFilters*self.crossSectionalArea(*params)
        areaBuffer = areaSolution
        firstPass = True

        print('initialized {} parallelFilters with {} total cross section area'.format(parallelFilters, areaSolution))
        bufferBar = ShadyBar('creating a new sequence..', max=areaSolution)

        while True:  # cyclone builder loop
            # TODO: for loop this since inherently finite case, requires major logic refactoring
            if curPosition[1] > length:
                # we have reached the end of the rectangle
                print('\nEOL: Cyclone Array complete! throwing out {} items due to clipping'.format(len(arrayBuffer)))
                print('completed {} sequences with total cross sectional area per sequence of {}'.format(sequenceCounter - 1, areaSolution))
                #print('dumping {} buffer items from last sequence clipping\n'.format(
                #    len(arrayBuffer)))
                arrayBuffer.clear()

                scad_render_to_file(array, 'cycloneArray.scad')
                break

            ########### ITERATE TO NEXT SEQUENCE ###########
            intakeRadius = sqrt(params[0]*params[1]/pi)

            # iterate next row parameters.
            #TODO: triggering iteration on init_param row/sequence
            if not firstPass and areaBuffer <=0:
                # set x and y distance for new parameters
                yDistance = params[5]  # set spacing for next sequence

                params = [y(x) if y(x) > z or z == -1 else x for x,
                          y, z in zip(params, delta_params, final_params)]

                # set parameters based on given equation matrix, wallWidth is only float.
                paramsBuffer = [int(x) for x in params[:-1]]
                paramsBuffer.append(params[len(params)-1])
                params = paramsBuffer
                areaBuffer = areaSolution
                
                sequenceCounter += 1
                bufferBar = ShadyBar('creating new sequence: {}'.format(sequenceCounter), max=areaSolution)
                #print('\n\n new filter sequence: iterating filter parameters: {}\n'.format(params))

                # dump the buffer into the solution object
                for item in arrayBuffer:
                    if array is None:
                        array = item
                    else:
                        array += item

                arrayBuffer.clear()
                
                # new sequence spacing
                yDistance += 3*params[5] + 2*params[7]
                #build manifold for new sequence row
                sameSequence = False
            else:
                # same sequence spacing
                yDistance = 4*params[5] + 2*params[7]
                #build manifold for same sequence row
                sameSequence = True

            xDistance = 3*params[5] + 2*params[7]
            # reset x axis with carriage return
            curPosition[0] = 0  # carriage return x axis
           
            if not firstPass:
                curPosition[1] += yDistance
                intakeRadius = sqrt(params[1]*params[0])/pi
            firstPass = False
            
            parallelFilters = width//xDistance
            
            #calculate if nextRow will be the last in current sequence
            lastSequence = areaBuffer - parallelFilters*self.crossSectionalArea(*params) <= 0

            ###########BUILD MANIFOLD###########
            arrayBuffer.append(self.intakeManifold(params, intakeRadius, width, curPosition, yDistance, manifoldCeiling, \
                                sameSequence, lastSequence))
            arrayBuffer.append(self.exhaustManifold(params, intakeRadius, width, curPosition, yDistance, manifoldCeiling, \
                                sameSequence, lastSequence))

            # keep z dimension the same throughout array by mutating collector depth
            # TODO: this should be extracted and integrated into collectorDepth parameter
            # of cycloneFilter parameterization
            # TODO: this needs to be refactored somewhere outside main loop. consider cycloneRow
            #       since this is where cyclones are constructed and manipulated.
            # TODO: integrate this with manifold ceiling to reduce z-axis params
            if params[6] + params[4] < height:
                params[4] += height - (params[6] + params[4])
            elif params[6] + params[4] > height:
                params[4] -= (params[6] + params[4]) - height

            ###########ADD FILTERS TO CURRENT SEQUENCE###########

            # TODO: odd/even causes lots of sphagetti. make a function call across the conditions or refactor somehow
            if parallelFilters % 2 != 0:
                if areaBuffer <= 0:
                    # reached parallel cross-sectional area equivalence
                    break

                # start from origin then offset each filter origin+xDistance
                initialFilter = cycloneFilter(*params)

                initialFilter = left(curPosition[0])(initialFilter)
                initialFilter = forward(curPosition[1])(initialFilter)

                # have to tap manifold access lines, since hole() cant be used due to vortex searcher
                manifoldAccess = cylinder(
                    r=intakeRadius, h=2*params[7])
                manifoldAccess = left(curPosition[0])(manifoldAccess)
                manifoldAccess = forward(curPosition[1])(manifoldAccess)

                initialFilter = initialFilter - hole()(manifoldAccess)

                curPosition[0] += xDistance

                arrayBuffer.append(initialFilter)
                areaBuffer -= self.crossSectionalArea(*params)

                bufferBar.next(self.crossSectionalArea(*params))

                # TODO: check rectangular prism bounds, this will inevitably bug during cubic-suspension
                #       just wait for feature to expose

                for x in range(int((parallelFilters-1)//2)):
                    bufferBar.next(2*self.crossSectionalArea(*params))

                    if areaBuffer <= 0:
                        # reached parallel cross-section area equivalence
                        break

                    leftFilter, rightFilter = self.cycloneRow(
                        params, intakeRadius, curPosition)

                    # TODO: consider moving the below into cycloneRow since manifold mutates passed in buffer.
                    #      resolve this design divergence. try to refactor this using regex with logical expressions
                    curPosition[0] += xDistance

                    arrayBuffer.append(leftFilter)
                    areaBuffer -= self.crossSectionalArea(*params)
                    arrayBuffer.append(rightFilter)
                    areaBuffer -= self.crossSectionalArea(*params)

            else:
                # start from origin + xDistance/2
                curPosition[0] += xDistance/2
                for x in range(int(parallelFilters//2)):  # parallel loop
                    bufferBar.next(2*self.crossSectionalArea(*params))
                    if areaBuffer <= 0:
                        # reached parallel cross-sectional area equivalence
                        break

                    leftFilter, rightFilter = self.cycloneRow(
                        params, intakeRadius, curPosition)

                    curPosition[0] += xDistance

                    arrayBuffer.append(leftFilter)
                    areaBuffer -= self.crossSectionalArea(*params)
                    arrayBuffer.append(rightFilter)
                    areaBuffer -= self.crossSectionalArea(*params)
        return array

    def cycloneRow(self, params, intakeRadius, curPosition):
        '''
        returns a symmetric pair of cyclone filters used in creating rows within a sequence
        '''
        #LEFT Cyclone
        leftParams = params
        leftParams[2] = True

        # tap manifold from cylinder
        leftManifoldAccess = cylinder(
            r=intakeRadius, h=2*params[7])
        leftManifoldAccess = left(curPosition[0])()(leftManifoldAccess)
        leftManifoldAccess = forward(curPosition[1])()(leftManifoldAccess)

        leftFilter = cycloneFilter(*leftParams)
        leftFilter = left(curPosition[0])(leftFilter)
        leftFilter = forward(curPosition[1])(leftFilter)

        leftFilter = leftFilter - hole()(leftManifoldAccess)

        #RIGHT Cyclone
        rightParams = params
        rightParams[2] = False

        # tap manifold from cylinder
        rightManifoldAccess = cylinder(
            r=intakeRadius, h=2*params[7])
        rightManifoldAccess = right(curPosition[0])()(rightManifoldAccess)
        rightManifoldAccess = forward(curPosition[1])()(rightManifoldAccess)

        rightFilter = cycloneFilter(*rightParams)
        rightFilter = right(curPosition[0])(rightFilter)
        rightFilter = forward(curPosition[1])(rightFilter)

        rightFilter = rightFilter - hole()(rightManifoldAccess)
        
        return leftFilter, rightFilter

    def exhaustManifold(self, params, intakeRadius, width, curPosition, yDistance, manifoldCeiling, sameSequence, lastSequence):
        '''
        Build an exhaust spanning all cyclone filters in a sequence.
        Also connects exhaust of this sequence to intake of the next.

        PARAMETERS:
            params: params defining cycloneFilters in this row
            intakeRadius: cross sectional area of a filter
            width: width of cycloneArray's constraining rectangular prism
            curPosition: tuple containing current (x,y) position of the cursor writing filters
            yDistance: current spacing between filter rows
            manifoldCeiling: height of manifold pipelining
            sameSequence: a boolean for if this row is the first in the sequence
            lastSequence: a boolean for if this row is the last in the sequence
        RETURNS:
            a fully constructed OpenSCADObject exhaust manifold
        '''
        #Configuration
        exhaustManifoldWidth = width + params[7]
        exhaustManifoldLength = 3*intakeRadius + params[7]
        # TODO: verify equivalence with intakeButt 
        exhaustManifoldHeight = manifoldCeiling

        exhaustCrossBarWidth = width/3
        exhaustCrossBarReach = exhaustManifoldLength + yDistance

        exhaustManifoldSolid = cube([\
                exhaustManifoldWidth, \
                exhaustManifoldLength, \
                exhaustManifoldHeight], center = True)
        exhaustManifold = exhaustManifoldSolid - hole()(cube([ \
                exhaustManifoldWidth - params[7], \
                exhaustManifoldLength - params[7], \
                exhaustManifoldHeight - params[7]], center=True))

        if lastSequence:
            #build exhaustButt
            #TODO: exhaustButt sticks out
            exhaustButtSolid = cube([\
                    exhaustManifoldWidth, \
                    exhaustManifoldLength, \
                    exhaustManifoldHeight], center=True)
            exhaustButt = exhaustButtSolid - hole()(\
                    cube([\
                    exhaustManifoldWidth - params[7], \
                    exhaustManifoldLength - params[7], \
                    exhaustManifoldHeight - params[7]], center=True))

            #position atop the exhaustManifold
            exhaustButt = up(exhaustManifoldHeight)(exhaustButt)

            exhaustManifold += exhaustButt

        if sameSequence:
            #build exhaust crossBar
            exhaustCrossBarSolid = cube([\
                    exhaustCrossBarWidth, \
                    exhaustCrossBarReach, \
                    exhaustManifoldHeight], center=True)
            exhaustCrossBar = exhaustCrossBarSolid - hole()(cube([\
                   exhaustCrossBarWidth - params[7], \
                   exhaustCrossBarReach - params[7], \
                   exhaustManifoldHeight - params[7]], center=True))
            
            exhaustCrossBar = up(exhaustManifoldHeight/2)(exhaustCrossBar)
            exhaustCrossBar = forward(exhaustCrossBarReach/2 - exhaustManifoldLength/2)(exhaustCrossBar)
            #TODO: move back a bit

            exhaustManifold += exhaustCrossBar

        #move exhaustManifold solution into position
        exhaustManifold = forward(curPosition[1])(exhaustManifold)
        #TODO: move into position due to centering
        exhaustManifold = up(exhaustManifoldHeight/2 + params[7])(exhaustManifold)
        
        return exhaustManifold

    def intakeManifold(self, params, intakeRadius, width, curPosition, yDistance, manifoldCeiling, sameSequence, lastSequence):
        '''
        Build an intake spanning all cyclone filters in a sequence.

        PARAMETERS:
            params: params defining cycloneFilters in this row
            intakeRadius: cross sectional area of a filter
            width: width of cycloneArray's constraining rectangular prism
            curPosition: tuple containing current (x,y) position of the cursor writing filters
            yDistance: current spacing between filter rows
            manifoldCeiling: height of manifold pipelining
            sameSequence: a boolean for if this row is the first in the sequence
            lastSequence: a boolean for if this row is the last in the sequence
        RETURNS:
            a fully constructed OpenSCADObject intake manifold
        '''
        # TODO: remove sameSequence once fully featured
        #Configuration
        intakeManifoldWidth = width/3
        intakeManifoldLength = params[5] + params[7]

        UPipeVHeight = manifoldCeiling + intakeManifoldLength + params[7]
        
        #intakeCrossBarHeight = intakeManifoldLength #+ params[0/2]
        #if sameSequence:
        #    intakeCrossBarReach = yDistance 
        #else:
        #yDistance without considering previous/next sequence
        intakeCrossBarReach = 4*params[5] + 2*params[7] 

        intakeCrossBarReach += intakeManifoldLength

        buttSpan =  yDistance + params[7]  - 1.5*params[5] - 1.5*params[7]#NOTE: this is correct spacing

        #Build intakeManifold that connects cyclone inlets in parallel
        #TODO: fix manifold wall spacing. This is unclean code and breaks design regularity 
        intakeManifoldSolid = cube([ \
            width + params[7], \
            intakeManifoldLength, \
            3*intakeRadius + params[0] + params[7]], center=True)

        intakeManifold = intakeManifoldSolid - hole()(cube([ \
            width, \
            params[5], \
            3*intakeRadius + params[0]], center = True))
       
        if not sameSequence:
            #Build intakeButt that connects previous sequence in series

            #essentially, this is a third U-Pipe vertical segment centered about y-origin
            #extrude here to previous exhaustButt due to yDistance containing previous row's radius
            #NOTE: subtracting 3 wallWidth here to make up for U-Pipe's not having wallWidth 3
            intakeButtSolid = cube([\
                    intakeManifoldWidth + 2*params[7], \
                    intakeManifoldLength, \
                    UPipeVHeight], center=True)
            intakeButt = intakeButtSolid - hole()(cube([ \
                intakeManifoldWidth - 3*params[7], \
                intakeManifoldLength - params[7], \
                UPipeVHeight - params[7]], center=True))

            intakeButt = up(UPipeVHeight/2)(intakeButt)
            #intakeButt = back(intakeManifoldLength/2)(intakeButt)

            #Build other half of the butt
            intakeButtSpanSolid = cube([\
                    intakeManifoldWidth, \
                    buttSpan, \
                    intakeManifoldLength], center=True)
            intakeButtSpan = intakeButtSpanSolid - hole()(cube([\
                    intakeManifoldWidth - params[7], \
                    buttSpan - params[7],\
                    intakeManifoldLength - params[7]], center=True))
            intakeButtSpan = intakeButtSpanSolid

            #intakeButtSpan = up(intakeManifoldLength + manifoldCeiling + params[0]/2 + params[7])(intakeButtSpan)
            intakeButtSpan = up(intakeManifoldLength + params[0]/2 + params[7])(intakeButtSpan)
            intakeButtSpan = back(buttSpan/2 - intakeManifoldLength/2)(intakeButtSpan)

            #Butt assembly
            intakeManifold += intakeButt + intakeButtSpan

        #build U-Pipe that connects the intakes of current 
        #row in a sequence in parallel 
        UPipeSolid = cube([\
            intakeManifoldWidth, \
            intakeManifoldLength, \
            UPipeVHeight], center=True)
        UPipe = UPipeSolid - hole()(cube([ \
            intakeManifoldWidth - params[7],\
            intakeManifoldLength - params[7], \
            UPipeVHeight - params[7]], center=True)) #VHeight is open on either end

        UPipe = up(UPipeVHeight/2)(UPipe)# set atop the intakeManifold (xy-plane)

        if not lastSequence:
            #daisy chain U-Pipes in each row 
            #to connect sequence's intakes in parallel
            intakeCrossBarSolid = cube([\
                intakeManifoldWidth, \
                intakeCrossBarReach, \
                intakeManifoldLength], center=True)
            intakeCrossBar = intakeCrossBarSolid - hole()(cube([\
                intakeManifoldWidth - params[7], \
                intakeCrossBarReach - params[7], \
                intakeManifoldLength - params[7]], center=True))

            #TODO: minimize crossBarHeight
            #TODO: refactor this with intakeButtSpan
            intakeCrossBar = up(intakeManifoldLength + manifoldCeiling + params[0]/2 + params[7])(intakeCrossBar) 
            intakeCrossBar = forward(intakeCrossBarReach/2 - intakeManifoldLength/2)(intakeCrossBar)

            # connect intakeCrossBar
            UPipe += intakeCrossBar  

        # mirror along y-axis 
        leftUPipe = left(width/3 + params[7]/2)(deepcopy(UPipe))
        rightUPipe = right(width/3 + params[7]/2)(deepcopy(UPipe))

        UPipe = leftUPipe + rightUPipe
        intakeManifold += UPipe 
        
        # move intake manifold into position
        intakeManifold = back(1.5*params[5] + 1.5*params[7])(intakeManifold)
        intakeManifold = forward(
            curPosition[1])(intakeManifold)

        intakeManifold = down(params[0]/2 + params[7])(intakeManifold)

        return intakeManifold
