from cycloneFilter import cycloneFilter
from solid import *
from solid.utils import *
import viewscad
import math
from copy import deepcopy

#TODO: use distribute_in_grid method from openscad utils, some introspection is needed for parametric manifold fitting
#      so this may be the best implementation.
#TODO: write shell method that implementes hole() or subtractive differencing given wallWidth and hole boolean
#      e.g.: shell(wall=10, hole=True)(cube([100,100,100]))

class cycloneArray:
    # TODO: prototype transition to object oriented
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

    # TODO: consider inheriting openscadObject class as in primitives
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
                length: array size containing sequential filters
                width: array size containing parallel filters 
                height: array height for filters
                manifoldCeiling: height of manifold above cyclone filters
        '''
        # TODO: constrain manifoldCeiling to prevent pressure differential/bernoulli effect and integrate with height z axis constraints

        # TODO: add option or functional interface to take in vacuum_intake_manifold total cross sectional area and solve init_params intake_radius
        #       vacuum_intake_area = intake_radius*boxX_dimension/cylinder_radius #now solve for intake_radius..
        #                                                       ^ how many filters can fit in first row (reimman-sum sub-intervals of intake_radius)
        #
        #       intake_radius = vacuum_intake_area*cylinder_radius/boxX_dimension #solved for intake radius
        # eventually this need to be interface and constraints between radial compressor and vacuum inlet should be established

        # TODO: assert each cyclone doesnt cause pressure differentials due to cylinder_radius
        #      being smaller than intake/outlet_radius (already preliminary cheked just need thorough code trace)
        # TODO: assert initial cylinder can fit in the first place (init cylinderRadius)
        #
        # TODO: curtains cause low pressure regions due to square loss across large volume. this increases clog chance greatly
        #      compared to high pressure pipelines minimized for cross section of intake/outlet where constant high velocity fields
        #      keep particle friction dynamic. This is too complicated, would require kernel stride from one sequence to the next.
        #      This would also require coarse grid such as in the initial sequence's intake since theres a venturi differential.
        #

        # TODO: set positional parameters as dictionary for keyword reference and unpack with ** operator
        # TODO: in the future attempt inheritance and polymorphism when creating heirarchy of parametrized abstraction
        #  (e.g.: cyclone filter->cyclone array)

        # TODO: refactor/reduce all of this conditions are spread out too much
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

        while True:  # cyclone builder loop
            print('in the loop')

            ########### ITERATE TO NEXT SEQUENCE ###########
            if areaBuffer <= 0 and areaSolution != 0:
                print('\n iterating filter parameters-- new sequence.. \n')
                params = [y(x) if y(x) > z or z == -1 else x for x,
                          y, z in zip(params, delta_params, final_params)]

                # set parameters based on given equation matrix, wallWidth is only float.
                paramsBuffer = [int(x) for x in params[:-1]]
                paramsBuffer.append(params[len(params)-1])
                params = paramsBuffer
                print(params)
                areaBuffer = areaSolution

                print('appending filters')
                # dump the buffer into the solution object
                for cyclone in arrayBuffer:
                    if array is None:
                        array = cyclone
                    else:
                        array += cyclone

                #TODO: connect previous and current sequence manifolds here. need if else condition
                #      for whether manifold is parallel or sequential.

                arrayBuffer.clear()

            # keep z dimension the same throughout array by mutating collector depth
            # TODO: this should be extracted and integrated into collectorDepth parameter
            # of cycloneFilter parameterization
            # TODO: integrate this with manifold ceiling to reduce z-axis params
            if params[6] + params[4] < height:
                params[4] += height - (params[6] + params[4])
            elif params[6] + params[4] > height:
                params[4] -= (params[6] + params[4]) - height

            curPosition[1] += yDistance  # y position
            curPosition[0] = 0  # carriage return x axis

            # distance between each filter
            xDistance = 2*params[5] + params[7] + \
                2*sqrt(params[1]*params[0]/pi)
            yDistance = 3*params[5] + params[7]

            parallelFilters = width//xDistance
            print('adding {} parallel filters to current sequence..'.format(
                parallelFilters))

            ########### INITIALIZE ###########
            if areaSolution == 0:
                # first filter sequence sets areaSolution (currently just one row is initial sequence of filters)
                areaSolution = parallelFilters*self.crossSectionalArea(*params)
                areaBuffer = areaSolution
                print('initialized areaSolution: {} areaBuffer: {}'.format(
                    areaSolution, areaBuffer))

            if curPosition[1] > length:
                # we have reached the end of the rectangle
                print('EOL: filter sequence')
                # TODO: remove manifolds in dump log
                print('dumping {} buffer items from last sequence clipping'.format(
                    len(arrayBuffer)))
                arrayBuffer.clear()
                break

            ########### ADD FILTERS TO CURRENT SEQUENCE###########
            # TODO: some of this can likely be refactored out to function calls
            print('parallelFilters {} mod 2 is {}'.format(
                parallelFilters, parallelFilters % 2))

            intakeRadius = sqrt(params[0]*params[1]/pi)

            if parallelFilters % 2 != 0:
                if areaBuffer <= 0:
                    # reached parallel cross-sectional area equivalence
                    break

                print('odd')
                # start from origin then offset each filter origin+xDistance
                initialFilter = cycloneFilter(*params)
                # TODO: this always follows xDistance carriage return
                initialFilter = left(curPosition[0])(initialFilter)
                initialFilter = forward(curPosition[1])(initialFilter)

                # have to tap manifold access lines, since hole() cant be used due to vortex searcher
                manifoldAccess = cylinder(
                    r=intakeRadius - params[7], h=2*params[7])
                manifoldAccess = left(curPosition[0])(manifoldAccess)
                manifoldAccess = forward(curPosition[1])(manifoldAccess)

                initialFilter = initialFilter - hole()(manifoldAccess)

                curPosition[0] += xDistance

                arrayBuffer.append(initialFilter)
                areaBuffer -= self.crossSectionalArea(*params)
                print('areaBuffer: {}'.format(areaBuffer))

                # TODO: check rectangular prism bounds.
                for x in range(int((parallelFilters-1)//2)):
                    print('areaBuffer: {}'.format(areaBuffer))
                    if areaBuffer <= 0:
                        # reached parallel cross-section area equivalence
                        break

                    leftFilter, rightFilter = self.cycloneRow(
                        params, intakeRadius, curPosition)

                    #TODO: consider moving the below into cycloneRow since manifold mutates passed in buffer.
                    #      resolve this design divergence.
                    curPosition[0] += xDistance

                    arrayBuffer.append(leftFilter)
                    areaBuffer -= self.crossSectionalArea(*params)
                    arrayBuffer.append(rightFilter)
                    areaBuffer -= self.crossSectionalArea(*params)

                # TODO: verify cylinderRadius
                if areaBuffer > 0:
                    newSequence = True
                else:
                    newSequence = False

                manifold = self.cycloneManifold(
                        params, newSequence, intakeRadius, curPosition, xDistance,yDistance, width, manifoldCeiling)

                arrayBuffer.append(manifold)
            else:
                print('even')
                # start from origin + xDistance/2
                curPosition[0] += xDistance/2
                for x in range(int(parallelFilters//2)):  # parallel loop
                    print('areaBuffer: {}'.format(areaBuffer))
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

                #attach the manifold to this cyclone row
                if areaBuffer > 0:
                    newSequence = True
                else:
                    newSequence = False
                manifold = self.cycloneManifold(
                            params, newSequence, intakeRadius, curPosition, xDistance, yDistance, width, manifoldCeiling)

                arrayBuffer.append(manifold)

        return array

    def cycloneRow(self, params, intakeRadius, curPosition):
        '''
        returns a symmetric pair of cyclone filters used in creating rows within a sequence
        '''
        leftParams = params
        leftParams[2] = True
        # tap manifold from cylinder
        # TODO: ensure this doesnt clip vortex searcher
        leftManifoldAccess = cylinder(
            r=intakeRadius - params[7], h=2*params[7])
        leftManifoldAccess = left(curPosition[0])()(leftManifoldAccess)
        leftManifoldAccess = forward(curPosition[1])()(leftManifoldAccess)

        leftFilter = cycloneFilter(*leftParams)
        leftFilter = left(curPosition[0])(leftFilter)
        leftFilter = forward(curPosition[1])(leftFilter)

        leftFilter = leftFilter - hole()(leftManifoldAccess)

        rightParams = params
        rightParams[2] = False
        # tap manifold from cylinder
        rightManifoldAccess = cylinder(
            r=intakeRadius - params[7], h=2*params[7])
        rightManifoldAccess = right(curPosition[0])()(rightManifoldAccess)
        rightManifoldAccess = forward(curPosition[1])()(rightManifoldAccess)

        rightFilter = cycloneFilter(*rightParams)
        rightFilter = right(curPosition[0])(rightFilter)
        rightFilter = forward(curPosition[1])(rightFilter)

        rightFilter = rightFilter - hole()(rightManifoldAccess)
        return leftFilter, rightFilter

    def cycloneManifold(self, params, isParallel, intakeRadius, curPosition, xDistance, yDistance, width, manifoldCeiling):
        '''
        build the manifold in this row connecting the previous and next rows while considering sequencing.
        '''
        #NOTE: exhaust conflicts with manifold when smaller, for now constrain final_params accordingly.
        # spans entire row, height and width set to volume constraint
        exhaustSpan = 2*(curPosition[0] - xDistance + intakeRadius)
        intakeSpan = yDistance - 2*params[5] - 2*params[7]

        #build sequence or parallel manifold splice
        if isParallel:
            exhaustLength = yDistance #connect in parallel 
            #@DEPRECATED
            # intakeManifold = None #hack
        else:
            #on sequence iteration (here), create L pipe 
            exhaustLength = params[5] + intakeRadius #connect in series
   
        #build intake manifold
        intakeManifoldSolid = cube(
            [width + params[7], intakeSpan, params[5] + params[7]])
        intakeManifold = intakeManifoldSolid - hole()(up(params[7]/2)(right(params[7]/2)(forward(params[7]/2)(cube(
            [width, intakeSpan-params[7], params[5]])))))
        
        intakeManifold = left(width/2)(intakeManifold)
        intakeManifold = down(params[0])(intakeManifold)
        
        intakeManifold = back(params[5] + intakeSpan + params[7])(intakeManifold)
        intakeManifold = forward(curPosition[1])(intakeManifold)

        # build the exhaust manifold 
        exhaustManifoldSolid = cube(
            [width/2 + params[7], exhaustLength, manifoldCeiling + params[7]])

        #TODO: ensure manifold exhaust width/2 fits for very small residual rows in a sequence (less than 3)
        exhaustManifold = right(
            params[7]/2)(up(params[7]/2)(forward(params[7])(cube([width/2, exhaustLength, manifoldCeiling]))))
        exhaustManifold = exhaustManifoldSolid - hole()(exhaustManifold)

        exhaustManifold = right(
            exhaustSpan/2 + params[7] - width/4)(exhaustManifold)
        exhaustManifold = forward(params[5]/2)(exhaustManifold)

        # now build the actual pipeline connecting the rows of cyclones
        exhaustPipelineSolid = cube(
            [exhaustSpan+params[7], 2*intakeRadius + params[7], manifoldCeiling + params[7]])

        exhaustPipeline = up(params[7]/2)(right(params[7]/2)(hull()
                                                            (cube([exhaustSpan, 2*intakeRadius, manifoldCeiling]))))
        exhaustPipeline = exhaustPipelineSolid - \
            hole()(forward(params[7]/2)(exhaustPipeline))

        exhaustPipeline = exhaustPipeline + exhaustManifold
        # position atop the array
        exhaustPipeline = forward(curPosition[1]-intakeRadius)(exhaustPipeline)
        exhaustPipeline = left(exhaustSpan/2)(exhaustPipeline)
        exhaustPipeline = up(params[7])(exhaustPipeline)

        # TODO: now go up and over
        #TODO: also create inlet manifold
        manifold = exhaustPipeline + intakeManifold 
        return manifold #TODO: now return the exhaust as well.
        
