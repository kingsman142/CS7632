'''
 * Copyright (c) 2014, 2015 Entertainment Intelligence Lab, Georgia Institute of Technology.
 * Originally developed by Mark Riedl.
 * Last edited by Mark Riedl 05/2015
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
'''

import sys, pygame, math, numpy, random, time, copy, operator
from pygame.locals import *

from constants import *
from utils import *
from core import *

# Creates the path network as a list of lines between all path nodes that are traversable by the agent.
def myBuildPathNetwork(pathnodes, world, agent = None):
    lines = []
    ### YOUR CODE GOES BELOW HERE ###
    min_allowed_distance = agent.getMaxRadius()
    obstacles = world.getObstacles()
    obstacles_corners = [obstacle.getPoints() for obstacle in obstacles]

    # for each possible line between path nodes, make sure that line doesn't intersect an obstacle and it's far enough away from any obstacle
    for i in range(0, len(pathnodes)):
        for j in range(i+1, len(pathnodes)):
            point_begin = pathnodes[i]
            point_end = pathnodes[j]
            intersection = False # does the line between point_begin and point_end intersect any obstacles or come within min_allowed_distance?

            for obstacle_corners in obstacles_corners: # check for corner detection
                for corner in obstacle_corners:
                    if minimumDistance((point_begin, point_end), corner) <= min_allowed_distance:
                        intersection = True
            if not intersection and not rayTraceWorld(point_begin, point_end, world.getLines()): # check for intersection detection
                lines.append((pathnodes[i], pathnodes[j]))
    ### YOUR CODE GOES ABOVE HERE ###
    return lines
