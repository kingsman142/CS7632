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

import sys, pygame, math, numpy, random, time, copy
from pygame.locals import *

from constants import *
from utils import *
from core import *



###############################
### AStarNavigator2
###
### Creates a path node network and implements the A* algorithm to create a path to the given destination.

class AStarNavigator2(PathNetworkNavigator):


	### Finds the shortest path from the source to the destination using A*.
	### self: the navigator object
	### source: the place the agent is starting from (i.e., its lowest_cost_node_location location)
	### dest: the place the agent is told to go to
	def computePath(self, source, dest):
		self.setPath(None)
		### Make sure the next and dist matrices exist
		if self.agent != None and self.world != None:
			self.source = source
			self.destination = dest
			### Step 1: If the agent has a clear path from the source to dest, then go straight there.
			### Determine if there are no obstacles between source and destination (hint: cast rays against world.getLines(), check for clearance).
			### Tell the agent to move to dest
			if clearShot(source, dest, self.world.getLinesWithoutBorders(), self.world.getPoints(), self.agent):
				self.agent.moveToTarget(dest)
			else:
				### Step 2: If there is an obstacle, create the path that will move around the obstacles.
				### Find the path nodes closest to source and destination.
				start = getOnPathNetwork(source, self.pathnodes, self.world.getLinesWithoutBorders(), self.agent)
				end = getOnPathNetwork(dest, self.pathnodes, self.world.getLinesWithoutBorders(), self.agent)
				if start != None and end != None:
					### Remove edges from the path network that intersect gates
					newnetwork = unobstructedNetwork(self.pathnetwork, self.world.getGates(), self.world)
					closedlist = []
					### Create the path by traversing the pathnode network until the path node closest to the destination is reached
					path, closedlist = astar(start, end, newnetwork)
					if path is not None and len(path) > 0:
						### Determine whether shortcuts are available
						path = shortcutPath(source, dest, path, self.world, self.agent)
						### Store the path by calling self.setPath()
						self.setPath(path)
						if self.path is not None and len(self.path) > 0:
							### Tell the agent to move to the first node in the path (and pop the first node off the path)
							first = self.path.pop(0)
							self.agent.moveToTarget(first)
		return None

	### Called when the agent gets to a node in the path.
	### self: the navigator object
	def checkpoint(self):
		myCheckpoint(self)
		return None

	### This function gets called by the agent to figure out if some shortcuts can be taken when traversing the path.
	### This function should update the path and return True if the path was updated.
	def smooth(self):
		return mySmooth(self)

	def update(self, delta):
		myUpdate(self, delta)


### Removes any edge in the path network that intersects a worldLine (which should include gates).
def unobstructedNetwork(network, worldLines, world):
	newnetwork = []
	for l in network:
		hit = rayTraceWorld(l[0], l[1], worldLines)
		if hit == None:
			newnetwork.append(l)
	return newnetwork



### Returns true if the agent can get from p1 to p2 directly without running into an obstacle.
### p1: the lowest_cost_node_location location of the agent
### p2: the destination of the agent
### worldLines: all the lines in the world
### agent: the Agent object
def clearShot(p1, p2, worldLines, worldPoints, agent):
	### YOUR CODE GOES BELOW HERE ###
	min_allowed_distance = agent.getMaxRadius()

	# for each possible line to a path node, make sure that line doesn't intersect an obstacle and it's far enough away from any obstacle
	intersection = False # does the line between p1 and p2 intersect any obstacles or come within min_allowed_distance?
	for obstacle_corner in worldPoints: # check for corner detection
		if minimumDistance((p1, p2), obstacle_corner) <= min_allowed_distance:
			intersection = True
	# for each path node the agent can travel to, check to see if it intersects with any world lines (i.e. obstacles and gates)
	collisionBetweenPathNodeAndWorldLines = False
	for line in worldLines:
		if minimumDistance(line, p2) <= min_allowed_distance:
			collisionBetweenPathNodeAndWorldLines = True

	# finally, make sure all the checks from above pass
	if not intersection and not rayTraceWorld(p1, p2, worldLines) and not collisionBetweenPathNodeAndWorldLines: # check for intersection detection
		return True
	### YOUR CODE GOES ABOVE HERE ###
	return False

### Given a location, find the closest pathnode that the agent can get to without collision
### agent: the agent
### location: the location to check from (typically where the agent is starting from or where the agent wants to go to) as an (x, y) point
### pathnodes: a list of pathnodes, where each pathnode is an (x, y) point
### world: pointer to the world
def getOnPathNetwork(location, pathnodes, worldLines, agent):
	node = None
	### YOUR CODE GOES BELOW HERE ###
	min_distance = None
	min_pathnode_location = None

	# grab the world points to plug into clearShot()
	worldPoints = set()
	for line in worldLines:
		worldPoints.add(line[0])
		worldPoints.add(line[1])
	worldPoints = list(worldPoints)

	# for each path node, find the node that's closest to the agent that we can actually travel to
	for pathnode in pathnodes:
		distance_between_locations = distance(location, pathnode) # distance between agent and path node
		if min_distance is None or distance_between_locations < min_distance: # set the min distance if it doesn't exist already
			if clearShot(location, pathnode, worldLines, [], agent):
				min_distance = distance_between_locations
				min_pathnode_location = pathnode

	node = min_pathnode_location
	### YOUR CODE GOES ABOVE HERE ###
	return node



### Implement the a-star algorithm
### Given:
### Init: a pathnode (x, y) that is part of the pathnode network
### goal: a pathnode (x, y) that is part of the pathnode network
### network: the pathnode network
### Return two values:
### 1. the path, which is a list of states that are connected in the path network
### 2. the closed list, the list of pathnodes visited during the search process
def astar(init, goal, network):
	path = []
	open = []
	closed = []
	### YOUR CODE GOES BELOW HERE ###
	parents = {}
	distances = {init: 0}
	heuristics = {init: distances[init] + distance(init, goal)}
	open.append(init)

	while len(open) > 0:
		# find node with lowest heuristic cost
		new_arr = [(item, heuristics[item]) for item in open]
		lowest_cost = min(new_arr, key = lambda x : x[1])
		lowest_cost_index = new_arr.index(lowest_cost)
		lowest_cost_node_location = new_arr[lowest_cost_index][0]

		if lowest_cost_node_location == goal: # we are finished, so traverse back from the destination node to each node's lowest cost parent
			path.append(lowest_cost_node_location)
			while lowest_cost_node_location in parents:
				lowest_cost_node_location = parents[lowest_cost_node_location]
				if lowest_cost_node_location == init:
					break
				else:
					path.insert(0, lowest_cost_node_location)
			path.insert(0, lowest_cost_node_location) # lowest_cost_node_location should be equal to init
			break
		else: # keep navigating along the lowest cost path
			open = open[0:lowest_cost_index] + open[(lowest_cost_index+1):]
			closed.append(lowest_cost_node_location)

			# find all neighbors of this node so we can iterate further
			neighbors = set()
			for line in network:
				if lowest_cost_node_location in line:
					neighbors.add(line[0])
					neighbors.add(line[1])
			neighbors.remove(lowest_cost_node_location)

			for neighbor in neighbors: # add all of this node's neighbors so we can continue with A*
				if neighbor not in closed: # make sure we haven't already visited this node
					neighbor_heuristic = distances[lowest_cost_node_location] + distance(lowest_cost_node_location, neighbor)
					if not neighbor in open or neighbor_heuristic < distances[neighbor]: # did we find a better path to this node/neighbor than exists already?
						if not neighbor in open:
							open.append(neighbor)
						distances[neighbor] = neighbor_heuristic
						heuristics[neighbor] = distances[neighbor] + distance(neighbor, goal)
						parents[neighbor] = lowest_cost_node_location
	### YOUR CODE GOES ABOVE HERE ###
	return path, closed


def myUpdate(nav, delta):
	### YOUR CODE GOES BELOW HERE ###
	gates = nav.world.getGates()

	if rayTraceWorld(nav.agent.getLocation(), nav.agent.moveTarget, gates) is not None: # there is an intersection between this agent's min-path (from A*) and the gates
		nav.setPath(None)
		nav.agent.stopMoving()
	### YOUR CODE GOES ABOVE HERE ###
	return None




def myCheckpoint(nav):
	### YOUR CODE GOES BELOW HERE ###
	gates = nav.world.getGates()

	if rayTraceWorld(nav.agent.getLocation(), nav.agent.moveTarget, gates) is not None: # there is an intersection between this agent's min-path (from A*) and the gates
		nav.setPath(None)
		nav.agent.stopMoving()
	### YOUR CODE GOES ABOVE HERE ###
	return None







### This function optimizes the given path and returns a new path
### source: the lowest_cost_node_location position of the agent
### dest: the desired destination of the agent
### path: the path previously computed by the A* algorithm
### world: pointer to the world
def shortcutPath(source, dest, path, world, agent):
	path = copy.deepcopy(path)
	### YOUR CODE GOES BELOW HERE ###
	start_index = 0
	end_index = len(path) - 1

	# find the optimal starting point
	for node_index, node in enumerate(path):
		if clearShot(source, node, world.getLinesWithoutBorders(), world.getPoints(), agent):
			start_index = node_index
	path = path[start_index:]

	# find the optimal ending point
	for node_index, node in reversed(list(enumerate(path))):
		if clearShot(dest, node, world.getLinesWithoutBorders(), world.getPoints(), agent):
			end_index = node_index
	path = path[:(end_index+1)]

	# do string pulling on all the nodes in between
	for node_index, node in enumerate(path):
		skip_to_index = None
		if (node_index + 2) < (len(path) - 1):
			for skip_index in range(node_index + 2, len(path)):
				if clearShot(path[node_index], path[skip_index], world.getLines(), world.getPoints(), agent):
					skip_to_index = skip_index
		if not skip_to_index is None:
			path = path[0:(node_index+1)] + path[skip_to_index:]
	### YOUR CODE GOES BELOW HERE ###
	return path


### This function changes the move target of the agent if there is an opportunity to walk a shorter path.
### This function should call nav.agent.moveToTarget() if an opportunity exists and may also need to modify nav.path.
### nav: the navigator object
### This function returns True if the moveTarget and/or path is modified and False otherwise
def mySmooth(nav):
	### YOUR CODE GOES BELOW HERE ###
	world = nav.world
	agent = world.agent
	destination = nav.getDestination()

	# if we are going toward a destination and there's a clear shot between the agent's location and the final destination, then just move directly there
	if not destination is None and clearShot(agent.getLocation(), destination, world.getLines(), world.getPoints(), agent):
		agent.moveToTarget(destination)
		return True
	### YOUR CODE GOES ABOVE HERE ###
	return False
