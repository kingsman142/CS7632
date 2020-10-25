from constants import *
from utils import *
from core import *

import pdb
import copy
from functools import reduce

from statesactions import *

############################
## HELPERS

### Return true if the given state object is a goal. Goal is a State object too.
def is_goal(state, goal):
	return len(goal.propositions.difference(state.propositions)) == 0

### Return true if the given state is in a set of states.
def state_in_set(state, set_of_states):
	for s in set_of_states:
		if s.propositions != state.propositions:
			return False
	return True

### For debugging, print each state in a list of states
def print_states(states):
	for s in states:
		ca = None
		if s.causing_action is not None:
			ca = s.causing_action.name
		print(s.id, s.propositions, ca, s.get_g(), s.get_h(), s.get_f())


############################
### Planner
###
### The planner knows how to generate a plan using a-star and heuristic search planning.
### It also knows how to execute plans in a continuous, time environment.

class Planner():

	def __init__(self):
		self.running = False              # is the planner running?
		self.world = None                 # pointer back to the world
		self.the_plan = []                # the plan (when generated)
		self.initial_state = None         # Initial state (State object)
		self.goal_state = None            # Goal state (State object)
		self.actions = []                 # list of actions (Action objects)

	### Start running
	def start(self):
		self.running = True

	### Stop running
	def stop(self):
		self.running = False

	### Called every tick. Executes the plan if there is one
	def update(self, delta = 0):
		result = False # default return value
		if self.running and len(self.the_plan) > 0:
			# I have a plan, so execute the first action in the plan
			self.the_plan[0].agent = self
			result = self.the_plan[0].execute(delta)
			if result == False:
				# action failed
				print("AGENT FAILED")
				self.the_plan = []
			elif result == True:
				# action succeeded
				done_action = self.the_plan.pop(0)
				print("ACTION", done_action.name, "SUCCEEDED")
				done_action.reset()
		# If the result is None, the action is still executing
		return result

	### Call back from Action class. Pass through to world
	def check_preconditions(self, preconds):
		if self.world is not None:
			return self.world.check_preconditions(preconds)
		return False

	### Call back from Action class. Pass through to world
	def get_x_y_for_label(self, label):
		if self.world is not None:
			return self.world.get_x_y_for_label(label)
		return None

	### Call back from Action class. Pass through to world
	def trigger(self, action):
		if self.world is not None:
			return self.world.trigger(action)
		return False

	### Generate a plan. Init and goal are State objects. Actions is a list of Action objects
	### Return the plan and the closed list
	def astar(self, init, goal, actions):
		plan = []    # the final plan
		open = []    # the open list (priority queue) holding State objects
		closed = []  # the closed list (already visited states). Holds state objects
		### YOUR CODE GOES HERE
		parents = {}
		distances = {init: 0}
		heuristics = {init: distances[init] + self.compute_heuristic(init, goal, actions)}
		open.append(init)

		while len(open) > 0:
			# find node with lowest heuristic cost
			new_arr = [(item, heuristics[item]) for item in open]
			lowest_cost = min(new_arr, key = lambda x : x[1])
			lowest_cost_index = new_arr.index(lowest_cost)
			lowest_cost_node_location = new_arr[lowest_cost_index][0]

			if is_goal(lowest_cost_node_location, goal): # we are finished, so traverse back from the destination node to each node's lowest cost parent
				while lowest_cost_node_location in parents:
					# find the action that led from this state's parent to the current state, and append it to the plan
					plan.insert(0, lowest_cost_node_location.causing_action)

					# continue backtracking
					lowest_cost_node_location = parents[lowest_cost_node_location]
					if is_goal(lowest_cost_node_location, init):
						break
				break
			else: # keep navigating along the lowest cost path
				open = open[0:lowest_cost_index] + open[(lowest_cost_index+1):] # pop off lowest cost node
				closed.append(lowest_cost_node_location)

				# find all neighbors of this node so we can iterate further
				successors = []
				for action in actions:
					if action.preconditions.issubset(lowest_cost_node_location.propositions):
						new_state = State(propositions = lowest_cost_node_location.propositions.union(action.add_list) - action.delete_list)
						new_state.parent = lowest_cost_node_location
						new_state.causing_action = action
						successors.append(new_state)

				# add all of this node's successors so we can continue with A*
				for successor in successors:
					if not state_in_set(successor, closed): # make sure we haven't already visited this node
						successor_distance = distances[lowest_cost_node_location] + 1 # distance to this neighbor/successor in the graph thus far
						if not successor in open or successor_distance < distances[successor]: # did we find a better path to this node/successor than exists already?
							if not successor in open:
								open.append(successor)
							distances[successor] = successor_distance
							heuristics[successor] = distances[successor] + self.compute_heuristic(successor, goal, actions) # heuristic = distance travelled + estimated remaining distance left
							parents[successor] = lowest_cost_node_location # used for backtracking
		### CODE ABOVE
		return plan, closed

	### Compute the heuristic value of the current state using the HSP technique.
	### Current_state and goal_state are State objects.
	def compute_heuristic(self, current_state, goal_state, actions):
		actions = copy.deepcopy(actions)  # Make a deep copy just in case
		h = 0                             # heuristic value to return
		### YOUR CODE BELOW
		# initialize some dummy variables
		dummy_goal = Action(name = "dummy_goal_transition", preconditions = goal_state.propositions, add_list = [], delete_list = [])
		dummy_start = Action(name = "empty_current_state_transition", add_list = list(current_state.propositions), delete_list = [], preconditions = [])
		actions.append(dummy_goal) # make sure we know when we reach the dummy goal so we can return the heuristic value

		# create the graph
		graph = {}
		for preceding_action in actions:
			for next_action in actions:
				# one of the add_list propositions in action matches a proposition in the preconditions of next_action
				shared_propositions = preceding_action.add_list.intersection(next_action.preconditions)
				if not preceding_action == next_action and len(shared_propositions) > 0:
					if not next_action in graph:
						graph[next_action] = []
					for proposition in shared_propositions:
						graph[next_action].append((proposition, preceding_action))

		# walk the graph
		visited_nodes = []
		visited_propositions = []
		distances = {}
		to_visit = [dummy_start]
		while len(to_visit) > 0:
			# mark that we just visited this node, as well as its propositions, as we'll be able to find the next available states easier
			next_action = to_visit.pop(0) # pop off FIFO queue
			visited_nodes.append(next_action)
			visited_propositions += next_action.add_list

			for action in actions:
				# check to see if this action's predecessors have all been visited
				all_preconditions_satisfied = True
				for precondition in action.preconditions:
					if not precondition in visited_propositions:
						all_preconditions_satisfied = False
						break

				# all predecessors of this action have been visited
				if all_preconditions_satisfied and not action in visited_nodes and not action in to_visit:
					to_visit.append(action)

			# compute distance of this new action
			if next_action not in distances:
				distances[next_action] = 0
			incoming_edge_weights = [0]
			if next_action in graph:
				for edge in graph[next_action]: # look at all predecessors (or incoming edges) of this node, and calculate their incoming edge weight
					preceding_action = edge[1]
					if preceding_action in visited_nodes: # make sure the incoming edge has been visited; if not, we won't have a valid distance for it yet
						incoming_edge_weights.append(distances[preceding_action] + preceding_action.cost)

			distances[next_action] = max(incoming_edge_weights) # calculate the actual distance of this current node we're on
			h = max(h, distances[next_action])

		if dummy_goal in distances:
			h = distances[dummy_goal]
		### YOUR CODE ABOVE
		return h
