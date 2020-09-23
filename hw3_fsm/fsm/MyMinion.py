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
from moba import *

class MyMinion(Minion):

	def __init__(self, position, orientation, world, image = NPC, speed = SPEED, viewangle = 360, hitpoints = HITPOINTS, firerate = FIRERATE, bulletclass = SmallBullet):
		Minion.__init__(self, position, orientation, world, image, speed, viewangle, hitpoints, firerate, bulletclass)
		self.states = [Idle]
		### Add your states to self.states (but don't remove Idle)
		### YOUR CODE GOES BELOW HERE ###
		self.states += [AttackBuilding, Move]
		### YOUR CODE GOES ABOVE HERE ###

	def start(self):
		Minion.start(self)
		self.changeState(Idle)





############################
### Idle
###
### This is the default state of MyMinion. The main purpose of the Idle state is to figure out what state to change to and do that immediately.

class Idle(State):

	def enter(self, oldstate):
		State.enter(self, oldstate)
		# stop moving
		self.agent.stopMoving()

	def execute(self, delta = 0):
		State.execute(self, delta)
		### YOUR CODE GOES BELOW HERE ###
		my_team_symbol = self.agent.getTeam()
		enemy_base = self.agent.world.getEnemyBases(my_team_symbol)[0] # base(s) belonging to enemy

		self.agent.changeState(Move, enemy_base)
		### YOUR CODE GOES ABOVE HERE ###
		return None

##############################
### Taunt
###
### This is a state given as an example of how to pass arbitrary parameters into a State.
### To taunt someome, Agent.changeState(Taunt, enemyagent)

class Taunt(State):

	def parseArgs(self, args):
		self.victim = args[0]

	def execute(self, delta = 0):
		if self.victim is not None:
			print("Hey " + str(self.victim) + ", I don't like you!")
		self.agent.changeState(Idle)

##############################
### YOUR STATES GO HERE:

class AttackBuilding(State):
	def execute(self, delta = 0):
		# check the target's distance; if it's too far or not visible, we can't attack it
		target_distance = distance(self.agent.getLocation(), self.target.getLocation())
		if not self.target in self.agent.getVisible() or target_distance > BULLETRANGE or not self.target.isAlive():
			self.agent.changeState(Idle)
		else: # else, shoot at it
			self.agent.turnToFace(self.target.getLocation())
			self.agent.shoot()

	def enter(self, oldstate):
		self.agent.navigator.path = None
		self.agent.navigator.destination = None

	def exit(self):
		return

	def parseArgs(self, args):
		self.target = args[0]

class Move(State):
	def execute(self, delta = 0):
		my_team_symbol = self.agent.getTeam()
		towers = self.agent.world.getEnemyTowers(my_team_symbol) # towers belonging to enemy
		bases = self.agent.world.getEnemyBases(my_team_symbol) # base(s) belonging to enemy
		minions = self.agent.world.getEnemyNPCs(my_team_symbol) # minions belonging to enemy

		# if we're not moving, then move
		if self.agent.moveTarget is None and not self.target is None:
			self.agent.navigateTo(self.target.getLocation())

		# we can attack a tower, so attack it (towers must be destroyed before bases)
		for tower in towers:
			if tower in self.agent.getVisible() and distance(self.agent.getLocation(), tower.getLocation()) <= BULLETRANGE and tower.isAlive():
				self.agent.changeState(AttackBuilding, tower)

		# we can attack a base, so attack it
		for base in bases:
			if base in self.agent.getVisible() and distance(self.agent.getLocation(), base.getLocation()) <= BULLETRANGE and base.isAlive():
				self.agent.changeState(AttackBuilding, base)

	def enter(self, oldstate):
		if not self.target is None:
			self.agent.navigateTo(self.target.getLocation())

	def exit(self):
		return

	def parseArgs(self, args):
		self.target = args[0]
