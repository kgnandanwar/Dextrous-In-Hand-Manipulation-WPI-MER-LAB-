#!/usr/bin/env python3

import numpy as np
import math
from collections import deque

class Node:
	"""docstring for Node"""
	def __init__(self, arr):
		self.arr = arr
		self.g = None
		self.h = None
		self.cost = None
		self.parent = None
		self.step = None


def get_setup():
	s_p1 = [-1.25, 0, 3.5]
	s_p2 = [0, -1.25, 3.5]
	s_p3 = [1.25, 0, 3.5]

	# g_p1 = [-1.25, 0, 1.5]
	# g_p2 = [0, -1.25, 1.5]
	# g_p3 = [1.25, 0, 1.5]

	g_p1 = [0, -1.25, 0.5]
	g_p2 = [1.25, 0, 0.5]
	g_p3 = [0, 1.25, 0.5]

	start = np.array([s_p1, s_p2, s_p3])
	goal = np.array([g_p1, g_p2, g_p3])

	return start, goal


def neighbours(s_node):
	fingers = s_node.arr
	directions = ["UP", "DOWN", "CW", "CCW"]
	n_list = []
	for d in directions:
		temp = np.array(fingers, copy=True)  
		if  d == "UP":
			if fingers[0][2] + 1 < 4:
				temp[0][2] += 1
				temp[1][2] += 1
				temp[2][2] += 1
				n_list.append((d, temp))
		if d == "DOWN":
			if fingers[0][2] - 1 > -4:
				temp[0][2] -= 1
				temp[1][2] -= 1
				temp[2][2] -= 1
				n_list.append((d, temp))
		if d == "CW" or d == "CCW":
			temp[0] = rot_point(temp[0], d)
			temp[1] = rot_point(temp[1], d)
			temp[2] = rot_point(temp[2], d)
			n_list.append((d, temp))

	return n_list

def rot_point(point, direction):
	if direction == "CW":
		theta = np.pi/2
	if direction == "CCW":
		theta = -np.pi/2
	rot_mat = np.array([[round(math.cos(theta), 2), round(-math.sin(theta), 2), 0],
                        [round(math.sin(theta), 2), round(math.cos(theta), 2), 0],
                        [0, 0, 1]])
	new_point = rot_mat.dot(point)
	return new_point

def gcost(direction):
	if direction == "UP" or direction == "DOWN":
		return 1.0
	if direction == "CW" or direction == "CCW":
		return 5.5


def astar(start, goal):
	found = False
	visit = deque([])
	q = []
	steps = 0
	path = []
	if np.array_equal(start, goal, equal_nan=False):
		return path, steps

	s_node = Node(start)
	s_node.cost = 0
	s_node.g = 0
	s_node.h = 0

	visit.append((tuple(start[0]),tuple(start[1]),tuple(start[2])))


	while not found:
		if np.array_equal(s_node.arr, goal, equal_nan=False):
			found = True
			break
		else:
			for node in neighbours(s_node):
				fingers = node[1]

				if (tuple(fingers[0]),tuple(fingers[1]),tuple(fingers[2])) not in visit:
					h_total = 0
					for i in range(len(fingers)):
						h = 0 
						h = np.sqrt((fingers[i][0]-goal[i][0])**2 + (fingers[i][1]-goal[i][1])**2 + (fingers[i][2]-goal[i][2])**2)
						h_total += h

					temp_obj = Node(fingers)
					temp_obj.h = h_total
					temp_obj.parent = s_node
					temp_obj.g = s_node.g + gcost(node[0])
					temp_obj.cost = temp_obj.g + temp_obj.h
					temp_obj.step = node[0]
					q.append((temp_obj, temp_obj.cost))
					

			q.sort(key = lambda x:x[1])
			priority = q.pop(0)
			s_node = priority[0]
			visit.append((tuple(s_node.arr[0]),tuple(s_node.arr[1]),tuple(s_node.arr[2])))

	steps = len(visit)
	while s_node.parent is not None:
		path.append(s_node.step)
		s_node = s_node.parent
	path.reverse()

	return path, steps


if __name__ == "__main__":
	start, goal = get_setup()
	path, steps = astar(start, goal)
	# print(start[0][2], "start")
	# print("goal", goal)
	print(path)
