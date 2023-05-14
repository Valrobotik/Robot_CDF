#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Path: Valrob_pkg\script\test_node.py

#algorithme d'evitement d'obstacle

#import rospy # type: ignore
import matplotlib.pyplot as plt # type: ignore

# maps sur la quelle on travaille
class maps():
    def __init__(self) -> None:
        self.x_max = 3
        self.y_max = 1
        self.x_min = 0
        self.y_min = -1

        self.__resolution = 0.01

        self.__map = [[0 for i in range(int((self.x_max - self.x_min)/self.__resolution))] for j in range(int((self.y_max - self.y_min)/self.__resolution))]

    def set_obstacle(self, x, y):
        self.__map[int((y-self.y_min)/self.__resolution)][int((x-self.x_min)/self.__resolution)] = 1
    
    def set_obstacle_ligne(self, x1, y1, x2, y2):
        dy = y2 - y1
        dx = x2 - x1
        
        xmax = max(x1, x2)
        xmin = min(x1, x2)
        ymax = max(y1, y2)
        ymin = min(y1, y2)

        if abs(dx) > abs(dy):
            a = dy/dx
            b = y1 - a*x1
            for x in range(int(xmin/self.__resolution), int(xmax/self.__resolution)):
                y = a*x*self.__resolution + b
                self.set_obstacle(x*self.__resolution, y)
        else:
            a = dx/dy
            b = x1 - a*y1
            for y in range(int(ymin/self.__resolution), int(ymax/self.__resolution)):
                x = a*y*self.__resolution + b
                self.set_obstacle(x, y*self.__resolution)

    def get_obstacle(self, x, y):
        return self.__map[int((y-self.y_min)/self.__resolution)][int((x-self.x_min)/self.__resolution)]
    
    def get_obstacle_indice(self, i, j):
        return self.__map[int(i)][int(j)]
    
    def get_indice(self, x, y):
        return (int((y-self.y_min)/self.__resolution), int((x-self.x_min)/self.__resolution))
    
    def get_position(self, i, j):
        return (self.x_min + i*self.__resolution, self.y_min + j*self.__resolution)
    
    def display_map(self):
        plt.imshow(self.__map)
        plt.show()

    def resolution(self):
        return self.__resolution
    
    def get(self):
        return self.__map

    def obstacle_proximite(self, x, y):
        """renvoie 1 si un obstacle est a proximite du point (x, y)"""
        for i in range(-4, 6):
            for j in range(-4, 6):
                if self.get_obstacle(x+i*self.__resolution, y+j*self.__resolution) == 1:
                    return 1
        return 0
    
    def obstacle_proximite_indice(self, i, j):
        """renvoie 1 si un obstacle est a proximite du point (x, y)"""
        for k in range(-4, 6):
            for l in range(-4, 6):
                if self.get_obstacle_indice(i+k, j+l) == 1:
                    return 1
        return 0

    def distance_obstacle(self, x, y):
        """retourne la distance avec l'obstacle le plus proche et ces coordonnees"""
        ir = -1
        jr = -1
        distance = 100
        for i in range(-2, 3):
            for j in range(-2, 3):
                if self.get_obstacle(x+i*self.__resolution, y+j*self.__resolution) == 1:
                    distance = min(distance, (i**2 + j**2)**0.5)
                    ir = i
                    jr = j
        return (distance, self.get_position(ir, jr))

    def distance_obstacle_indice(self, i, j):
        """retourne la distance avec l'obstacle le plus proche et ces coordonnees"""
        ir = -1
        jr = -1
        distance = 100
        for k in range(-2, 3):
            for l in range(-2, 3):
                if self.get_obstacle_indice(int(i+k), int(j+l)) == 1:
                    distance = min(distance, (k**2 + l**2)**0.5)
                    ir = k
                    jr = l
        return (distance, (i+ir, j+jr))

# algorithme de planificateur de trajectoire
class path_planner():
    def __init__(self) -> None:
        x_robot : float = 0
        y_robot : float = 0
        x_goal : float = 0
        y_goal : float = 0
        self.__maps = maps()
        self.initMap()
        self.__path = []
    
    def resest_map(self):
        self.__maps = maps()
        self.initMap()
        self.__path = []

    def initMap(self):
        #initialisation de la map de test avec des obstacles fixes
        self.__maps.set_obstacle_ligne(0.8, 0, 1.5, 0.5)

  

    
    def set_robot(self, x, y):
        self.x_robot = x
        self.y_robot = y
    
    def set_goal(self, x, y):
        self.x_goal = x
        self.y_goal = y
    
    def proximity(selfr, i1, j1, i2, j2):
        """renvoie 1 si les points (i1, j1) et (i2, j2) sont proches"""
        if (i1-i2)**2 + (j1-j2)**2 < 2:
            return 1
        return 0

    def calc_path(self):
        """calcule le chemin entre le robot et le goal en evitant les obstacles"""
        i, j = self.__maps.get_indice(self.x_robot, self.y_robot)
        print(i, j)
        i_goal, j_goal = self.__maps.get_indice(self.x_goal, self.y_goal)
        while not self.proximity(i, j, i_goal, j_goal):
            if self.__maps.obstacle_proximite_indice(i, j):
                i, j = self.avoid_obstacle(i, j, i_goal, j_goal)
            else:
                i, j = self.go_to_goal(i, j , i_goal, j_goal)
            self.__path.append((i, j))

    def go_to_goal(self, i, j, i_goal, j_goal):
        """calcule le chemin le plus court entre le robot et le goal en ligne droite"""
        di = i_goal - i
        dj = j_goal - j
        i_max = max(i, i_goal)
        i_min = min(i, i_goal)
        j_max = max(j, j_goal)
        j_min = min(j, j_goal)
        if abs(di) > abs(dj):
            a = dj/di
            b = j - a*i
            i = i_min+1
            j = a*i + b
        else :
            a = di/dj
            b = i - a*j
            j = j_min+1
            i = a*j + b
        return (int(i), int(j))

    def avoid_obstacle(self,i, j, i_goal, j_goal):
        """calcule le chemin le plus court entre le robot et le goal en evitant les obstacles"""
        distance, (i_obstacle, j_obstacle) = self.__maps.distance_obstacle_indice(i, j)
        i_next, j_next = self.go_to_goal(i, j, i_goal, j_goal)
        i_prev, j_prev = i, j 
        distance_next, (i_obstacle_next, j_obstacle_next) = self.__maps.distance_obstacle_indice(i_next, j_next)
        if distance_next < distance:
            return (i_next, j_next)
        else:
            ei = i_obstacle - i
            ej = j_obstacle - j
            if abs(ei) < abs(ej):
                j+=1
                if self.__maps.distance_obstacle_indice(i, j)[0] > distance:
                    j-=2
            else:
                i+=1
                if self.__maps.distance_obstacle_indice(i, j)[0] > distance:
                    i-=2
        print(i, j, i_prev, j_prev, i_obstacle, j_obstacle, i_obstacle_next, j_obstacle_next)
        return (i,j)
       
        
    
    def display_path(self):
        carte = self.__maps.get()
        for (i,j) in self.__path:
            carte[int(i)][int(j)] = 2
        plt.imshow(carte)
        plt.show()

test = path_planner()
test.set_robot(0, 0)
test.set_goal(2, 0.2)
test.calc_path()
test.display_path()

    