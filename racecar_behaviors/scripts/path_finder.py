#!/usr/bin/env python

from dataclasses import dataclass
from math import floor, sqrt, inf
from typing import List, Tuple
import os
from matplotlib import pyplot as plt
from nav_msgs.srv import GetMap, GetMapResponse
import numpy as np
import rospy
import seaborn as sns
from racecar_behaviors.srv import BlobList, BlobListResponse
from racecar_behaviors.msg import BlobData
from blob_detector import Blob



@dataclass
class Point:
    """2D Cartesian point (x,y)."""
    x: float = 0
    y: float = 0

class PathFinder:
    def __init__(self, occupancyGrid=None, iterations: int = 5) -> None:
        self.map_height = 0
        self.map_width = 0
        self.map_resolution = 0.0
        self.origin_point = Point()
        self.map_heigth_m = 0.0
        self.map_width_m = 0.0
        self.blobs: List[Blob] = []
        if occupancyGrid is None:
            occupancyGrid = self.get_map_client()
        self.original_map = self.preface_map(occupancyGrid, 1)
        self.mapOfWorld = self.preface_map(occupancyGrid)
        self.generate_report_and_blob_list()
        self.execute_full_analysis(iterations)

    def execute_full_analysis(self, iterations):
        self.brushfire(iterations)
        start: Point = Point(0,0)
        for blob in self.blobs:
            goal: Point = Point(blob.x,blob.y)
            self.find_best_path(start, goal, blob.id)

        try:
            os.system("rm -rf ~/output_directory")
        except:
            pass

        os.system("cp -R ~/.ros/output_directory ~/")

    def generate_report_and_blob_list(self):
        rospy.wait_for_service('send_blob_data')
        try:
            get_blob_list = rospy.ServiceProxy('send_blob_data', BlobList)
            blob_response = get_blob_list(1)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        else:
            filepath = os.path.join('output_directory', 'Report.txt')
            
            if os.path.exists(filepath):
                os.remove(filepath)

            with open(filepath, 'a') as file:
                for blob in blob_response.blobs:
                    file.write(f'{blob.x:.2f} {blob.y:.2f} photo_object_{blob.id}.png trajectory_object_{blob.id}.png \n')
                    self.blobs.append(blob)
                rospy.loginfo("Report has been created!")
    
    def find_best_path(self, start: Point, goal: Point, blob_id):
        cell_start = self.get_cell_coordinate(start)
        cell_goal = self.get_cell_coordinate(goal)
        cell_goal = self.nearest_cell_to_obstacle(Point(cell_goal[0], cell_goal[1]))
        m_n = lambda node : self.m_neighbors_8(node, self.mapOfWorld)
        (seq, cost) = self.astar(cell_start, cell_goal, self.m_cost, m_n, self.m_h)
        self.draw_path(self.original_map, cell_start, cell_goal, seq[1:len(seq)-1], blob_id)

    def get_cell_coordinate(self, point: Point) -> Tuple[float, float]:
        dx_m = abs(point.x - self.origin_point.x)
        dy_m = abs(point.y - self.origin_point.y)
        dx_cell = floor(dx_m * self.map_height / self.map_heigth_m)
        dy_cell = floor(dy_m * self.map_width / self.map_width_m)
        return (dy_cell, dx_cell)

    def nearest_cell_to_obstacle(self, goal: Point) -> Tuple:
        #self.mapOfWorld[goal.x][goal.y] = 0
        white_cell_found = False
        distance = 1
        while(white_cell_found == False):
            if(self.mapOfWorld[goal.x + distance][goal.y] == 0):
                white_cell_found = True
                cell = (goal.x + distance, goal.y)
            if(self.mapOfWorld[goal.x - distance][goal.y] == 0):
                white_cell_found = True
                cell = (goal.x - distance, goal.y)
            if(self.mapOfWorld[goal.x][goal.y + distance] == 0):
                white_cell_found = True
                cell = (goal.x, goal.y + distance)
            if(self.mapOfWorld[goal.x][goal.y - distance] == 0):
                white_cell_found = True
                cell = (goal.x, goal.y - distance)
            if(self.mapOfWorld[goal.x + distance][goal.y + distance] == 0):
                white_cell_found = True
                cell = (goal.x + distance, goal.y + distance)
            if(self.mapOfWorld[goal.x - distance][goal.y - distance] == 0):
                white_cell_found = True
                cell = (goal.x - distance, goal.y - distance)
            if(self.mapOfWorld[goal.x + distance][goal.y - distance] == 0):
                white_cell_found = True
                cell = (goal.x + distance, goal.y - distance)
            if(self.mapOfWorld[goal.x - distance][goal.y + distance] == 0):
                white_cell_found = True
                cell = (goal.x - distance, goal.y + distance)
            distance = distance + 1
        # if goal.x < cell.x:
        #     increment_x = 1
        # else:
        #     increment_x = -1

        # if goal.x < cell.x:
        #     increment_y = 1
        # else:
        #     increment_y = -1
        
        # for x in range(goal.x,  cell.x, increment_x):
        #     self.mapOfWorld[x][goal.y] = 0
        #     for y in range(goal.y, cell.y, increment_y):
        #         self.mapOfWorld[x][y] = 0
            
        # for y in range(goal.y, cell.y, increment_y):
        #     self.mapOfWorld[goal.x][y] = 0
        #     for x in range(goal.x,  cell.x, increment_x):
        #         self.mapOfWorld[x][y] = 0

        return cell
    
    def brushfire(self, iterations):
        for _ in range(iterations):
            self.brushfire_one_pass()
        for coord in zip(*np.where(self.mapOfWorld==0)):
            self.mapOfWorld[coord[0]][coord[1]] = 0
        for coord in zip(*np.where(self.mapOfWorld==-5)):
            self.mapOfWorld[coord[0]][coord[1]] = 1
        return self.mapOfWorld
    
    def brushfire_one_pass(self):
        for coord in zip(*np.where(self.mapOfWorld==1)):
            neighbours = []
            neighbours.append([coord[0] - 1, coord[1]])
            neighbours.append([coord[0] + 1, coord[1]])
            neighbours.append([coord[0], coord[1] -1 ])
            neighbours.append([coord[0], coord[1] + 1])
            for neighbour in neighbours:
                if self.mapOfWorld[neighbour[0]][neighbour[1]] == 0:
                    self.mapOfWorld[neighbour[0]][neighbour[1]] = 1

    def get_map_client(self):
        prefix = "/racecar"
        rospy.wait_for_service(prefix + '/get_map')
        try:
            get_map = rospy.ServiceProxy(prefix + '/get_map', GetMap)
            response: GetMapResponse = get_map()
        except (rospy.ServiceException) as e:
            rospy.logwarn("Service call failed: %s"%e)
            return
        rospy.loginfo("Got map=%dx%d resolution=%f", response.map.info.height, response.map.info.width, response.map.info.resolution)    
        self.map_height = response.map.info.height
        self.map_width = response.map.info.width
        self.map_resolution = response.map.info.resolution
        self.origin_point.x = response.map.info.origin.position.x
        self.origin_point.y = response.map.info.origin.position.y
        self.map_heigth_m = (self.map_height*self.map_resolution)
        self.map_width_m = (self.map_width*self.map_resolution)
        return np.reshape(response.map.data, [response.map.info.height, response.map.info.width])
    
    def preface_map(self, occupancyGrid, value_of_unknowns=-5):
        map = np.zeros(occupancyGrid.shape, dtype=int)
        map[occupancyGrid==100] = 1 # obstacles
        map[occupancyGrid==-1] = value_of_unknowns  # unknowns
        return map
    
    def draw_path(self, obs_map, start, goal, seq, blob_id):
        self.draw_map(obs_map, start, goal)
        points = np.asarray(seq)
        plt.scatter(y=points[:,0]+0.4, x=points[:,1]+0.4, color="white", s=0.75)
        plt.gca().invert_xaxis()
        plt.legend(numpoints=1)
        plt.savefig(f"output_directory/trajectory_object_{blob_id}.png")
        plt.close()
        plt.cla()
        plt.clf()
        #plt.show()

    def draw_map(self, obs_map, start, goal):
        plt.figure()
        sns.heatmap(data=obs_map, annot=False)  
        # NOTE : Le syst??me de coordonn??es est invers?? pour le "scatter plot" :
        (s_y, s_x) = start
        (g_y, g_x) = goal
        plt.scatter(x=s_x+0.4, y=s_y+0.4, color="blue")
        plt.scatter(x=g_x+0.4, y=g_y+0.4, color="yellow")

    def m_cost(self, node_a, node_b):
        # Pour une carte, le co??t est toujours de 1.
        # On s'assure tout de m??me que les cellules sont adjacentes.
        dist_x = abs(node_a[0] - node_b[0])
        dist_y = abs(node_a[1] - node_b[1])
        assert dist_x <= 1 and dist_y <= 1, "m_cost : %s is not a neighbor of %s"%(str(node_a), str(node_b))
        if(dist_x == 1 and dist_y == 1):
            return sqrt(2)
        else:
            return 1

    def m_neighbors_8(self, node, obs_map):
        # G??n??re les voisins valides de node selon la carte (obs_map)
        # Connectivit?? 8.
        # Retourne une liste de tuples dont les cases sont vides (=0).
        ns = []
        x = node[0]
        y = node[1]
        lx = obs_map.shape[0] - 1
        ly = obs_map.shape[1] - 1
        
        min_x = -1 if (x > 0) else 0
        min_y = -1 if (y > 0) else 0
        max_x = 2 if (x < lx) else 1
        max_y = 2 if (y < ly) else 1
        
        for dx in range(min_x, max_x):
            for dy in range(min_y, max_y):
                if ((dx == 0) and (dy == 0)):
                    continue
                n = (x+dx, y+dy)
                if (obs_map[n] == 0):
                    ns.append(n)
                
        return ns

    def m_h(self, node_a, node_b):
        from math import sqrt
        (ax, ay) = node_a
        (bx, by) = node_b
        return sqrt((ax-bx)**2 + (ay-by)**2)

    def astar(self, start, goal, c_fun, n_fun, h_fun):
        # Cherche le chemin le plus court entre start et goal ?? l'aide de l'algorithme A*.
        # Retourne un tuple de la s??quence et du co??t : (seq, cost)
        # Utilise les fonctions :
        #   c_fun(edge): retourne le co??t de parcours du lien edge
        #   n_fun(node): retourne les voisins du noeud node
        #   h_fun(node_a, node_b): retourne la valeur de l'heuristique du co??t de parcours entre node_a et node_b
        
        search_set = [start] # Ensemble de recherche, ne contient que le noeud de d??part pour l'instant.
        
        # Dictionnaire contenant le plus bas co??t r??el du chemin (G) jusqu'au noeud sp??cifi??.
        g = {}
        # On initialise avec g = 0 pour le d??part
        g[start] = 0
        
        # Dictionnaire contenant la plus basse valeur de la fonction f(node) pour chaque noeud.
        f = {} 
        # On initialise f(start) avec g (qui est 0) et l'heuristique jusqu'?? la fin.
        f[start] = g[start] + h_fun(start, goal)
    
        # Dictionnaire qui conserve pour chaque noeud la provenance permettant la valeur f la plus faible.
        # Utilis?? pour reconstruire le chemin lorsque l'objectif est atteint.
        from_node = {}
        # On initialise avec le noeud de d??part sans provenance (None).
        from_node[start] = None
        
        while (len(search_set) > 0):
            # On trouve le noeud ayant la plus petite valeur f dans l'ensemble de recherche :
            min_f = inf
            min_n = None
            for n in search_set:
                assert n in f, "Error : %s not in F!"%(str(n))
                if f[n] < min_f:
                    min_f = f[n]
                    min_n = n
            
            # On retire le noeud ayant le plus petit 'f' et on poursuit avec lui :
            current = min_n
            #print("Current: %s, F(%s) = %d"%(current, current, min_f))
            search_set.remove(min_n)
            
            # Si le noeud en cours est l'objectif, c'est qu'aucun autre noeud dans l'espace de recherche n'a
            # un meilleur potentiel du chemin le plus court. On reconstruit le chemin ensuite. 
            if (current == goal):
                path_r = [goal]
                previous = from_node[goal]
                while (previous is not None):
                    path_r.append(previous)
                    previous = from_node[previous]             
                path_r.reverse() # On remet la liste dans le bon ordre
                return (path_r, g[goal])
            
            ns = n_fun(current) # Les voisins du noeud en cours
            for n in ns:
                # Pour chaque voisin, on calcule sa fonction f et on l'ajoute ?? l'ensemble de recherche seulement si
                # la valeur de g est plus basse que celle d??j?? connue pour ce noeud
                g_n = g[current] + c_fun(current, n)
                f_n = g_n + h_fun(n, goal)
                #print("G(%s) = %d, F(%s) = %d"%(n, g_n, n, f_n))
                if ((n not in g) or (g_n < g[n])):
                    from_node[n] = current
                    g[n] = g_n
                    f[n] = f_n
                    if (n not in search_set):
                        #print("Adding %s"%(str(n)))
                        search_set.append(n)
    
        # Nous avons vid?? l'espace de recherche sans trouver de solution. Retourner une liste vide et un co??t n??gatif.
        return ([], -1)

    
