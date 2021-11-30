#!/usr/bin/env python

from typing import List, Dict
import rospy
from rtabmap_ros.srv import GetMap, GetMapResponse
from math import sqrt, inf
import networkx as nx
import matplotlib.pyplot as plt

g_nodes: List
g_edges: List
g_dists: List
g_pos: List
g_costs: Dict


def main():
    rospy.init_node('brushfire')
    prefix = "racecar"
    rospy.wait_for_service(prefix + '/get_map_data')
    try:
        get_map_data = rospy.ServiceProxy(prefix + '/get_map_data', GetMap)
        response: GetMapResponse = get_map_data()
        global g_nodes 
        global g_edges
        global g_dists
        global g_pos
        global g_costs
        g_nodes = response.data.graph.posesId
        g_edges = [(unfiltered_edge.fromId, unfiltered_edge.toId) for unfiltered_edge in response.data.graph.links]
        g_dists = [sqrt((unfiltered_edge.transform.translation.x)**2+(unfiltered_edge.transform.translation.y)**2) for unfiltered_edge in response.data.graph.links]
        interim_pos = [(poses.position.x, poses.position.y) for poses in response.data.graph.poses]
        g_pos = {g_nodes[i]:interim_pos[i] for i in range(len(g_nodes))}
        g_costs = dict(zip(g_edges, g_dists)) # Création d'un dictionnaire associant les liens à des distances.
        draw_graph(g_nodes, g_edges, g_dists, g_pos)
        (seq, cost) = astar(1, 28, g_cost, g_neighbors, g_h)
        print("Le plus court chemin MTL - SHE est : %s (%d)."%(seq, cost))
    except (rospy.ServiceException) as e:
        print("Service call failed: %s"%e)
        return

def draw_graph(nodes, edges, dists, pos):
    graph = nx.Graph()
    graph.add_nodes_from(nodes)
    graph.add_edges_from(edges)
    
    nx.draw_networkx(graph, pos, node_size=1000, node_color="#EEEEEE")
    nx.draw_networkx_edge_labels(graph, pos, dict(zip(edges, dists)))

    # Nécessaire pour agrandir la zone de dessin :
    l,r = plt.xlim()
    t,b = plt.ylim()
    plt.xlim(l-20, r+20)
    plt.ylim(t-10, b+10)
    
    plt.axis('off')
    plt.show()
    
def g_cost(node_a, node_b):
    edge   = (node_a, node_b)
    # Il faut aussi vérifier le lien inverse vu que nous sommes bidirectionnels :
    edge_r = (node_b, node_a)
    
    if (edge in g_costs):
        return g_costs[edge]
    elif (edge_r in g_costs):
        return g_costs[edge_r]
    else:
        # ERROR!
        return -1

def g_neighbors(node):
    assert node in g_nodes, "g_neighbors: node unknown"
    
    neighbors = []
    for (a,b) in g_edges:
        if (a == node):
            neighbors.append(b)
        elif (b == node):
            neighbors.append(a)
    return neighbors

def g_h(node_a, node_b):
    assert node_a in g_pos, "g_h: %s not in g_pos"%(node_a)
    assert node_b in g_pos, "g_h: %s not in g_pos"%(node_b) 
    
    pos_a = g_pos[node_a]
    pos_b = g_pos[node_b]
    
    return sqrt((pos_a[0]-pos_b[0])**2 + (pos_a[1] - pos_b[1])**2)

def astar(start, goal, c_fun, n_fun, h_fun):
    # Cherche le chemin le plus court entre start et goal à l'aide de l'algorithme A*.
    # Retourne un tuple de la séquence et du coût : (seq, cost)
    # Utilise les fonctions :
    #   c_fun(edge): retourne le coût de parcours du lien edge
    #   n_fun(node): retourne les voisins du noeud node
    #   h_fun(node_a, node_b): retourne la valeur de l'heuristique du coût de parcours entre node_a et node_b
    
    search_set = [start] # Ensemble de recherche, ne contient que le noeud de départ pour l'instant.
    
    # Dictionnaire contenant le plus bas coût réel du chemin (G) jusqu'au noeud spécifié.
    g = {}
    # On initialise avec g = 0 pour le départ
    g[start] = 0
    
    # Dictionnaire contenant la plus basse valeur de la fonction f(node) pour chaque noeud.
    f = {} 
    # On initialise f(start) avec g (qui est 0) et l'heuristique jusqu'à la fin.
    f[start] = g[start] + h_fun(start, goal)
   
    # Dictionnaire qui conserve pour chaque noeud la provenance permettant la valeur f la plus faible.
    # Utilisé pour reconstruire le chemin lorsque l'objectif est atteint.
    from_node = {}
    # On initialise avec le noeud de départ sans provenance (None).
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
        print("Current: %s, F(%s) = %d"%(current, current, min_f))
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
            # Pour chaque voisin, on calcule sa fonction f et on l'ajoute à l'ensemble de recherche seulement si
            # la valeur de g est plus basse que celle déjà connue pour ce noeud
            g_n = g[current] + c_fun(current, n)
            f_n = g_n + h_fun(n, goal)
            print("G(%s) = %d, F(%s) = %d"%(n, g_n, n, f_n))
            if ((n not in g) or (g_n < g[n])):
                from_node[n] = current
                g[n] = g_n
                f[n] = f_n
                if (n not in search_set):
                    print("Adding %s"%(str(n)))
                    search_set.append(n)
   
    # Nous avons vidé l'espace de recherche sans trouver de solution. Retourner une liste vide et un coût négatif.
    return ([], -1)

if __name__ == '__main__':
    main()
    
