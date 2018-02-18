########################################################################################################################
#                                                                                                                      #
#                                   UCD Michael Smurfit Graduate Business School                                       #
#                                        MIS40550: Network Software Modelling                                          #
#                                                    Assignment - 1                                                    #
#                                               Due Date: 12 March 2017                                                #
#                                      Submitted by: Deepak Kumar Gupta (16200660)                                     #
#                                                                                                                      #
########################################################################################################################



import heapq as hq
import networkx as nx
import matplotlib.pyplot as plt
import math
import time
import random as random
########################################################################################################################
#                                                                                                                      #
#                   --------------------------------------------------------------------------------                   #
#                                               Class: Queue                                                           #
#                   --------------------------------------------------------------------------------                   #
#          Description: Creating node for heap queue that will be used for running efficient Dijkstra Algorithm        #
#                                                                                                                      #
########################################################################################################################

class Queue:
    def __init__(self, v, p):  #V is node and p is Priority in a heap tree
        self.v = v
        self.p = p

    def __lt__(self, other):
        return self.p < other.p

########################################################################################################################
#                                                                                                                      #
#                   --------------------------------------------------------------------------------                   #
#                                               Function : dijkstra(G,S,T)                                             #
#                   --------------------------------------------------------------------------------                   #
# Parameters:                                                                                                          #
#   G: Graph                                                                                                           #
#   S: Source node from where shortest path to be find                                                                 #
#   T: Target node till where shortest path to be find                                                                 #
# Description:                                                                                                         #
#   - Heapq data structures has been used to implement Dijkstra algorithm                                              #
#   - <object>.heappop() and <object>.heappush() methods used to pop and push vertices from a graph                    #
# Function return shortest travel path using heapq data structures                                                     #
#                                                                                                                      #
########################################################################################################################

def dijkstra(G, S, T):
    start = [Queue(S, 0.0)]                       # Creating initial start node using HeapQ and setting its value to 0.0
    goal = set()
    pred = dict()                                                         # Dictionary to store visited nodes in a graph
    dist = dict()                                                     # Dictionary to store distance from point to point
    pred[S] = None
    dist[S] = 0.0
    while start:
        C = hq.heappop(start).v                   # Pop the smallest item off the heap, maintaining the heap invariant.
        if C == T:
            return traversal(T, pred)
        goal.add(C)
        for pointer in G[C]:
            if pointer in goal:
                continue
            dist_temp = dist[C] + G[C][pointer]['weight']
            if pointer not in dist or dist[pointer] > dist_temp:                          #Checking vertex with low cost
                dist[pointer] = dist_temp
                pred[pointer] = C
                hq.heappush(start, Queue(pointer, dist[C] + G[C][pointer]['weight']))          # Adding vertex to queue
    return []


########################################################################################################################
#                   --------------------------------------------------------------------------------                   #
#                                           Function : bidirectional_dijkstra(G,S,T)                                   #
#                   --------------------------------------------------------------------------------                   #
# Parameters:                                                                                                          #
#   G: Graph                                                                                                           #
#   S: Source node from where shortest path to be find                                                                 #
#   T: Target node till where shortest path to be find                                                                 #
# Description:                                                                                                         #
#   - Heapq data structures has been used to implement Dijkstra algorithm                                              #
#   - <object>.heappop() and <object>.heappush() methods used to pop and push vertices from a graph                    #
# Stopping Criteria: 1. dist_S[startS[0].v] + dist_T[startT[0].v] >= v_dist['weight']                                  #
#                    2. len(startS) + len(goal_S) < len(startT) + len(goal_T)                                          #
#                    3. when a node is scanned in both directions                                                      #
#                                                                                                                      #
########################################################################################################################


def bidirectional_dijkstra(G, S, T):

    startS = [Queue(S, 0.0)]   # Creating initial start node for forward search using HeapQ and setting its value to 0.0
    startT = [Queue(T, 0.0)]   # Creating initial start node for forward search using HeapQ and setting its value to 0.0

    goal_S = set()
    goal_T = set()

    pre_S = dict()
    pre_T = dict()
    dist_S = dict()                                                 # Dictionary to store distance from source to target
    dist_T = dict()                                                 # Dictionary to store distance from target to source

    v_dist = {'weight': math.inf}                                         # Setting other vertex initial distance to inf
    node = {'weight': None}


    pre_S[S] = None
    pre_T[T] = None
    dist_S[S] = 0.0
    dist_T[T] = 0.0
    def update(v, weight,goal):
        if v in goal:
            distance = dist_T[v] + weight
            if v_dist['weight'] > distance:
                v_dist['weight'] = distance
                node['weight'] = v

    while startS and startT:
        if dist_S[startS[0].v] + dist_T[startT[0].v] >= v_dist['weight']:
            return reverse_traversal(node['weight'], pre_S, pre_T)

        if len(startS) + len(goal_S) < len(startT) + len(goal_T):
            C = hq.heappop(startS).v                #Pop the smallest item off the heap, maintaining the heap invariant.
            goal_S.add(C)                                                                             #C is current node
            for fwd in G[C]:
                if fwd in goal_S:
                    continue
                cur_dist = dist_S[C] + G[C][fwd]['weight']
                if fwd not in dist_S or cur_dist < dist_S[fwd]:
                    dist_S[fwd] = cur_dist
                    pre_S[fwd] = C
                    hq.heappush(startS, Queue(fwd, cur_dist))
                    update(fwd, cur_dist, goal_T)
        else:
            C = hq.heappop(startT).v                # Pop the smallest item off the heap, maintaining the heap invariant
            goal_T.add(C)
            for back in G[C]:
                if back in goal_T:
                    continue
                cur_dist = dist_T[C] + G[back][C]['weight']
                if back not in dist_T or cur_dist < dist_T[back]:
                    dist_T[back] = cur_dist
                    pre_T[back] = C
                    hq.heappush(startT, Queue(back, cur_dist))
                    update(back, cur_dist, goal_S)

    return []


########################################################################################################################
#                                                                                                                      #
# Function : traversal(T,pred)                                                                                         #
# Description: Accept two argument i.e A Node and Pred (Predecessor) list of visited node in a graph                   #
#              Function returns path of forward traversal                                                              #
#                                                                                                                      #
########################################################################################################################

def traversal(T,pred):
    path = []
    while T:
        path.append(T)
        T = pred[T]
    return path[::-1]


########################################################################################################################
#                                                                                                                      #
#                   --------------------------------------------------------------------------------                   #
#                                               Function : traversal(v,pre_S,pre_T)                                    #
#                   --------------------------------------------------------------------------------                   #
# Description: Accept three argument i.e A Node and two Pred (Predecessor) list of visited node in a graph             #
#              Function returns path of traversal for bi-directional dijkstra algorithm, as it combine path traversal  #
#              of forward and backward traversal                                                                       #
#                                                                                                                      #
########################################################################################################################

def reverse_traversal(v, pre_S, pre_T):
    path = traversal(v, pre_S)
    v = pre_T[v]
    while v:
        path.append(v)
        v = pre_T[v]
    return path

########################################################################################################################
#                                                                                                                      #
#                   --------------------------------------------------------------------------------                   #
#                                               Function : distance(G,path)                                            #
#                   --------------------------------------------------------------------------------                   #
# Description: Function take a graph and path as argument and return total distance for that particular path in a graph#
#                                                                                                                      #
########################################################################################################################

def distance(G, path):
    dist = 0.0
    tot_v = len(path) -1  #Total Number of Vertex minus 1
    for i in range(tot_v):
        dist += G[path[i]][path[i + 1]]['weight']
    return dist


########################################################################################################################
#                                                                                                                      #
#                   --------------------------------------------------------------------------------                   #
#                                          Function: generate_random_graph(n,e)                                        #
#                   --------------------------------------------------------------------------------                   #
# Parameters:                                                                                                          #
#       n: no. of nodes                                                                                                #
#       n: no. of nodes                                                                                                #
# Methods Used:                                                                                                        #
#   dense_gnm_random_graph(n,e) for generating random graphs                                                           #
#   circular_layout(G): arranging node in circular way                                                                 #
#   get_edge_attributes(G, 'weight'): getting weights of each edge                                                     #
#   draw_networkx_edge_labels(): Plotting weights of each edge on graph                                                #
#                                                                                                                      #
########################################################################################################################

def generate_random_graph(n,e):
    G = nx.dense_gnm_random_graph(n, e)
    for (u, v) in G.edges():
        G.edge[u][v]['weight'] = random.randint(1, 10)
    pos = nx.circular_layout(G)
    nx.draw_networkx(G, pos, node_size=700)
    labels = nx.get_edge_attributes(G, 'weight')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)
    plt.savefig('graph.png')
    plt.show(G)
    return G


########################################################################################################################
#                                                                                                                      #
#                   --------------------------------------------------------------------------------                   #
#                                               Main Block of Program                                                  #
#                   --------------------------------------------------------------------------------                   #
#                                                                                                                      #
# Initial Inputs:                                                                                                      #
#       - S: Source node from where traversal to begin                                                                 #
#       - T: Target node till where shortest path to be find                                                           #
#       - n: No. of nodes, an input for generating random graphs                                                       #
#       - e: No. of edges, an input for generating random graphs                                                       #
#                                                                                                                      #
# Methods Called:                                                                                                      #
#       - generate_random_graph(n,e): generates the random graph of n node and e edges, which is then converted in dict#
#         format using nx.to_dict_of_dicts(G)                                                                          #
#       - time.perf_counter(): is used for calculating runtime of algorithm                                            #
#       - dijkstra(G,S,T): calling single directional dijkstra algorithm to find shortest path                         #
#       - bidirectional_dijkstra(G,S,T): calling bidirectional dijkstra algorithm to find shortest path                #
#                                                                                                                      #
# Output Variable:                                                                                                     #
#       - path: return shortest path for single directional dijkstra                                                   #
#       - bi_path: return shortest path for bidirectional dijkstra algorithm                                           #
#       - dist: return shortest path distance for dijkstra algorithm                                                   #
#       - bi_dist: return shortest path distance for bidirectional dijkstra algorithm                                  #
#                                                                                                                      #
########################################################################################################################


if __name__ == "__main__":
    S = 0  # Source Vertex
    T = 5  # Target Vertex
    n = 8 # No. of nodes
    e = 10 # No. of Edges
    G_to_dict = nx.to_dict_of_dicts(generate_random_graph(n,e))


    t1 = time.perf_counter()
    path = dijkstra(G_to_dict, S, T)
    t2 =  time.perf_counter()
    print("Bi-Dijkstra runtime is", 1000*(t2 - t1), "milliseconds.")

    t3 = time.perf_counter()
    bi_path = bidirectional_dijkstra(G_to_dict, S, T)
    t4= time.perf_counter()
    print("Dijkstra runtime is", 1000*(t4 - t3), "milliseconds.")

    dist = distance(G_to_dict, path)
    bi_dist = distance(G_to_dict, bi_path)
    print("Bi Directional Dijkstra ", bi_path, bi_dist)
    print("Dijkstra", path, dist)

