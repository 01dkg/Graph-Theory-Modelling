import heapq
import networkx as nx
import matplotlib.pyplot as plt
from time import time


def create_graph():
    G = nx.Graph()
    G.add_edge('a', 'b', weight=4)                                                              #Adding Edges and weight
    G.add_edge('a', 'c', weight=2)
    G.add_edge('b', 'c', weight=1)
    G.add_edge('b', 'd', weight=5)
    G.add_edge('c', 'd', weight=8)
    G.add_edge('c', 'e', weight=10)
    G.add_edge('e', 'd', weight=2)
    G.add_edge('d', 'z', weight=6)
    G.add_edge('e', 'z', weight=3)
    pos = nx.spring_layout(G)                                                                  # positions for all nodes
    #nx.draw_networkx(G,pos,node_size=700)
    labels = nx.get_edge_attributes(G,'weight')
    #nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)
    plt.show()                                                                                          # Display graph
    return G


def dijkstra(G_to_dict, source, target):
    dist_cost = {}
    prev_node = {}
    vertex_to_traverse = G_to_dict.keys()                                            #Adding all vertex to be traversed
    for vertex in G_to_dict:
        dist_cost[vertex] = float('inf')                          #Setting distance cost to infinity for first iteration
        prev_node[vertex] = None

    vertex_already_traversed = []                                                 #List to save already traversed vertex
    dist_cost[source] = 0                                                         #Distance cost from Source Vertex is 0
    while len(vertex_already_traversed) < len(vertex_to_traverse):        #Checking any vertex left for traversal or not

        still_in = {vertex: dist_cost[vertex] \
                    for vertex in [vertex for vertex in vertex_to_traverse if vertex not in vertex_already_traversed]}

        nearest_vertex = min(still_in, key=dist_cost.get)
        vertex_already_traversed.append(nearest_vertex)
        for vertex in G_to_dict[nearest_vertex]:
            if dist_cost[vertex] > dist_cost[nearest_vertex] + G_to_dict[nearest_vertex][vertex]['weight']:
                dist_cost[vertex] = dist_cost[nearest_vertex] + G_to_dict[nearest_vertex][vertex]['weight']
                prev_node[vertex] = nearest_vertex
    shortest_path = [target]
    while source not in shortest_path:
        shortest_path.append(prev_node[shortest_path[-1]])
    # return the shortest_path in order source -> target, and it's cost
    return shortest_path[::-1], dist_cost[target]

class HeapQueue:
    def __init__(self, vertex, priority_vertex):
        self.vertex = vertex
        self.priority_vertex = priority_vertex

    def __lt__(self, other):
        return self.priority_vertex < other.priority_vertex


def reverse_path_traversal(target_node, parent_list):
    path_list = []
    while target_node:
        path_list.append(target_node)
        target_node = parent_list[target_node]
    return path_list[::-1]                                                                     #Reverse Path is returned


def bi_reverse_path_traversal(current_vertex, parent_source, parent_target):
    path_list = reverse_path_traversal(current_vertex, parent_source)

    current_vertex = parent_target[current_vertex]
    while current_vertex:
        path_list.append(current_vertex)
        current_vertex = parent_target[current_vertex]
    return path_list

def find_shortest_path_distance(graph, shortest_path):
    shortest_distance = 0.0
    for i in range(len(shortest_path) - 1):
        tail = shortest_path[i]
        head = shortest_path[i + 1]
        if not graph[tail][head]:
            raise Exception("Not a path.")
    shortest_distance =shortest_distance+ graph[tail][head]['weight']
    return shortest_distance

def dijkstra_bidirectional(graph, S, T):
    #S is Source T is Target
    Initial_S = [HeapQueue(S, 0.0)]
    Initial_T = [HeapQueue(T, 0.0)]
    Goal_S = set()
    Goal_T= set()
    parent_S = {}
    parent_T = {}
    dist_S = {}
    dist_T = {}
    min_path_distance = {'value': 1e9}
    current_vertex = {'value': None}
    parent_S[S] = None
    parent_T[T] = None
    dist_S[S] = 0.0
    dist_T[T] = 0.0

    def update_queue(node, node_score,end):
        if node in end:
            path_length = dist_S[node] + node_score
            if min_path_distance['value'] > path_length:
                min_path_distance['value'] = path_length
                current_vertex['value'] = node

    def expandF():
        pointer = heapq.heappop(Initial_S).vertex
        Goal_S.add(pointer)
        for pointer_node in graph[pointer]:
            if pointer_node in Goal_S:
                continue
            temp_cost = dist_S[pointer] + graph[pointer][pointer_node]['weight']
            if pointer_node not in dist_S.keys() or temp_cost < dist_S[pointer_node]:
                dist_S[pointer_node] = temp_cost
                parent_S[pointer_node] = pointer
                heapq.heappush(Initial_S, HeapQueue(pointer_node, temp_cost))
                update_queue(pointer_node, temp_cost, Goal_T)

    def expandB():
        pointer = heapq.heappop(Initial_T).vertex
        Goal_T.add(pointer)
        for pointer_node in graph[pointer]:
            if pointer_node in Goal_T:
                continue
            temp_cost = dist_T[pointer] + graph[pointer_node][pointer]['weight']
            if pointer_node not in dist_T.keys() or temp_cost < dist_T[pointer_node]:
                dist_T[pointer_node] = temp_cost
                parent_T[pointer_node] = pointer
                heapq.heappush(Initial_T, HeapQueue(pointer_node, temp_cost))
                update_queue(pointer_node, temp_cost, Goal_S)

    def explore(Initial,Goal,):
        pointer = heapq.heappop(Initial_T).vertex
        Goal_T.add(pointer)
        for pointer_node in graph[pointer]:
            if pointer_node in Goal_T:
                continue
            temp_cost = dist_T[pointer] + graph[pointer_node][pointer]['weight']
            if pointer_node not in dist_T.keys() or temp_cost < dist_T[pointer_node]:
                dist_T[pointer_node] = temp_cost
                parent_T[pointer_node] = pointer
                heapq.heappush(Initial_T, HeapQueue(pointer_node, temp_cost))
                update_queue(pointer_node, temp_cost, Goal_S)

    while Initial_S and Initial_T:                       #Stopping Condition for bi-directional dijkstra algorithm
        if dist_S[Initial_S[0].vertex] + dist_T[Initial_T[0].vertex] >= min_path_distance['value']:
            return bi_reverse_path_traversal(current_vertex['value'], parent_S, parent_T)
        if len(Initial_S) + len(Goal_S) < len(Initial_T) + len(Goal_T):
            expandF()
        else:
            expandB()
    return []


def main():
    G_to_dict = nx.to_dict_of_dicts(create_graph())
    shortest_path, distance = dijkstra(G_to_dict, 'a', 'z')
    print("Shortest Path is:",shortest_path,distance)
    shortest_path = dijkstra_bidirectional(G_to_dict, 'a', 'z')
    shortest_path_distance =   find_shortest_path_distance(G_to_dict, shortest_path)
    print("Bi Directional",shortest_path,shortest_path_distance)
main()