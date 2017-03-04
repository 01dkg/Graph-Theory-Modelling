
import heapq
import networkx as nx
import matplotlib.pyplot as plt
from time import time


def create_graph():
    G = nx.Graph()
    #Adding Edges and weight
    G.add_edge('a', 'b', weight=4)
    G.add_edge('a', 'c', weight=2)
    G.add_edge('b', 'c', weight=1)
    G.add_edge('b', 'd', weight=5)
    G.add_edge('c', 'd', weight=8)
    G.add_edge('c', 'e', weight=10)
    G.add_edge('e', 'd', weight=2)
    G.add_edge('d', 'z', weight=6)
    G.add_edge('e', 'z', weight=3)
    pos = nx.spring_layout(G)  # positions for all nodes
    nx.draw_networkx(G,pos,node_size=700)
    labels = nx.get_edge_attributes(G,'weight')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)
    plt.show()  # display
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

class HeapEntry:
    def __init__(self, node, priority):
        self.node = node
        self.priority = priority

    def __lt__(self, other):
        return self.priority < other.priority


def traceback_path(target, parents):
    path = []

    while target:
        path.append(target)
        target = parents[target]

    return path[::-1]   #Reverse Path is returned


def bi_traceback_path(touch_node, parent_source, parent_target):
    path = traceback_path(touch_node, parent_source)
    touch_node = parent_target[touch_node]

    while touch_node:
        path.append(touch_node)
        touch_node = parent_target[touch_node]

    return path

def path_cost(graph, path):
    cost = 0.0
    for i in range(len(path) - 1):
        tail = path[i]
        head = path[i + 1]
        if not graph[tail][head]:
            raise Exception("Not a path.")

        cost =cost+ graph[tail][head]['weight']

    return cost

def bi_dijkstra(graph, source, target):
    start_source = [HeapEntry(source, 0.0)]
    start_target = [HeapEntry(target, 0.0)]
    end_source = set()
    end_target = set()
    parent_source = dict()
    parent_target = dict()
    dist_source = dict()
    dist_target = dict()
    best_path_length = {'value': 1e9}
    touch_node = {'value': None}
    parent_source[source] = None
    parent_target[target] = None
    dist_source[source] = 0.0
    dist_target[target] = 0.0

    def update_forward_frontier(node, node_score):
        if node in end_target:
            path_length = dist_target[node] + node_score
            if best_path_length['value'] > path_length:
                best_path_length['value'] = path_length
                touch_node['value'] = node

    def update_backward_frontier(node, node_score):
        if node in end_source:
            path_length = dist_source[node] + node_score

            if best_path_length['value'] > path_length:
                best_path_length['value'] = path_length
                touch_node['value'] = node

    def expand_forward_frontier():
        current = heapq.heappop(start_source).node
        end_source.add(current)

        for child_node in graph[current]:
            if child_node in end_source:
                continue

            tentative_score = dist_source[current] + graph[current][child_node]['weight']

            if child_node not in dist_source.keys() or tentative_score < dist_source[child_node]:
                dist_source[child_node] = tentative_score
                parent_source[child_node] = current
                heapq.heappush(start_source, HeapEntry(child_node, tentative_score))
                update_forward_frontier(child_node, tentative_score)

    def expand_backward_frontier():
        current = heapq.heappop(start_target).node
        end_target.add(current)

        for parent in graph[current]:
            if parent in end_target:
                continue

            tentative_score = dist_target[current] + graph[parent][current]['weight']

            if parent not in dist_target.keys() or tentative_score < dist_target[parent]:
                dist_target[parent] = tentative_score
                parent_target[parent] = current
                heapq.heappush(start_target, HeapEntry(parent, tentative_score))
                update_backward_frontier(parent, tentative_score)

    while start_source and start_target:
        tmp = dist_source[start_source[0].node] + dist_target[start_target[0].node]

        if tmp >= best_path_length['value']:
            return bi_traceback_path(touch_node['value'], parent_source, parent_target)

        if len(start_source) + len(end_source) < len(start_target) + len(end_target):
            expand_forward_frontier()
        else:
            expand_backward_frontier()

    return []


def main():
    start_time = time()
    graph= create_graph()
    G_to_dict = nx.to_dict_of_dicts(graph)
    shortest_path, distance = dijkstra(G_to_dict, 'a', 'z')
    end_time = time()
    print("Bidirectional Dijkstra's algorithm in", 1000.0 * (end_time - start_time), "milliseconds.")
    print("Shortest Path is:",shortest_path)
    print("Shortest Distance is",distance )
    path = bi_dijkstra(G_to_dict, 'a', 'z')
    cost =   path_cost(G_to_dict, path)
    print("Bi Directional",path,cost)


if __name__ == '__main__':
    main()