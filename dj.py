import heapq
from random import choice
from random import uniform
import networkx as nx
import matplotlib.pyplot as plt
from time import time


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

    return list(reversed(path))


def bi_traceback_path(touch_node, parentsa, parentsb):
    path = traceback_path(touch_node, parentsa)
    touch_node = parentsb[touch_node]

    while touch_node:
        path.append(touch_node)
        touch_node = parentsb[touch_node]

    return path


def dijkstra(graph, source, target):
    open = [HeapEntry(source, 0.0)]
    closed = set()
    parents = dict()
    distance = dict()

    parents[source] = None
    distance[source] = 0.0

    while open:
        top_heap_entry = heapq.heappop(open)
        current = top_heap_entry.node

        if current == target:
            return traceback_path(target, parents)

        closed.add(current)

        for child in graph.get_children_of(current):
            if child in closed:
                continue

            tentative_cost = distance[current] + graph.get_arc_weight(current, child)

            if child not in distance.keys() or distance[child] > tentative_cost:
                distance[child] = tentative_cost
                parents[child] = current
                heap_entry = HeapEntry(child, tentative_cost)
                heapq.heappush(open, heap_entry)

    return []  # Target not reachable from source, return empty list.


def bidirectional_dijkstra(graph, source, target):
    opena = [HeapEntry(source, 0.0)]
    openb = [HeapEntry(target, 0.0)]
    closeda = set()
    closedb = set()
    parentsa = dict()
    parentsb = dict()
    distancea = dict()
    distanceb = dict()

    best_path_length = {'value': 1e9}
    touch_node = {'value': None}

    parentsa[source] = None
    parentsb[target] = None

    distancea[source] = 0.0
    distanceb[target] = 0.0

    def update_forward_frontier(node, node_score):
        if node in closedb:
            path_length = distanceb[node] + node_score

            if best_path_length['value'] > path_length:
                best_path_length['value'] = path_length
                touch_node['value'] = node

    def update_backward_frontier(node, node_score):
        if node in closeda:
            path_length = distancea[node] + node_score

            if best_path_length['value'] > path_length:
                best_path_length['value'] = path_length
                touch_node['value'] = node

    def expand_forward_frontier():
        current = heapq.heappop(opena).node
        closeda.add(current)

        for child in graph[current]:
            if child in closeda:
                continue

            tentative_score = distancea[current] + graph[current][child]['weight']

            if child not in distancea.keys() or tentative_score < distancea[child]:
                distancea[child] = tentative_score
                parentsa[child] = current
                heapq.heappush(opena, HeapEntry(child, tentative_score))
                update_forward_frontier(child, tentative_score)

    def expand_backward_frontier():
        current = heapq.heappop(openb).node
        closedb.add(current)

        for parent in graph[current]:
            if parent in closedb:
                continue

            tentative_score = distanceb[current] + graph[parent][current]['weight']

            if parent not in distanceb.keys() or tentative_score < distanceb[parent]:
                distanceb[parent] = tentative_score
                parentsb[parent] = current
                heapq.heappush(openb, HeapEntry(parent, tentative_score))
                update_backward_frontier(parent, tentative_score)

    while opena and openb:
        tmp = distancea[opena[0].node] + distanceb[openb[0].node]

        if tmp >= best_path_length['value']:
            return bi_traceback_path(touch_node['value'], parentsa, parentsb)

        if len(opena) + len(closeda) < len(openb) + len(closedb):
            expand_forward_frontier()
        else:
            expand_backward_frontier()

    return []


def create_random_digraph(nodes, arcs, max_weight):
    graph = create_graph()
    node_list = []

    for node in range(nodes):
        graph.add_node(node)
        node_list.append(node)

    for _ in range(arcs):
        weight = uniform(0.0, max_weight)
        graph.add_arc(choice(node_list),
                      choice(node_list),
                      weight)

    return graph, node_list


def path_cost(graph, path):
    cost = 0.0

    for i in range(len(path) - 1):
        tail = path[i]
        head = path[i + 1]

        if not graph[tail][head]:
            raise Exception("Not a path.")

        cost =cost+ graph[tail][head]['weight']

    return cost

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

def main():
    #graph, node_list = create_random_digraph(1000000, 5000000, 10.0)
    #del node_list[:]
    graph = create_graph()
    G_to_dict = nx.to_dict_of_dicts(graph)
    path = bidirectional_dijkstra(G_to_dict, 'a', 'z')
    cost = path_cost(G_to_dict,path)
    print(path,cost)


if __name__ == "__main__":
    main()