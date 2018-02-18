import heapq
from random import choice
from random import uniform
from time import time

__author__ = 'Rodion "rodde" Efremov'


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

        for child in graph.get_children_of(current):
            if child in closeda:
                continue

            tentative_score = distancea[current] + graph.get_arc_weight(current, child)

            if child not in distancea.keys() or tentative_score < distancea[child]:
                distancea[child] = tentative_score
                parentsa[child] = current
                heapq.heappush(opena, HeapEntry(child, tentative_score))
                update_forward_frontier(child, tentative_score)

    def expand_backward_frontier():
        current = heapq.heappop(openb).node
        closedb.add(current)

        for parent in graph.get_parents_of(current):
            if parent in closedb:
                continue

            tentative_score = distanceb[current] + graph.get_arc_weight(parent, current)

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



def path_cost(graph, path):
    cost = 0.0

    for i in range(len(path) - 1):
        tail = path[i]
        head = path[i + 1]

        if not graph.has_arc(tail, head):
            raise Exception("Not a path.")

        cost += graph.get_arc_weight(tail, head)

    return cost


def main():
    graph, node_list = create_random_digraph(1000000, 5000000, 10.0)
    source = choice(node_list)
    target = choice(node_list)
    del node_list[:]

    print("Source:", source)
    print("Target:", target)

    start_time = time()
    path1 = dijkstra(graph, source, target)
    end_time = time()

    print("Dijkstra's algorithm in", 1000.0 * (end_time - start_time), "milliseconds.")

    start_time = time()
    path2 = bidirectional_dijkstra(graph, source, target)
    end_time = time()

    print("Bidirectional Dijkstra's algorithm in", 1000.0 * (end_time - start_time), "milliseconds.")

    print("Paths are identical:", path1 == path2)

    print("Dijkstra path:")

    for node in path1:
        print(node)

    print("Path length:", path_cost(graph, path1))

    print("Bidirectional path:")

    for node in path2:
        print(node)

    print("Path length:", path_cost(graph, path2))

if __name__ == "__main__":
    main()
