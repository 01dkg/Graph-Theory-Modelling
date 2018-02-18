from heapq import heappush, heappop
from itertools import count
import networkx as nx


def bidirectional_dijkstra(G, source, target, weight='weight'):
    if source == target:
        return (0, [source])
    push = heappush
    pop = heappop
    # Init:   Forward             Backward
    dists  = [{},                {}]  # dictionary of final distances
    paths  = [{source: [source]}, {target: [target]}]  # dictionary of paths
    fringe = [[],                []]  # heap of (distance, node) tuples for
                                      # extracting next node to expand
    seen   = [{source: 0},        {target: 0}]  # dictionary of distances to
                                                # nodes seen
    c = count()
    # initialize fringe heap
    push(fringe[0], (0, next(c), source))
    push(fringe[1], (0, next(c), target))
    # neighs for extracting correct neighbor information
    if G.is_directed():
        neighs = [G.successors_iter, G.predecessors_iter]
    else:
        neighs = [G.neighbors_iter, G.neighbors_iter]
    # variables to hold shortest discovered path
    #finaldist = 1e30000
    finalpath = []
    dir = 1
    while fringe[0] and fringe[1]:
        # choose direction
        # dir == 0 is forward direction and dir == 1 is back
        dir = 1 - dir
        # extract closest to expand
        (dist, _, v) = pop(fringe[dir])
        if v in dists[dir]:
            # Shortest path to v has already been found
            continue
        # update distance
        dists[dir][v] = dist  # equal to seen[dir][v]
        if v in dists[1 - dir]:
            # if we have scanned v in both directions we are done
            # we have now discovered the shortest path
            return (finaldist, finalpath)

        for w in neighs[dir](v):
            if(dir == 0):  # forward
                if G.is_multigraph():
                    minweight = min((dd.get(weight, 1)
                                     for k, dd in G[v][w].items()))
                else:
                    minweight = G[v][w].get(weight, 1)
                vwLength = dists[dir][v] + minweight  # G[v][w].get(weight,1)
            else:  # back, must remember to change v,w->w,v
                if G.is_multigraph():
                    minweight = min((dd.get(weight, 1)
                                     for k, dd in G[w][v].items()))
                else:
                    minweight = G[w][v].get(weight, 1)
                vwLength = dists[dir][v] + minweight  # G[w][v].get(weight,1)

            if w in dists[dir]:
                if vwLength < dists[dir][w]:
                    raise ValueError(
                        "Contradictory paths found: negative weights?")
            elif w not in seen[dir] or vwLength < seen[dir][w]:
                # relaxing
                seen[dir][w] = vwLength
                push(fringe[dir], (vwLength, next(c), w))
                paths[dir][w] = paths[dir][v] + [w]
                if w in seen[0] and w in seen[1]:
                    # see if this path is better than than the already
                    # discovered shortest path
                    totaldist = seen[0][w] + seen[1][w]
                    if finalpath == [] or finaldist > totaldist:
                        finaldist = totaldist
                        revpath = paths[1][w][:]
                        revpath.reverse()
                        finalpath = paths[0][w] + revpath[1:]
    raise nx.NetworkXNoPath("No path between %s and %s." % (source, target))



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
    return G

G = create_graph()
(length, path) = bidirectional_dijkstra(G, 'a', target='z')
print(path,length)