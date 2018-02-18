import heapq
import networkx as nx
import matplotlib.pyplot as plt
from time import time


G = nx.Graph()
G.add_edge(0, 1, weight=4)                                                              #Adding Edges and weight
G.add_edge(0, 2, weight=2)
G.add_edge(1, 2, weight=1)
G.add_edge(1, 3, weight=5)
G.add_edge(2, 3, weight=8)
G.add_edge(2, 4, weight=1)
G.add_edge(4, 3, weight=2)
G.add_edge(3, 5, weight=6)
G.add_edge(4, 5, weight=3)
G.add_edge(5, 6, weight=6)
G.add_edge(6, 1, weight=7)# positions for all nodes
pos=nx.fruchterman_reingold_layout(G) # positions for all nodes

dij = [(0,2),(2,1),(1,3),(3,4),(4,5),(5,6)]
non = [(0,1),(2,4),(2,3),(3,5),(6,1),(5,6)]
fwd =[(0,2),(2,1),(1,3),(3,4)]
back=[(3,4),(4,5)]
# nodes
nx.draw_networkx_nodes(G,pos,node_size=700)
# edges
nx.draw_networkx_edges(G,pos,edgelist=fwd,
                    width=6)
nx.draw_networkx_edges(G,pos,edgelist=back, width=6,alpha=0.5,edge_color='b',style='dashed')
nx.draw_networkx_edges(G,pos,edgelist=non, width=6,alpha=0.5,edge_color='r',style='solid')
labels = nx.get_edge_attributes(G, 'weight')
nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)
# labels
nx.draw_networkx_labels(G,pos,font_size=20,font_family='sans-serif')

plt.axis('off')
plt.savefig("weighted_graph.png") # save as png
plt.show() # display