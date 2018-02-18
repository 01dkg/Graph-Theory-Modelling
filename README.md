**Dijkstra’s algorithm:** Dijkstra’s is shortest path algorithm, which
follows greedy based approach to determine the shortest path from a
single source weight graph \((G)\), where the weight of each edge from
\(u\) to \(v\) is non-negative i.e. \(w(u,v) \geq 0\). Initially, source
vertex is assigned a cost of 0 and all other vertex assigned with
\(\infty\) cost. To find the shortest path, Dijkstra’s picks the
unvisited vertex with the lowest cost, then calculated the cost from
source to each unvisited neighbour vertex. Time complexity for
Dijkstra’s Algorithm is \(O(n^2)\)

**Bidirectional Dijkstra algorithm**:  
1\. Alternate between forward traversal from \(source\) node and
backward traversal from \(target\) node  
2\. Calculate \(d_f(v)\) distance for forward traversal  
3\. Calculate \(d_b(v)\) distance for backward traversal  
4\. Stop when \(Q_f\) and \(Q_b\) queues are empty  
5\. After traversal end, find node \(u\) with min value of
\(d_f(u) + d_b(u)\) 6. Find shortest path from \(source\) to \(u\) and
from \(u\) to \(target\), combine both paths to get final shortest path
in a graph


Running time for finding shortest path between two points can be
improved by implemented bidirectional search along with Dijkstra
algorithm i.e. executing Dijkstra algorithm from both directions (from
source node to target node and vice versa) simultaneously. For a
homogeneous graph Bidirectional Dijkstra approach has runtime of
\(O(2*(n/2)^2)\), i.e. 2 times faster.

Consider that we have to find the shortest path between Dublin and Cork
city, then Bidirectional Dijkstra algorithm will start exploring paths
from both cities at same time and algorithm will stop traversing when
both paths meet i.e. when a node is scanned from both directions.
However, if Dijkstra has been used here then it will take more time to
find the shortest path, as algorithm will start traversing from a source
node (i.e. Dublin city) and stop when it reaches target node (i.e. Cork
city)

| \textbf{Iteration} 	| \textbf{Nodes} 	| \textbf{Edges} 	| \textbf{Source} 	| \textbf{Target} 	| \textbf{\begin{tabular}[c]{@{}c@{}}Dijkstra \\ run-time (in milliseconds)\end{tabular}} 	| \textbf{\begin{tabular}[c]{@{}c@{}}Bi-Directional Dijkstra\\  run-time (in milliseconds)\end{tabular}} 	|
|--------------------	|:--------------:	|:--------------:	|:---------------:	|:---------------:	|:---------------------------------------------------------------------------------------:	|:------------------------------------------------------------------------------------------------------:	|
| 1                  	|        4       	|        3       	|        0        	|        3        	|                                       0.034383466                                       	|                                               0.025146117                                              	|
| 2                  	|        5       	|        8       	|        0        	|        3        	|                                       0.059016397                                       	|                                               0.047726304                                              	|
| 3                  	|        6       	|       10       	|        0        	|        3        	|                                       0.062095513                                       	|                                               0.05593728                                               	|
| 4                  	|        8       	|       10       	|        0        	|        3        	|                                       0.064148257                                       	|                                               0.051318606                                              	|
| 5                  	|        9       	|       12       	|        0        	|        3        	|                                       0.087754816                                       	|                                               0.077491095                                              	|
| 6                  	|       15       	|       15       	|        0        	|        3        	|                                        0.08980756                                       	|                                               0.072359234                                              	|
| 7                  	|       18       	|       14       	|        0        	|        3        	|                                       0.126756957                                       	|                                               0.088781188                                              	|
| 8                  	|       32       	|       28       	|        0        	|        3        	|                                       0.167811841                                       	|                                               0.134967934                                              	|
| 9                  	|       50       	|       35       	|        0        	|        3        	|                                       0.185773353                                       	|                                               0.140099794                                              	|
| 10                 	|       60       	|       35       	|        0        	|        3        	|                                       0.119572352                                       	|                                               0.095452607                                              	|
