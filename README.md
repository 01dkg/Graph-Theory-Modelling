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
