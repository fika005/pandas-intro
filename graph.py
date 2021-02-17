### graph.py
### an implementation of a graph using an adjacency list.

## helper class representing a node in a graph. For the moment, nodes
## only have names. Later, we will add state variables.

class Node() :
    def __init__(self, n):
        self.name = n

    def __hash__(self):
        return hash(self.name)

### an edge is a link between two nodes. Right now, the only other
### information an edge carries is the weight of the link. Later we
### will add other annotations.

class Edge() :
    def __init__(self, src, dest, weight) :
        self.src = src
        self.dest = dest
        self.weight = weight

### The graph class itself.
### The nodeTable is a dictionary that maps names to Node objects.
### (this keeps us from having to repeatedly search edgeMap.keys())

### The edgeMap is a dictionary that maps nodes to lists of Edges emanating from that node.

class Graph() :

    def __init__(self):
        self.nodeTable = {}
        self.edgeMap = {}

    ### implements the 'in' keyword. Returns true if the node is in the graph.
    def __contains__(self, item):
        return item in self.nodeTable

    def getNode(self, src):
        return self.nodeTable[src]

    def addNode(self, src):
        if src not in self.nodeTable:
            self.nodeTable[src] = Node(src)

    def addEdge(self, src, dest, weight):
        e = Edge(src, dest, weight)
        self.addNode(src)
        self.addNode(dest)
        if src in self.edgeMap:
            self.edgeMap[src].append(e)
        else:
            self.edgeMap[src] = [e]


    ## Assume file is in the mtx format: % is a comment
    ## Otherwise it's source destination weight
    ### The file in the github repo will work as a sample for you.
    ### It's in the format: source, vertex, weight. You should assume that the graph is symmetric -
    ### if there's an edge from a to b, there's an edge from b to a.
    ### You can find lots of others here: http://networkrepository.com/index.php
    def readFromFile(self, fname):
        with open(fname) as f:
            for l in f.readlines():
                if not l.startswith("%"):
                    (s, d, w) = l.split()
                    self.addEdge(s, d, w)
                    self.addEdge(d, s, w)

    ### inputs are the name of a startNode and endNode. Given this,
    ### return a list of Nodes that indicates the path from start to finish, using breadth-first search.
    ###node=source
    # queue = []
    # while node != sink:
    # getEdges(node)
    # for edge in edges :
    #   queue.enqueue(edge)
    # node = queue.dequeue()

    def breadthFirstSearch(self, startNode, endNode):
        queue = []
        node = Node(startNode)
        traversal_list = [node]
        traversal_set = {node.name}
        while node.name != endNode:
            for edge in self.edgeMap[node.name]:
                if edge.dest not in traversal_set:
                    queue.append(Node(edge.dest))
            node = queue.pop(0)
            traversal_set.add(node.name)
            traversal_list.append(node)
        return traversal_list

    ### inputs are the name of a startNode and endNode. Given this,
    ### return a list of Nodes that indicates the path from start to finish, using depth-first search.
    ### node = source
    ### stack = []
    ### while node != sink:
    ###     getEdges(node)
    ###     for edge in edges:
    ###         stack.push(edge)
    ###     node = stack.pop()

    def depthFirstSearch(self, startNode, endNode):
        stack = []
        node = Node(startNode)
        traversal_list = [node]
        traversal_set = {node.name}
        while node.name != endNode:
            for edge in self.edgeMap[node.name]:
                if edge.dest not in traversal_set:
                    stack.append(Node(edge.dest))
            node = stack.pop()
            traversal_set.add(node.name)
            traversal_list.append(node)
        return traversal_list

    ### implement Djikstra's all-pairs shortest-path algorithm.
    ### https://yourbasic.org/algorithms/graph/#dijkstra-s-algorithm
    ### return the array of distances and the array previous nodes.
    '''
    Algorithm Dijkstra(G, s)
    for each vertex v in G
        dist[v] ← ∞
        prev[v] ← undefined
    dist[s] ← 0

    Q ← the set of all nodes in G
    while Q is not empty
        u ← vertex in Q with smallest distance in dist[]
        Remove u from Q.
        if dist[u] = ∞
            break
        for each neighbor v of u
            alt ← dist[u] + dist_between(u, v)
            if alt < dist[v]
                dist[v] ← alt
                prev[v] ← u

    return dist[], prev[]
    '''
    def djikstra(self, startNode):
        q = list(self.nodeTable.keys())
        dist = {vertex: float('inf') for vertex in q}
        prev = {vertex: None for vertex in q}
        dist[startNode] = 0

        while len(q) != 0:
            min_val = float('inf')
            for key in q:
                if dist[key] <= min_val:
                    min_val = dist[key]
                    u = key
            q.remove(u)
            if min_val == float('inf'):
                break

            for edge in self.edgeMap[u]:
                alt = dist[u] + int(edge.weight)
                if alt < dist[edge.dest]:
                    dist[edge.dest] = alt
                    prev[edge.dest] = u

        return dist, prev




    ### takes as input a starting node, and computes the minimum spanning tree, using Prim's algorithm.
    ### https:// en.wikipedia.org/wiki/Prim % 27s_algorithm
    ### you should return a new graph representing the spanning tree generated by Prim's.
    '''
    reached = [source]
    unreached = [other vertices]
    tree = []
    While unreached is not empty: 
        find the lowest - cost edge(u, v) such that: u is in reached v is in unreached
        tree.add((u, v))
        reached.add(v)
        unreached.remove(v)
    '''
    def prim(self, startNode):
        tree = Graph()
        unreached = [key for key in self.nodeTable.keys() if key != startNode]
        reached = [Node(startNode)]
        while len(unreached) != 0:
            min_val = float('inf')
            for r in reached:
                for e in self.edgeMap[r.name]:
                    if int(e.weight) <= min_val and e.dest in unreached:
                        min_val = int(e.weight)
                        min_node = e.dest
                        min_source = e.src
            tree.addEdge(min_source, min_node, str(min_val))
            reached.append(Node(min_node))
            unreached.remove(min_node)
        return tree


    ### 686 students only ###
    ### takes as input a startingNode and returns a list of all nodes in the maximum clique containing this node.
    ### https://en.wikipedia.org/wiki/Clique_problem#Finding_a_single_maximal_clique
    '''
    clique = [source]
    q = [source]
    while q is not empty:   
        next = q.dequeue() 
        if there is an edge betwee next and allnodes in clique: 
            clique.add(next)
            q.enqueue(allnodes connected to next)
    '''
    def clique(self, startNode):
        max_clq = []
        q = [Node(startNode)]
        while len(q) != 0:
            next = q.pop(0)
            in_clique = True
            for n in max_clq:
                connected = False
                for e in self.edgeMap[n.name]:
                    if e.dest == next.name:
                        connected = True
                if connected == False:
                    in_clique = False
            if in_clique:
                max_clq.append(next)
                for e in self.edgeMap[next.name]:
                    if Node(e.dest) not in max_clq:
                        q.append(Node(e.dest))
        return max_clq


if __name__ == "__main__":
    graph = Graph()
    graph.readFromFile("mytest.txt")
    print(graph.breadthFirstSearch('1', '7'))
    print(graph.depthFirstSearch('1', '7'))
    print(graph.djikstra('0'))
    print(graph.prim('0'))
    print(graph.clique('5'))