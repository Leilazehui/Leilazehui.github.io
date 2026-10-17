"""
astar_planner.py
----------------
Grid-based A* path planner (8-connected) with *no diagonal corner-cutting*.

Constraint:
- You cannot move diagonally between two free cells if the two side-adjacent cells form a blocked corner.
  Example 2x2 cells:
    [1 2
     3 4]
  If cell 1 and 4 are obstacles, then 2 cannot move directly to 3 (diagonal across the corner).
  More generally: for a diagonal step, both adjacent orthogonal neighbors must be free.
"""

from __future__ import annotations
from typing import Dict, List, Optional, Tuple, Iterable
import math
import heapq
import numpy as np


Cell = Tuple[int, int]


def in_bounds(grid: np.ndarray, r: int, c: int) -> bool: #grid is defined as a Nth-dimensional homogeneous array? 
    return 0 <= r < grid.shape[0] and 0 <= c < grid.shape[1] 
    # r and c tells you the position (row and column) of the grid.abs
    #grid.shape[0] means the row, and [1] should be the column
    # e.g., r=1, c=2, then gridp[1,2] means the element on the 1st row and 2nd column is ===0 (free) or == 1(obstacle)

    # shape() return dimension of an array or dataframe as rows and column in tuples, e.g., (1, 2)
    '''
    in the map_generator file, 
    grid = (rng.random((N, N)) < obstacle_density).astype(np.uint8)
    so grid is a random size array. 
    grid.shape tells us the size of the grid in tuple (row, column)
    in_bounds helps us check the node position is out of the grid map or not. 
    '''


def is_free(grid: np.ndarray, r: int, c: int) -> bool: #this verify whether the node is on a empty grid or the grid is an obstacle? whether node is free?
    return in_bounds(grid, r, c) and grid[r, c] == 0 #not sure what it is doing
'''
if it is within the map, in_bounds checked, and if the grid is free (not obstacle), nice
e.g., grid = [
[0, 1],
[0, 0]
]
is_free(grid, 0, 1):
0th row, and 1st column is 1, it's obstacle, return False
is_free(grid, 1, 0):
1st row, 0th column is 0, free space, return True
'''

def heuristic(a: Cell, b: Cell) -> float: #this function calcualtes the heuristic cost, how far is goal from the current node
    return math.hypot(a[0] - b[0], a[1] - b[1]) #calcualte the euclidean distance

    # we input a and b, where one of them is current, one of them is goal.

def neighbors_8_no_corner_cut(grid: np.ndarray, r: int, c: int) -> Iterable[Tuple[int, int, float]]: #Iterable makes sure the whole function can iterate?
    """
    8-neighborhood with corner-cut prevention:
    For diagonal moves, require both adjacent orthogonal cells to be free.
    """
    # 4-connected
    cardinal = [(-1, 0), (1, 0), (0, -1), (0, 1)] #these 4 tuples refers the 4 directions the path can go, excludes diagonal coordinates
    for dr, dc in cardinal: #dr refers to row direction, dc refers to column direction
        nr, nc = r + dr, c + dc #the node moves in one of these 4 directions in 1 unit, and stored in nr, nc
        if is_free(grid, nr, nc): #if [nr, nc] is within the map and == 0 (no obstacle)
            yield nr, nc, 1.0 # 1.0 here means the cost of motion. 4 directions only need a cost of 1.
        '''
        using yield instead of return here reportvalid elements one by one. return return everything all at once
        but since yield checks whether the grid is valid one at each time, it outputs one grid that is verified each time.
        so it checked the up, down, left and right direction one by one. 

        the Iterable[] tells this fn generates a lot of (int, int, float)
        it will keep iterate over.
        '''
        
    cornercut = [(-1, 1), (1, 1), (-1, -1), (1, -1)]
    for cr, cc in cornercut:
        tr, tc = r + cr, c + cc
        if not is_free(grid, tr, c):
            continue
        if not is_free(grid, r, tc):
            continue
        if not is_free(grid, tr, tc):
            continue
        
        yield tr, tc, math.sqrt(2)
          
          
        '''
        if is_free(grid, tr, tc):
            if is_free(grid, tr, c):
                if is_free(grid, r, tc):
                    yield tr, tc, math.sqrt(2)
            return
        return
        problem here: in first if-loop, if verified, it will end right after becuz of "return" here.
        so it cannot write in this way. furthermore, this loop duplicates, as the conditions are already checked in previous lines.
        '''

'''
so for diagonal, the cost would be sqrt(2), while cardinal directions are only 1
'''
    # diagonals
    # [write your code here to achieve diagonal moves with corner-cut prevention]



def reconstruct_path(came_from: Dict[Cell, Cell], start: Cell, goal: Cell) -> List[Cell]:
    # [write your code here to provide the path reconstruction logic]
    '''
    the purpose of camefrom stores the immediate predecessor of the node in the best path f/ound so far. without camefrom, know cost, not know path
    while the A* can only record the h_cost (the cost from start to current) and h(cost (the cost from current to goal))
    there is an open set that stores the priority queue, with the node that is the cheapest, AKA current.
    the algorithm to check the neighbors of the current, and find the cheapest path.
    current = node_with_the_lowest_f_in_the_open_Set
    if yes, update:
        current[neighbor] = current
        g_score[neighbor] = tentative g_score
    A* dont record every path, just the best path by storing only 1 link per node(parent), 
    when goal is found, we can walk backwards using cameFrom until reach the start.
    '''
    path = []
    current = goal
    while current != start:
        path.append(current)
        current = came_from[current]
    
    path.append(start)
    path.reverse()

    return path

    '''
    reconstruction logic only finds the entire path and then reverse it. it does not detect and analyze nodes. 
    the came_from dictionary, start and goal nodes are recorded in the A* algorithm, not here. but this function is for reversing the path. why?
    here is how came_from works:
    node starts from A, found B: current = A, neighbor = B
    that means B is from A: came_from[B] = A
    then, current = B, neighbor = C, that means C is from B: came_from[C] = B
    FINALLY, current = C, neighbor = G, means G is from C, came_from[G] = C
    all these appended nodes in the came_from becomes: came_from = {
    B:A,
    C:B,
    G:C}
    then, as the node stops at the goal, now, current == goal
    to append these nodes into a new list, we will get a reversed path, because the first node is the current node == goal
    so, path = []
    append(current) -> path = [G, C, B, A]

    ***in the dictionary, the relatioinship between the key and values are: [child:parent]
    that means when you call the child (key), you get the parent (value)
    so, the first call: append(current), we append G, then at came_from[current], G called C, append C; then C call B, append B, B called A, append A.
    what it looks like in code:
    came_from[G] = C, append C
    came_from[C] = B, append B
    came_from[B] = A, append A

    TO GET THE correct path, we reverse it: 
    path.reverse
    then, we get path = [A, B, C, G]. Done!

   * for heapq, it is used to implment open list. it recorded the nodes around the current nodes, and then algorithm test each of the nodes to see if it fulfills the requirements.
   that means if the node has the lowest f_cost among all its neighbors from all directions.
   so, open list consists of nodes that are yet ot be explored. 
   then it is sorted using heapq function according to their f_cost. then the node with the lowest cost will be heappoped and expand first 

   came_from does not permanently store the first parent found. If A* later discovers a cheaper path to the same node, 
   came_from[node] will be updated to the new parent. Therefore, came_from always stores the current best-known parent of a node.
    '''


def astar_search(grid: np.ndarray, start: Cell, goal: Cell) -> Tuple[Optional[List[Cell]], List[Cell]]:
    """
    A* on 8-connected grid (no corner-cutting).

    Returns
    -------
    path : Optional[List[Cell]]
        Path from start to goal inclusive, or None if failed.
    explored : List[Cell]
        Order of expanded nodes for visualization.
    """

    '''
    The path list created inside reconstruct_path()
    is the same path returned by astar_search().

    reconstruct_path() is responsible for creating the path.

    astar_search() is responsible for calling reconstruct_path(),
    receiving the path, and returning it to visualization.py.

    The path is Optional because A* may fail to find a route,
    in which case path = None.
    '''
    # You need to call neighbors_8_no_corner_cut and reconstruct_path in your implementation.
    # [write your code here to implement the A* search algorithm using the above helper functions]


    open_heap = []
    came_from = {}
    g_score = {}

    explored = []

    g_score[start] = 0

    start_f = heuristic(start, goal)

    heapq.heappush(open_heap,(start_f, start))

    while open_heap:

        current_f, current = heapq.heappop(open_heap)

        explored.append(current)

        if current == goal:

            path = reconstruct_path(came_from,start,goal)

            return path, explored

        for rr, cc, move_cost in neighbors_8_no_corner_cut(grid,current[0],current[1]):

            neighbor = (rr, cc)

            tentative_g = (g_score[current] + move_cost)

            if (neighbor not in g_score or tentative_g < g_score[neighbor]):

                came_from[neighbor] = current

                g_score[neighbor] = tentative_g

                f_cost = (tentative_g + heuristic(neighbor, goal))

                heapq.heappush(open_heap,(f_cost, neighbor))

    return None, explored


'''
所以是current= start
while open_heap:
  for rr, cc, move+cost in neighbors_8_no_corner_cut(grid, current[0], current[1]):
   找出合法鄰居，計算他們的heuristc function, 並加上move_cost，就是他們的f_cost。
   這個move_cost隨著nodes不斷叠加。然後把這些合法鄰居和他們的f_cost組成tuple，(node, f_cost)放進open_heap。
   接著，open_heap裏的最小的f_cost數值用heapq.heappop(open_heap)拿出來后放進explored。接著，我們要存每個node的成本在g_score，所以是{node:cost}放在g_cost。
   那麽我們能夠g_cost裏找出目前的node，加上他的move_cost，就是tentativ cost, 接著用g_cost(current) + move cost寫出來。對比g_score裏有的其他node，
   如果比上一個node小，就替代它，然後在came_from更新這個nodes，而g_score[neighbor]就變成現在那個node的cost了，而這個數字就會和下一個nodes的tentative cost 比較。
   但是came_from如何知道有什麽nodes呢？如果我一開始不把start，之後那個nodes放進去，它怎麽自動record nodes？

   current = start

↓

neighbor_8_no_corner_cut()

↓

neighbor

↓

tentative_g =
g_score[current]
+
move_cost

↓

發現更好的路？

if neighbor not in g_score
or
tentative_g < g_score[neighbor]

↓

g_score[neighbor]
=
tentative_g

↓

came_from[neighbor]
=
current

↓

h = heuristic(neighbor, goal)

↓

f = tentative_g + h

↓

push (f, neighbor)
進 open_heap
'''
            
        


                
