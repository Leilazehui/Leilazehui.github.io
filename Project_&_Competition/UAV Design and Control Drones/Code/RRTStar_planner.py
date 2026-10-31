"""
rrtstar_planner.py
------------------
Grid-based collision checking + Continuous RRT* planner.

Updates (v2):
1) Segment collision checking additionally enforces *no diagonal corner-cutting* in the underlying grid:
   When the discretized traversal moves diagonally from cell A to cell B, both adjacent orthogonal cells
   must be free; otherwise the segment is considered in collision.

2) Parent choosing and rewire remain modular and can be disabled for ablation:
   - enable_parent_choosing
   - enable_rewire
"""
from __future__ import annotations
from dataclasses import dataclass
from typing import List, Optional, Tuple
import math
import random
import numpy as np
'''
# a regular class may require manually defining methods such as __init__(), __repr__(), and __eq__()
#__init__(): initializes the object attributes
#__repr__(): defines how the object is displayed
#__eq__(): compares the values of two objects

#In a dataclass, Type annotations define the fields.
#  __init__() is generated automatically, 
#  __repr__(), provides the object representation
#__eq__(): compares objects based on their gield values

dataclass simplifies the format of class.

'''
@dataclass
class Node:
    x: float
    y: float
    parent: Optional[int] = None  # index in nodes list. 
    #used to indicate that a variable, parameter or return value
    # accept either a specified data type or None. Here, it means the value can be an interger or None
    cost: float = 0.0


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))
'''
clamp: 限制範圍
v: value to be limited, lo is low, hi is hi. 
so if _clamp(120, 0 , 100), the result is 100, becuz 120 exceeds the max (which is hi=100)
if _clamp(-3, 0 , 100), the resul is 0, becuz -3 lower than lo (= 0)
in case sample node is out of map, this fn pull it back!
'''

def _dist_xy(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(ax - bx, ay - by)
'''
the euclidean distance between 2 nodes
'''

def point_in_free(grid: np.ndarray, x: float, y: float) -> bool:
    r = int(round(y))
    c = int(round(x))
    if 0 <= r < grid.shape[0] and 0 <= c < grid.shape[1]:
        return grid[r, c] == 0
    return False
'''
see whether the grid is free for node, if it is free space, return true, otherwise, false.
'''

def _is_free_cell(grid: np.ndarray, r: int, c: int) -> bool:
    return 0 <= r < grid.shape[0] and 0 <= c < grid.shape[1] and grid[r, c] == 0
'''
make sure the grid is not obstacle, but a free cell for a node to be placed.
0 = free, 1 = obstacle
'''

def _bresenham_cells(r0: int, c0: int, r1: int, c1: int) -> List[Tuple[int, int]]:
    """Bresenham line on integer grid cells (returns list of (r,c) cells)."""
    cells = []
    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    sr = 1 if r1 >= r0 else -1
    sc = 1 if c1 >= c0 else -1

    r, c = r0, c0
    if dc > dr:
        err = dc / 2.0
        while c != c1:
            cells.append((r, c))
            err -= dr
            if err < 0:
                r += sr
                err += dc
            c += sc
        cells.append((r1, c1))
    else:
        err = dr / 2.0
        while r != r1:
            cells.append((r, c))
            err -= dc
            if err < 0:
                c += sc
                err += dr
            r += sr
        cells.append((r1, c1))
    return cells
'''
err accumulates the error and help algorithm determne when to change row and column.
this fn is to calcualte which cells did a line go through, and find all the cells
that the line will go through to link up two nodes.
'''

def segment_collision_free(
    grid: np.ndarray,
    x0: float, y0: float,
    x1: float, y1: float,
    step: float = 0.5
) -> bool:
    """
    Check collision-free segment with:
    (A) continuous sampling along the segment (step in grid units)
    (B) discrete Bresenham traversal + NO diagonal corner-cut rule
    """
    length = _dist_xy(x0, y0, x1, y1)
    if length == 0:
        return point_in_free(grid, x0, y0) #if nodes dist - 0, it's free space, won't hit any wall
    n = int(math.ceil(length / step)) #round up the number.要多少次step才能夠達到下一個node
    for i in range(n + 1): #which is the no of node dist / step size) + 1
        t = i / n #e.g., 11/10 = 1.x
        x = x0 + t * (x1 - x0) #每一次增加一次step
        y = y0 + t * (y1 - y0)
        if not point_in_free(grid, x, y): #如果發現并不是free cell,代表這個path一路上會有一些obstacle
            return False

    r0, c0 = int(round(y0)), int(round(x0))
    r1, c1 = int(round(y1)), int(round(x1))
    cells = _bresenham_cells(r0, c0, r1, c1) #the nodes that the path will go through
    if not cells: #if not the cell, continue
        return True

    prev = cells[0] #so it's r0?
    if not _is_free_cell(grid, prev[0], prev[1]): #check if it's not a free cell
        return False

    for cur in cells[1:]: #c0, r1, c1
        r_prev, c_prev = prev #every these variables are tuples i believe
        r_cur, c_cur = cur
        if not _is_free_cell(grid, r_cur, c_cur): #check if the other coordinates are free or not?
            return False

        dr = r_cur - r_prev
        dc = c_cur - c_prev
        if dr != 0 and dc != 0: #so that we are not comparing the same node
            if not _is_free_cell(grid, r_prev, c_cur): #make sure any kinds of node formed by these 4 variables are free
                return False
            if not _is_free_cell(grid, r_cur, c_prev):
                return False

        prev = cur

    return True


def sample_free(grid: np.ndarray, rng: random.Random) -> Tuple[float, float]:
    N = grid.shape[0]
    for _ in range(5000): # 
        #it means we dont need this num if we use _. it means we will try for 5000 times at most with randomly generated x,y
        x = rng.uniform(0, N - 1)
        y = rng.uniform(0, N - 1)
        if point_in_free(grid, x, y):
            return x, y # if it's free space, return. To avoid get a sample node that is in the wall.
    free = np.argwhere(grid == 0)  # returns a 2D array of index corrindates 
    # where an array is non-zero or where a given condition is True, 
    # such as (x>1) = coordinate rows like [[0,2],[1,0],...] for matching elements.
    # make sure that the node is not in the wall (non-zero) and is free (0)
    if len(free) == 0:
        raise RuntimeError("No free space in the grid.") #it it's false in the argwhere fn, empty 
    r, c = free[len(free) // 2] #?
    return float(c), float(r)


def nearest_node(nodes: List[Node], x: float, y: float) -> int: #find the node nearest to the sample
    best_i = 0 
    best_d = float("inf") #positive infinity, larger than any real number so thatt d is always true at the first case
    for i, n in enumerate(nodes): #allows to loop over an iterable while automatically keeping track of the index and value,
        #returning paris of (index, element)
        d = _dist_xy(n.x, n.y, x, y)
        if d < best_d:
            best_d = d
            best_i = i
    return best_i
'''but like why use float("inf") to compare?'''

def steer(from_node: Node, to_x: float, to_y: float, step_len: float) -> Tuple[float, float]:
    dx = to_x - from_node.x
    dy = to_y - from_node.y
    d = math.hypot(dx, dy)
    if d <= step_len:
        return to_x, to_y
    scale = step_len / d
    return from_node.x + dx * scale, from_node.y + dy * scale
#move 1 step to the sample node
'''basically, nearest node + sample direction + step size = new node'''

def near_nodes(nodes: List[Node], new_node: Node, radius: float) -> List[int]:
    idx = []
    for i, n in enumerate(nodes):
        if _dist_xy(n.x, n.y, new_node.x, new_node.y) <= radius:
            idx.append(i)
    return idx


def choose_parent(
    grid: np.ndarray,
    nodes: List[Node],
    new_node: Node,
    neighbor_ids: List[int], #only include the 附近節點編號,not cost list
    enable_parent_choosing: bool = True,
) -> int:
    """Parent choosing module (can be disabled for ablation)."""


    # [write your code here to achieve choose_parent]
    if not enable_parent_choosing:
        return new_node.parent
    best_parent = new_node.parent
    parent_node = nodes[new_node.parent]
    best_cost = (parent_node.cost + _dist_xy(parent_node.x, parent_node.y, new_node.x, new_node.y))
    for nid in neighbor_ids:
        neighbor = nodes[nid]
        if not segment_collision_free(grid, neighbor.x, neighbor.y, new_node.x, new_node.y):
            continue
        tentative_cost = (neighbor.cost + _dist_xy(neighbor.x, neighbor.y, new_node.x, new_node.y))
        if tentative_cost < best_cost:
            best_cost = tentative_cost
            best_parent = nid
    return best_parent


def rewire(
    grid: np.ndarray,
    nodes: List[Node],
    new_id: int,
    neighbor_ids: List[int],
    enable_rewire: bool = True,
) -> List[Tuple[int, int]]:
    """Rewire module (can be disabled for ablation)."""

    # [write your code here to achieve rewire]
    if not enable_rewire:
        return 
    new_node = nodes[new_id]
    for nid in neighbor_ids:
        neighbor = nodes[nid]
        if not segment_collision_free(
            grid, new_node.x, new_node.y, neighbor.x, neighbor.y):
            continue
  
        temporary_cost = (
            new_node.cost + _dist_xy(new_node.x, new_node.y, neighbor.x, neighbor.y))

        if temporary_cost < neighbor.cost:
            neighbor.parent = new_id
            neighbor.cost = temporary_cost
    return 



def rrt_star(
    grid: np.ndarray,
    start_rc: Tuple[int, int],
    goal_rc: Tuple[int, int],
    max_iters: int = 2500,
    step_len: float = 3.0,
    goal_sample_rate: float = 0.05,
    neighbor_radius: float = 6.0,
    goal_radius: float = 3.0,
    seed: int = 0,
    enable_parent_choosing: bool = True,
    enable_rewire: bool = True,
) -> Tuple[Optional[List[Tuple[int, int]]], List[Node], List[Tuple[int, int]]]:
    """
    RRT* planner on grid map with collision checking.
    Returns (path_rc, nodes, edges_added).
    """
    sr, sc = start_rc
    gr, gc = goal_rc
    start = Node(float(sc), float(sr), parent=None, cost=0.0)
    goal_xy = (float(gc), float(gr))

    if not point_in_free(grid, start.x, start.y):
        raise ValueError("Start is not in free space.")
    if not point_in_free(grid, goal_xy[0], goal_xy[1]):
        raise ValueError("Goal is not in free space.")

    rng = random.Random(seed)

    nodes: List[Node] = [start]
    edges: List[Tuple[int, int]] = []

    goal_id: Optional[int] = None
    best_goal_cost = float("inf")

    for _ in range(max_iters):
        if rng.random() < goal_sample_rate:
            x_rand, y_rand = goal_xy
        else:
            x_rand, y_rand = sample_free(grid, rng)

        nn_id = nearest_node(nodes, x_rand, y_rand)
        x_new, y_new = steer(nodes[nn_id], x_rand, y_rand, step_len)

        N = grid.shape[0]
        x_new = _clamp(x_new, 0, N - 1)
        y_new = _clamp(y_new, 0, N - 1)
        if not point_in_free(grid, x_new, y_new):
            continue
        if not segment_collision_free(grid, nodes[nn_id].x, nodes[nn_id].y, x_new, y_new):
            continue

        new_node = Node(
            x_new,
            y_new,
            parent=nn_id,
            cost=nodes[nn_id].cost + _dist_xy(nodes[nn_id].x, nodes[nn_id].y, x_new, y_new),
        )

        neigh = near_nodes(nodes, new_node, neighbor_radius)
        if nn_id not in neigh:
            neigh.append(nn_id)

        parent_id = choose_parent(grid, nodes, new_node, neigh, enable_parent_choosing=enable_parent_choosing)
        
        new_node.parent = parent_id
        new_node.cost = nodes[parent_id].cost + _dist_xy(nodes[parent_id].x, nodes[parent_id].y, new_node.x, new_node.y)

        new_id = len(nodes)
        nodes.append(new_node)
        edges.append((new_node.parent if new_node.parent is not None else -1, new_id))

        _ = rewire(grid, nodes, new_id, neigh, enable_rewire=enable_rewire)

        if _dist_xy(new_node.x, new_node.y, goal_xy[0], goal_xy[1]) <= goal_radius:
            cand_cost = new_node.cost + _dist_xy(new_node.x, new_node.y, goal_xy[0], goal_xy[1])
            if cand_cost < best_goal_cost:
                best_goal_cost = cand_cost
                goal_id = new_id

    if goal_id is None:
        return None, nodes, edges

    path_xy: List[Tuple[float, float]] = [goal_xy]
    cur = goal_id
    while cur is not None:
        n = nodes[cur]
        path_xy.append((n.x, n.y))
        cur = n.parent
    path_xy.reverse()

    path_rc = [(int(round(y)), int(round(x))) for (x, y) in path_xy]
    compact = []
    for p in path_rc:
        if not compact or compact[-1] != p:
            compact.append(p)
    return compact, nodes, edges











'''
wrong version:
best_cost = inf
best_parent = new_code.parent
for nid in neighbor_ids:
    neighbor = nodes[nid]
    tentative_cost = neighbor.cost + _dist_xy(neighbor, new_node)

    if tentative_cost < new_node.cost:
        best_cost = neighbor.cost
        best_parent = neighbor
return best parent

problem 1: _dist_xy(neighbor.x, neighbor.y, new_node.x, new_node.y)

problem 2: if tentative_cost < best_cost
Analysis: the new_node.cost is not assured. so it's just an expectation value.  
as we want to find the current lowest cost, should compare with best_cost
so, it should be if tentative_cost < best_cost:

problem 3: best_cost = neighbor.cost
Analysis: as we just calcaulted, tentative_cost = neighbor.cost + distance
so the neighbor.cost is already included in the tentative_cost, and now if we are say
best_cost = neighbor.cost, it's meaningless
so, it should be best_cost = tentative_cost

problem 4: best_parent = neighbor
Analysis: as mentioned parent_id = choose_parent(...)
it should return nid instead of neighbor
so it should be best_parent = nid

draft 2:
best_cost = inf
best_parent = new_code.parent
best_cost_1 = new_code.cost + _dist_xy(neighbor, new_node) 
for nid in neighbor_ids:
    neighbor = nodes[nid]
    tentative_cost = neighbor.cost + _dist_xy(neighbor.x, neighbor.y, new_node.x, new_node.y)

    if best_cost_1 < best_cost:
        if tentative_cost < best_cost:
            best_cost = tentative_cost
            best_parent = nid
return best parent

problem 1: best_cost_1 is unnecessary. 
what i want should be the expected cost of the parent node. so, sth like:
parent_node = nodes[new_node.parent]
best_cost = parent_node.cost + _dist_xy(parent_node.x, parent_node.y, new_node.x, new_node.y)

draft 3:
best_parent = new_node.parent
parent_node = nodes[new_node.parent]
best_cost = parent_node.cost + _dist_xy(parent_node.x, parent_node.y, new_node.x, new_node.y)

for nid in neighbor_ids:
    neighbor = nodes[nid]
    tentative_cost = neighbor.cost + _dist_xy(neighbor.x, neighbor.y, new_node.x, new_node.y)
    if segment_collision_free():
        if tentative_cost < best_cost:
            best_cost = tentative_cost
            best_parent = nid
return best_parent
'''

'''
rewire draft:
    if not enabled_rewire:
        return 
    for nid in neighbor_ids:
        neighbor = nodes[nid]
        for id in new_id:
            neighbor_cost = neighbor + node[id]
            temporary_cost = new_node.cost + _dist_xy(new_node.x, new_node.y, neighbor.x, neighbor.y)
            if temporary_cost < neighbor_cost:
                node[neighbor] = new_node
    return 

problem 1: new_id is an integer, e.g., new_id = 24, then nodes[24] means the newly added new node
so "for id in new_id" is unnecessary.

problem 2:newighbor_cost = neighbor + node[id]
Analysis: neighbor is not a number, but an object from Node.
what really need to be compared should be neighbor.cost

problem 3: node[neighbor] = new_node
neighbor is not an index, so it cant be put in this way.
e.g., neighbor.parent = new_id

problem 4: neighbor_cost 不需要被定義，本來已經有的object

problem 5: add segment_collision_free
'''
