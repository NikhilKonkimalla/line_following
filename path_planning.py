import queue
import numpy as np


def preprocessing(map, end):
    curr_val = 2
    goal = False
    queue = [end]
    neighbors = [[1, 0], [0, 1], [-1, 0], [0, -1], [1, 1], [-1, 1], [-1, -1], [1, -1]]
    while queue:
        x, y = queue.pop(0)
        val = map[x][y]
        for a, b in neighbors:
            if x+a<len(map) and 0<=x+a and y+b<len(map[0]) and y+b>=0 and map[x+a][y+b]==0:
                map[x+a, y+b]=val-1
                queue.append((x+a, y+b))
                #if x+a = goal = True

    return map

def wavefront(map, start):
    curr_val = 2
    start = False
    queue = [start]
    visited = [start]
    neighbors = [[1, 0], [0, 1], [-1, 0], [0, -1], [1, 1], [-1, 1], [-1, -1], [1, -1]]
    while queue:
        x, y = queue.pop(0)
        val = map[x][y]
        for a, b in neighbors:
            if x+a<len(map) and 0<=x+a and y+b<len(map[0]) and y+b>=0 and map[x+a][y+b]==0:
                if map[x+a][y+b]-1==map[x][y]:
                    visited.append((x+a, x+b))
                    queue.append((x+a, x+b))
                    break
    return visited




