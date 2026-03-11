import numpy as np
from collections import deque

class bfs:
    def __init__(self,start, end, inversion):
        self.start_row = start[0]
        self.start_col = start[1]
        self.end_row = end[0]
        self.end_col = end[1]
        self.inverted = inversion

    def bfs(self, matrix, start_index, end_index):
        n = len(matrix)
        visited = [False] * n
        prev = [None] * n

        queue = deque()
        queue.append(start_index)
        visited[start_index] = True

        while queue:
            current = queue.popleft()
            if current == end_index:
                break

            for i in range(n):
                if matrix[current][i] > 0 and not visited[i]:
                    visited[i] = True
                    prev[i] = current
                    queue.append(i)

        path = []
        at = end_index
        while at is not None:
            path.append(at)
            at = prev[at]
        path.reverse()

        if not path or path[0] != start_index:
            return [], float('inf')

        return path

    def coord_to_index(self, row, col, rows=6, cols=6):
        return row * cols + col

    def index_to_coord(self, index, rows=6, cols=6):
        row = index // cols
        col = index % cols
        return (row, col)

    def convert_path(self, path, rows=6, cols=6):
        return [self.index_to_coord(i, rows, cols) for i in path]

    def run(self):
        with open("adj_matrix.txt", "r") as file:
            lines = file.readlines()



        matrix = np.loadtxt("adj_matrix.txt", delimiter=",", dtype=int)



        start_index = self.coord_to_index(self.start_row, self.start_col)
        end_index = self.coord_to_index(self.end_row, self.end_col)

        path = self.bfs(matrix, start_index, end_index)
        path = self.convert_path(path)

        with open("path.txt", "w") as file:
            for i in range(len(path)):
                line = str(path[i])
                if i != len(path) - 1:
                    line += " "
                line += "\n"
                file.writelines(line)
            file.write(str(self.inverted) + "\n")