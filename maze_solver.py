import numpy as np
from collections import deque

class bfs:
    def __init__(self):
        pass

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

        if lines:
            inverted = lines.pop()
            last_line = lines.pop()
            start_line= lines.pop()
            

        with open("adj_matrix.txt", "w") as file:
            file.writelines(lines)

        matrix = np.loadtxt("adj_matrix.txt", delimiter=",", dtype=int)

        start_row, start_col = start_line.split(", ")
        end_row, end_col = last_line.split(", ")
        start_row = int(start_row)
        start_col = int(start_col)
        end_row = int(end_row)
        end_col = int(end_col)

        start_index = self.coord_to_index(start_row, start_col)
        end_index = self.coord_to_index(end_row, end_col)

        path = self.bfs(matrix, start_index, end_index)
        path = self.convert_path(path)

        with open("path.txt", "w") as file:
            for i in range(len(path)):
                line = str(path[i])
                if i != len(path) - 1:
                    line += " "
                line += "\n"
                file.writelines(line)
            file.write(inverted)