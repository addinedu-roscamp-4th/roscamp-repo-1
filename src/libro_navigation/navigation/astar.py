"""

A* 알고리즘을 이용한 경로생성 노드
  - 기본 A* 알고리즘으로 경로를 생성합니다
  - (이용X)생성된 경로점을 subsample하여 일부 경로만 선택합니다
  - 경로를 단순화하기 위해 크게 방향을 트는 점에서만 방향을 다시 잡을 수 있도록 로직을 추가 했습니다

"""

import heapq
import numpy as np


class Astar:
    def __init__(self, obstacle_padding=9):
        self.directions = [(-1,0),(1,0),(0,-1),(0,1)]  #  상하좌우 총 4방향의 가능한 경로를 탐색
        self.obstacle_padding = obstacle_padding

    def heuristic(self, a, b):
        """
        맨해튼 거리를 기반으로 경로비용을 추정합니다
        """
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def run(self, grid, start, goal):
        new_grid = self.expand_obstacles(grid, self.obstacle_padding) # 장애물에 padding값이 더해진 맵을 이용

        open_set = []
        heapq.heappush(open_set, (0 + self.heuristic(start, goal), 0, start, [start]))
        # open_set: 우선순위 큐 (우선순위 = 추정 총 비용 = 현재까지 비용 + 휴리스틱) 비용이 낮은 노드부터 꺼내도록
        # (추정 총 비용, 현재까지 비용, 현재 노드 좌표, 경로 리스트)

        visited = set()  # 방문한 노드를 저장하는 집합
        
        if grid[start[0]][start[1]] == 1:
            print(f"[A*] 시작 지점이 장애물입니다: {start}")
            return None
        if grid[goal[0]][goal[1]] == 1:
            print(f"[A*] 목표 지점이 장애물입니다: {goal}")
            return None

        while open_set:
            est_total, cost, current, path = heapq.heappop(open_set)
            # 우선순위가 가장 낮은 노드를 꺼냄
            # est_total: 현재 노드의 f(n) = g(n) + h(n)
            # cost: 현재까지 누적 이동 비용 g(n)
            # current: 현재 노드 위치
            # path: 시작점부터 현재 노드까지의 경로 리스트

            # 목표지점 도달시 경로 단순화
            if current == goal:
                simple_path, show_path = self.simplify_path(path, start, goal)

                print(simple_path)
                return simple_path, show_path

            # 이미 방문한 노드는 스킵
            if current in visited:
                continue
            visited.add(current)

            # 4방향으로 경로를 탐색
            for d in self.directions:
                # 다음 노드 좌표를 계산
                nx, ny = current[0] + d[0], current[1] + d[1]
                if 0 <= nx < len(new_grid) and 0 <= ny < len(new_grid[0]) and new_grid[nx][ny] != 1: # 맵 안쪽이고 장애물이 아니면
                    next_node = (nx, ny)
                    if next_node not in visited:
                        new_cost = cost + 1  # 비용 누적
                        new_path = path + [next_node] # 현재경로에 다음노드 추가
                        heapq.heappush(open_set, (new_cost + self.heuristic(next_node, goal), new_cost, next_node, new_path)) # open set에도 다음노드 추가
        return None, None
    

    def expand_obstacles(self, grid, padding=1):
        """
        장애물에 너무 가깝게 경로가 생성되는 것을 방지하기위해 장애물 폭을 실제보다 넓게 인식하도록 설정합니다.
        """
        rows, cols = len(grid), len(grid[0])
        new_grid = [row[:] for row in grid]  # 깊은 복사

        for x in range(rows):
            for y in range(cols):
                if grid[x][y] == 1:
                    for dx in range(-padding, padding+1):
                        for dy in range(-padding, padding+1):
                            nx, ny = x + dx, y + dy
                            if 0 <= nx < rows and 0 <= ny < cols:
                                new_grid[nx][ny] = 1
        return new_grid
    

    def subsample_path(self, path, start, goal):
        """
        경로를 서브샘플링하여 3칸씩 뛰어넘는 방식으로 중간 포인트를 제거합니다.
        시작점과 목적지는 반드시 포함되어야 합니다.
        """
        # 서브샘플링된 경로를 담을 리스트
        simplified_path = [start]

        # 3칸씩 뛰어넘기, start와 goal은 반드시 포함되므로 경로에 넣고
        for i in range(1, len(path) - 1, 3):  # 3칸마다 경로점으로 선택
            simplified_path.append(path[i])

        # 마지막 목적지 추가
        simplified_path.append(goal)

        return simplified_path
    
    def calculate_angle(self, v1, v2):
        """
        두 벡터 v1과 v2 사이의 각도를 계산합니다.
        """
        dot_product = np.dot(v1, v2)
        norm_v1 = np.linalg.norm(v1)
        norm_v2 = np.linalg.norm(v2)
        
        # 코사인 각도 계산
        cos_theta = np.clip(dot_product / (norm_v1 * norm_v2), -1.0, 1.0)
        
        # 라디안으로 각도 계산 후, 이를 도(degree)로 변환
        angle_rad = np.arccos(cos_theta)
        angle_deg = np.degrees(angle_rad)
        
        return angle_deg
    
    
    def simplify_path(self, path, start, goal, angle_threshold=5):
        """
        경로를 단순화합니다.
        threshold 값 이상일 때만 새로운 경로에 점을 추가합니다.
        path: 경로를 나타내는 점들의 리스트
        start: 경로의 시작점
        goal: 경로의 목표점
        angle_threshold: 각도가 이 값 이상일 때만 점을 추가 (단위:deg)
        """
        simplified_path = []  # 첫 번째 점은 시작점으로 지정
        show_path = [start]
        i = 0  # 경로의 인덱스를 처음부터 시작
        
        while i < len(path) - 2:
            # 현재 점과 그 다음, 그 다음 점들로 벡터를 구함
            current_vec = np.array(path[i+1]) - np.array(path[i])
            next_vec = np.array(path[i+2]) - np.array(path[i+1])
            
            # 현재 점과 그 이후 점들로 벡터를 구해 각도 계산
            angle = self.calculate_angle(current_vec, next_vec)
            
            # 각도가 threshold 이상이면 경로에 점 추가(방향을 크게 트는 경우)
            if angle > angle_threshold:
                simplified_path.append(path[i+1])  # 현재 점을 경로에 추가
                show_path.append(path[i+1])
                i += 1  # 경로에서 한 점을 추가했으므로, 다음 점으로 이동
            else:
                i += 1  # 각도가 threshold 이하이면 그냥 넘어감
        
        # 목표점은 항상 포함
        simplified_path.append(goal)
        show_path.append(path[i+1])
        
        return simplified_path, show_path