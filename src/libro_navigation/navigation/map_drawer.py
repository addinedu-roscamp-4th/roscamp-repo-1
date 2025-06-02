import cv2
import numpy as np
import yaml
from navigation.astar import Astar

class MapDrawer:
    def __init__(self, pgm_path, yaml_path):
        self.pgm_path = pgm_path
        self.yaml_path = yaml_path
        self.grid = []
        self.resolution = None
        self.origin = None
        self.height = 0
        self.width = 0

        self.load_map()

    def load_map(self):
        # YAML 정보 읽기
        with open(self.yaml_path, 'r') as file:
            map_info = yaml.safe_load(file)

        self.resolution = map_info['resolution']
        self.origin = map_info['origin']

        # PGM 이미지 로드
        pgm_img = cv2.imread(self.pgm_path, cv2.IMREAD_GRAYSCALE)
        self.height, self.width = pgm_img.shape

        self.grid = []
        for i in range(self.height):
            row = []
            for j in range(self.width):
                pixel = pgm_img[i, j]
                if pixel == 205:
                    row.append(-1)  # unknown
                elif pixel < 200:
                    row.append(1)   # obstacle
                else:
                    row.append(0)   # free
            self.grid.append(row)

    def save_debug_image(self, start_pose=None, goal_pose=None, filename="grid_debug.png", mark_size=1):
        # 색상 정의 (BGR)
        COLOR_OBSTACLE = (0, 0, 0)
        COLOR_FREE = (255, 255, 255)
        COLOR_UNKNOWN = (127, 127, 127)
        COLOR_GOAL = (0, 0, 255) # 목적지(빨간색)
        COLOR_START = (0, 255, 0) # 시작점(초록색)

        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        for y in range(self.height):
            for x in range(self.width):
                val = self.grid[y][x]
                if val == 1:
                    img[y, x] = COLOR_OBSTACLE
                elif val == 0:
                    img[y, x] = COLOR_FREE
                else:
                    img[y, x] = COLOR_UNKNOWN

        if goal_pose:
            gy, gx = goal_pose
            for dy in range(-mark_size, mark_size + 1):
                for dx in range(-mark_size, mark_size + 1):
                    ny, nx = gy + dy, gx + dx
                    if 0 <= ny < self.height and 0 <= nx < self.width:
                        img[ny, nx] = COLOR_GOAL

        if start_pose: 
            sy, sx = start_pose
            for dy in range(-mark_size, mark_size + 1):
                for dx in range(-mark_size, mark_size + 1):
                    ny, nx = sy + dy, sx + dx
                    if 0 <= ny < self.height and 0 <= nx < self.width:
                        img[ny, nx] = COLOR_START

        # 경로 그리기
        if self.grid and goal_pose:
            planner = Astar()
            path = planner.run(self.grid, start_pose, goal_pose)
            img = self.draw_path(img, path)

        cv2.imwrite(filename, img)
        print(f"[디버그] 이미지 저장 완료: {filename}")
        return img

    def draw_path(self, img, path):
        COLOR_PATH = (0, 255, 255)
        if path is None:
            return img

        for y, x in path:
            if 0 <= y < self.height and 0 <= x < self.width:
                img[y, x] = COLOR_PATH
        return img

    def get_grid(self):
        return self.grid

    def get_resolution(self):
        return self.resolution

    def get_origin(self):
        return self.origin

    def get_size(self):
        return self.height, self.width


if __name__ == "__main__":
    # PGM과 YAML 경로
    pgm_path = "./map/libro_map.png"
    yaml_path = "./map/libro_map.yaml"
    map_drawer = MapDrawer(pgm_path, yaml_path)

    map_drawer.load_map()
    # 예시: 목표 좌표 (row, col) 형태
    # example_goal = (-14, 37)
    example_start = (50, 100)
    example_goal = (20, 160)
    map_drawer.save_debug_image(example_start, example_goal)