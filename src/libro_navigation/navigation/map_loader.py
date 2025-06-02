import cv2
import yaml

class MapLoader:
    def __init__(self, pgm_path, yaml_path):
        self.pgm_path = pgm_path
        self.yaml_path = yaml_path
        self.grid = []
        self.resolution = None
        self.origin = None
        self.height = 0
        self.width = 0

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

        return self.grid, self.resolution, self.origin, self.height, self.width