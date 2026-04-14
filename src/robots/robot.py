class Robot:
    def __init__(self, robot_id: int, pos: tuple[int, int, int]):
        self.id = robot_id
        self.position = pos
        self.path = []
        self.component = None
        self.voxel_index = 0
        self.has_voxel = False

    def step(self):
        if self.path:
            self.position = self.path.pop(0)

        if not self.path:
            self.path = []