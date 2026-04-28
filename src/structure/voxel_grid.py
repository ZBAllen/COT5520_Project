import random
from src.config import GRID_SIZE, RANDOM_VOXELS, RANDOM_STRUCTURE_ORIGINS, TEST_PRESET_NUM
from src.robots.robot import Robot


class VoxelGrid:
    def __init__(self):
        self.built = set()
        self.target = set()
        self.scaffold = set()

    def add_structure(self, structure_type: str):
        if structure_type == "cube":
            self.add_rectangular_prism((5, 5, 0), (4, 4, 4))

        elif structure_type == "pyramid":
            self.add_pyramid((5, 5, 0), 5)

        elif structure_type == "wall":
            self.add_rectangular_prism((5, 5, 0), (6, 1, 4))

        elif structure_type == "overhang":
            self.add_overhang((5, 5, 0), 10, 20)

        elif structure_type == "overhangs":
            self.add_overhangs((5, 5, 0), 3, 20)

        elif structure_type == "random":
            self.add_random_connected(RANDOM_VOXELS)

        elif structure_type == "preset_test":
            self.add_preset(TEST_PRESET_NUM)

    # ------------------------
    # PRESETS
    # ------------------------

    def add_rectangular_prism(self, origin: tuple[int, int, int], size: tuple[int, int, int]):
        ox, oy, oz = origin
        sx, sy, sz = size

        for x in range(sx):
            for y in range(sy):
                for z in range(sz):
                    self.target.add((ox + x, oy + y, oz + z))

    def add_pyramid(self, origin: tuple[int, int, int], size: int):
        ox, oy, oz = origin

        for z in range(size):
            for x in range(size - z):
                for y in range(size - z):
                    self.target.add((ox + x, oy + y, oz + z))

    def add_overhang(self, origin: tuple[int, int, int], height: int = 3, arm_length: int = 3):
        ox, oy, oz = origin

        # Vertical column
        for z in range(height):
            self.target.add((ox, oy, oz + z))

        # Horizontal arm extending in the x-direction from the top of the column
        for dx in range(1, arm_length + 1):
            self.target.add((ox + dx, oy, oz + height - 1))

    def add_overhangs(self, origin: tuple[int, int, int], num_overhangs: int = 3, arm_length: int = 3):
        ox, oy, oz = origin

        # Ground anchor column so the dependency graph has at least one ground root
        first_base_z = oz + 2

        for z in range(first_base_z):
            self.target.add((ox, oy, oz + z))

        for i in range(num_overhangs):
            # Each stub starts 2 voxels higher than the previous, with a gap below it
            base_z = oz + 2 + (i * 3)

            # Short 2-voxel tall stub (not connected to ground)
            self.target.add((ox + (i * 2), oy, base_z))
            self.target.add((ox + (i * 2), oy, base_z + 1))

            # Horizontal arm extending in x from the top of stub
            for dx in range(1, arm_length + 1):
                self.target.add((ox + (i * 2) + dx, oy, base_z + 1))

        self.target.add((ox + 15, oy, 7))
        self.target.add((ox + 15, oy, 8))

    # ------------------------
    # RANDOM CONNECTED STRUCTURE
    # ------------------------

    def add_random_connected(self, num_voxels: int, num_origins: int = RANDOM_STRUCTURE_ORIGINS):
        """Generate random connected structure with multiple ground origins."""

        max_x, max_y, max_z = GRID_SIZE

        if num_origins is None:
            num_origins = 1

        # Create multiple ground origin points
        origins = set()

        for _ in range(num_origins):
            while True:
                origin = (random.randint(0, max_x - 1),
                          random.randint(0, max_y - 1),
                          0)
                if origin not in origins:
                    origins.add(origin)
                    break

        # Add all origins to target
        for origin in origins:
            self.target.add(origin)

        # Grow from all origins with shared frontier
        frontier = list(origins)

        while len(self.target) < num_voxels and frontier:
            base = random.choice(frontier)

            x, y, z = base

            neighbors = [
                (x + 1, y, z), (x - 1, y, z),
                (x, y + 1, z), (x, y - 1, z),
                (x, y, z + 1), (x, y, z - 1)
            ]

            random.shuffle(neighbors)

            for n in neighbors:
                nx, ny, nz = n

                if not (0 <= nx < max_x and
                        0 <= ny < max_y and
                        0 <= nz < max_z):
                    continue

                # Must be attached to existing structure (or uncomment next line for ground)
                # if nz == 0 or base in self.target:
                if base in self.target:
                    if n not in self.target:
                        self.target.add(n)

                        frontier.append(n)

                        break

        print(f"Generated structure with {len(self.target)} voxels from {num_origins} origin(s)")

    def add_preset(self, preset_num: int):
        if preset_num == 1:
            # Two cubes

            self.add_rectangular_prism((0, 5, 0), (5, 5, 5))
            self.add_rectangular_prism((10, 0, 0), (5, 5, 5))

        elif preset_num == 2:
            # A double-link chain

            self.target.add((5, 5, 0))
            self.target.add((5, 5, 1))
            self.target.add((5, 4, 1))
            self.target.add((5, 6, 1))
            self.target.add((5, 4, 2))
            self.target.add((5, 6, 2))
            self.target.add((5, 4, 3))
            self.target.add((5, 5, 3))
            self.target.add((5, 6, 3))
            self.target.add((5, 4, 4))
            self.target.add((5, 6, 4))
            self.target.add((5, 4, 5))
            self.target.add((5, 5, 5))
            self.target.add((5, 6, 5))
            self.target.add((5, 5, 6))

            self.target.add((6, 5, 6))
            self.target.add((7, 5, 6))
            self.target.add((8, 5, 6))
            self.target.add((9, 5, 6))
            self.target.add((10, 5, 6))
            self.target.add((11, 5, 6))
            self.target.add((12, 5, 6))
            self.target.add((13, 5, 6))
            self.target.add((14, 5, 6))
            self.target.add((15, 5, 6))

        elif preset_num == 3:
            # A v-shaped structure

            self.target.add((5, 5, 0))
            self.target.add((5, 5, 1))
            self.target.add((5, 4, 1))
            self.target.add((5, 6, 1))
            self.target.add((5, 4, 2))
            self.target.add((5, 6, 2))
            self.target.add((5, 3, 2))
            self.target.add((5, 7, 2))
            self.target.add((5, 3, 3))
            self.target.add((5, 7, 3))
            self.target.add((5, 2, 3))
            self.target.add((5, 8, 3))

        elif preset_num == 4:
            # Two cube outlines

            self.target.add((5, 5, 0))
            self.target.add((6, 5, 0))
            self.target.add((7, 5, 0))
            self.target.add((8, 5, 0))
            self.target.add((9, 5, 0))
            self.target.add((10, 5, 0))

            self.target.add((10, 6, 0))
            self.target.add((10, 7, 0))
            self.target.add((10, 8, 0))
            self.target.add((10, 9, 0))
            self.target.add((10, 10, 0))

            self.target.add((9, 10, 0))
            self.target.add((8, 10, 0))
            self.target.add((7, 10, 0))
            self.target.add((6, 10, 0))
            self.target.add((5, 10, 0))

            self.target.add((5, 9, 0))
            self.target.add((5, 8, 0))
            self.target.add((5, 7, 0))
            self.target.add((5, 6, 0))

            self.target.add((5, 5, 0))
            self.target.add((5, 5, 1))
            self.target.add((5, 5, 2))
            self.target.add((5, 5, 3))
            self.target.add((5, 5, 4))
            self.target.add((5, 5, 5))

            self.target.add((10, 5, 0))
            self.target.add((10, 5, 1))
            self.target.add((10, 5, 2))
            self.target.add((10, 5, 3))
            self.target.add((10, 5, 4))
            self.target.add((10, 5, 5))

            self.target.add((10, 10, 0))
            self.target.add((10, 10, 1))
            self.target.add((10, 10, 2))
            self.target.add((10, 10, 3))
            self.target.add((10, 10, 4))
            self.target.add((10, 10, 5))

            self.target.add((5, 10, 0))
            self.target.add((5, 10, 1))
            self.target.add((5, 10, 2))
            self.target.add((5, 10, 3))
            self.target.add((5, 10, 4))
            self.target.add((5, 10, 5))

            self.target.add((5, 5, 5))
            self.target.add((6, 5, 5))
            self.target.add((7, 5, 5))
            self.target.add((8, 5, 5))
            self.target.add((9, 5, 5))
            self.target.add((10, 5, 5))

            self.target.add((10, 6, 5))
            self.target.add((10, 7, 5))
            self.target.add((10, 8, 5))
            self.target.add((10, 9, 5))
            self.target.add((10, 10, 5))

            self.target.add((9, 10, 5))
            self.target.add((8, 10, 5))
            self.target.add((7, 10, 5))
            self.target.add((6, 10, 5))
            self.target.add((5, 10, 5))

            self.target.add((5, 9, 5))
            self.target.add((5, 8, 5))
            self.target.add((5, 7, 5))
            self.target.add((5, 6, 5))

            self.target.add((5, 5, 5))
            self.target.add((5, 5, 6))
            self.target.add((5, 5, 7))
            self.target.add((5, 5, 8))
            self.target.add((5, 5, 9))
            self.target.add((5, 5, 10))

            self.target.add((10, 5, 5))
            self.target.add((10, 5, 6))
            self.target.add((10, 5, 7))
            self.target.add((10, 5, 8))
            self.target.add((10, 5, 9))
            self.target.add((10, 5, 10))

            self.target.add((10, 10, 5))
            self.target.add((10, 10, 6))
            self.target.add((10, 10, 7))
            self.target.add((10, 10, 8))
            self.target.add((10, 10, 9))
            self.target.add((10, 10, 10))

            self.target.add((5, 10, 5))
            self.target.add((5, 10, 6))
            self.target.add((5, 10, 7))
            self.target.add((5, 10, 8))
            self.target.add((5, 10, 9))
            self.target.add((5, 10, 10))

            self.target.add((5, 5, 10))
            self.target.add((6, 5, 10))
            self.target.add((7, 5, 10))
            self.target.add((8, 5, 10))
            self.target.add((9, 5, 10))
            self.target.add((10, 5, 10))

            self.target.add((10, 6, 10))
            self.target.add((10, 7, 10))
            self.target.add((10, 8, 10))
            self.target.add((10, 9, 10))
            self.target.add((10, 10, 10))

            self.target.add((9, 10, 10))
            self.target.add((8, 10, 10))
            self.target.add((7, 10, 10))
            self.target.add((6, 10, 10))
            self.target.add((5, 10, 10))

            self.target.add((5, 9, 10))
            self.target.add((5, 8, 10))
            self.target.add((5, 7, 10))
            self.target.add((5, 6, 10))

        elif preset_num == 5:
            self.target.add((5, 5, 0))
            self.target.add((5, 6, 0))
            self.target.add((6, 5, 0))
            self.target.add((6, 6, 0))

            self.target.add((5, 5, 1))
            self.target.add((5, 6, 1))
            self.target.add((6, 5, 1))
            self.target.add((6, 6, 1))

            self.target.add((5, 5, 2))
            self.target.add((5, 6, 2))
            self.target.add((6, 5, 2))
            self.target.add((6, 6, 2))
            self.target.add((4, 5, 2))
            self.target.add((4, 6, 2))
            self.target.add((7, 5, 2))
            self.target.add((7, 6, 2))
            self.target.add((11, 5, 2))
            self.target.add((11, 6, 2))
            self.target.add((12, 5, 2))
            self.target.add((12, 6, 2))

            self.target.add((3, 5, 3))
            self.target.add((3, 6, 3))
            self.target.add((4, 5, 3))
            self.target.add((4, 6, 3))
            self.target.add((7, 5, 3))
            self.target.add((7, 6, 3))
            self.target.add((8, 5, 3))
            self.target.add((8, 6, 3))
            self.target.add((10, 5, 3))
            self.target.add((10, 6, 3))
            self.target.add((11, 5, 3))
            self.target.add((11, 6, 3))
            self.target.add((12, 5, 3))
            self.target.add((12, 6, 3))

            self.target.add((2, 5, 4))
            self.target.add((2, 6, 4))
            self.target.add((3, 5, 4))
            self.target.add((3, 6, 4))
            self.target.add((8, 5, 4))
            self.target.add((8, 6, 4))
            self.target.add((9, 5, 4))
            self.target.add((9, 6, 4))
            self.target.add((10, 5, 4))
            self.target.add((10, 6, 4))
            self.target.add((11, 5, 4))
            self.target.add((11, 6, 4))

            self.target.add((2, 5, 5))
            self.target.add((2, 6, 5))
            self.target.add((3, 5, 5))
            self.target.add((3, 6, 5))
            self.target.add((8, 5, 5))
            self.target.add((8, 6, 5))
            self.target.add((9, 5, 5))
            self.target.add((9, 6, 5))
            self.target.add((10, 5, 5))
            self.target.add((10, 6, 5))

            self.target.add((3, 5, 6))
            self.target.add((3, 6, 6))
            self.target.add((4, 5, 6))
            self.target.add((4, 6, 6))
            self.target.add((7, 5, 6))
            self.target.add((7, 6, 6))
            self.target.add((8, 5, 6))
            self.target.add((8, 6, 6))

            self.target.add((4, 5, 7))
            self.target.add((4, 6, 7))
            self.target.add((5, 5, 7))
            self.target.add((5, 6, 7))
            self.target.add((6, 5, 7))
            self.target.add((6, 6, 7))
            self.target.add((7, 5, 7))
            self.target.add((7, 6, 7))

            self.target.add((5, 5, 8))
            self.target.add((5, 6, 8))
            self.target.add((6, 5, 8))
            self.target.add((6, 6, 8))

    def can_build(self,
                  voxel: tuple[int, int, int],
                  voxel_index: int,
                  robots: list[Robot],
                  component_order: list[tuple[int, int, int]]):
        """Returns if the given voxel can be built."""

        x, y, z = voxel

        # Must be part of structure or scaffold
        if voxel not in self.target and voxel not in self.scaffold:
            print("not part of struct")
            return False

        # Already built
        if voxel in self.built:
            print("already built")
            return False

        # Check if previous voxel in component has been placed
        if voxel_index > 0 and component_order[voxel_index - 1] not in self.built:
            print("prev not placed")
            return False

        # # No robot standing there currently
        # if voxel in [robot.position for robot in robots]:
        #     return False

        # Check all 6 directions
        neighbors = [
            (x + 1, y, z), (x - 1, y, z),
            (x, y + 1, z), (x, y - 1, z),
            (x, y, z + 1), (x, y, z - 1)
        ]

        if neighbors[0] in self.built and neighbors[1] in self.built:
            print("tight build A")
            return False

        if neighbors[2] in self.built and neighbors[3] in self.built:
            print("tight build B")
            return False

        if neighbors[4] in self.built and neighbors[5] in self.built:
            print("tight build C")
            return False

        has_neighbor = False

        for n in neighbors:
            if n in self.built:
                has_neighbor = True

        if has_neighbor:
            return True

        # If the voxel has no neighbors and is on the ground level, it's placeable.
        if z == 0:
            return True

        print(f"no neighbors and not on floor for voxel {voxel}, comp. ordering: {component_order}")

        return False