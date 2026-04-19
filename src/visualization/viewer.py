import gc
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from src.config import GRID_SIZE, DEPOT_POS
from src.robots.robot import Robot
from src.structure.voxel_grid import VoxelGrid

class Viewer:
    def __init__(self):
        self.elev = 25
        self.azim = 45
        self.fig = plt.figure()
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        self.auto_zoom = False

    def on_key(self, event):
        if event.key == 'left':
            self.azim += 1
        elif event.key == 'right':
            self.azim -= 1
        elif event.key == 'up':
            self.elev -= 1
        elif event.key == 'down':
            self.elev += 1
        elif event.key == 'z':
            self.auto_zoom = not self.auto_zoom

    def draw_cube(self, ax: Axes3D, position: tuple[int, int, int], color: str = 'green', alpha: float = 1.0):
        """Draw a cube at the given position."""

        x, y, z = position

        vertices = [
            [x, y, z], [x + 1, y, z], [x + 1, y + 1, z], [x, y + 1, z],
            [x, y, z + 1], [x + 1, y, z + 1], [x + 1, y + 1, z + 1], [x, y + 1, z + 1]
        ]

        faces = [
            [vertices[0], vertices[1], vertices[5], vertices[4]],
            [vertices[2], vertices[3], vertices[7], vertices[6]],
            [vertices[0], vertices[3], vertices[7], vertices[4]],
            [vertices[1], vertices[2], vertices[6], vertices[5]],
            [vertices[0], vertices[1], vertices[2], vertices[3]],
            [vertices[4], vertices[5], vertices[6], vertices[7]]
        ]

        cube = Poly3DCollection(faces, alpha=alpha, edgecolor='black', linewidths=0.5)
        cube.set_facecolor(color)
        ax.add_collection3d(cube)

    def draw(self, voxel_grid: VoxelGrid, robots: list[Robot], voxels_available: int = 0, voxels_in_transit: int = 0, total_steps: int = 0):
        self.fig.clear()

        ax = self.fig.add_subplot(111, projection='3d')

        ax.set_box_aspect([
            GRID_SIZE[0],
            GRID_SIZE[1],
            GRID_SIZE[2]
        ])

        # Draw target voxels
        for voxel in voxel_grid.target:
            if voxel in voxel_grid.built:
                # Draw built target voxels (solid blue)
                self.draw_cube(ax, voxel, color='blue', alpha=1.0)

            else:
                # Draw remaining target structure (light gray, translucent)
                self.draw_cube(ax, voxel, color='gray', alpha=0.2)

        # Draw scaffolding
        for voxel in voxel_grid.scaffold:
            if voxel in voxel_grid.built:
                # Draw built scaffold voxels (orange, semi-transparent to distinguish from target)
                self.draw_cube(ax, voxel, color='orange', alpha=0.8)

            else:
                # Draw unbuilt scaffold voxels (light orange ghost, for debugging)
                self.draw_cube(ax, voxel, color='orange', alpha=0.15)

        # Draw robots holding voxels (red spheres)
        rx = [r.position[0] + 0.5 for r in robots if r.has_voxel]
        ry = [r.position[1] + 0.5 for r in robots if r.has_voxel]
        rz = [r.position[2] + 0.5 for r in robots if r.has_voxel]

        ax.scatter(rx, ry, rz, c='red', s=50, marker='o')

        # Draw robots not holding voxels (cyan spheres)
        rx = [r.position[0] + 0.5 for r in robots if not r.has_voxel]
        ry = [r.position[1] + 0.5 for r in robots if not r.has_voxel]
        rz = [r.position[2] + 0.5 for r in robots if not r.has_voxel]

        ax.scatter(rx, ry, rz, c='cyan', s=50, marker='o')

        # Draw depot
        self.draw_cube(ax, DEPOT_POS, color='green', alpha=0.7)

        # Display voxel counts
        ax.text2D(0.05, 0.95, f'Available: {voxels_available} | In Transit: {voxels_in_transit}',
                  transform=ax.transAxes)

        # Display total robot steps
        ax.text2D(0.25, 0.85, f"Total Steps: {total_steps}", transform=ax.transAxes)

        # Set camera
        ax.view_init(elev=self.elev, azim=self.azim)

        # Auto-zoom or use full grid limits
        if self.auto_zoom and voxel_grid.built:
            built_list = list(voxel_grid.built)

            xs = [v[0] for v in built_list]
            ys = [v[1] for v in built_list]
            zs = [v[2] for v in built_list]

            # Add padding
            padding = 1

            ax.set_xlim(min(xs) - padding, max(xs) + 1 + padding)
            ax.set_ylim(min(ys) - padding, max(ys) + 1 + padding)

            if hasattr(ax, "set_zlim"):
                ax.set_zlim(min(zs) - padding, max(zs) + 1 + padding)

            else:
                ax.set_zbound(min(zs) - padding, max(zs) + 1 + padding)

        else:
            ax.set_xlim(0, GRID_SIZE[0])
            ax.set_ylim(0, GRID_SIZE[1])

            if hasattr(ax, "set_zlim"):
                ax.set_zlim(0, GRID_SIZE[2])

            else:
                ax.set_zbound(0, GRID_SIZE[2])

        plt.pause(0.001)

        gc.collect()