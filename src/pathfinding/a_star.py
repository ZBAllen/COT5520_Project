import heapq
from src.config import GRID_SIZE, DEPOT_POS
from src.utils import neighbors_3d, corner_neighbors

def heuristic(current_pos: tuple[int, int, int],
              target_pos: tuple[int, int, int]) -> int:
    """Manhattan Distance"""

    return abs(current_pos[0] - target_pos[0]) + abs(current_pos[1] - target_pos[1]) + abs(current_pos[2] - target_pos[2])

def a_star(start_pos: tuple[int, int, int],
           target_vox: tuple[int, int, int],
           built_voxels: set[tuple[int, int, int]]) -> list[tuple[int, int, int]]:
    """
    A* with obstacle avoidance for built structure.

    Args:
        start_pos: The starting position to find a path from.
        target_vox: The target position to find a path to.
        built_voxels: The set of voxels that are already built.

    Returns:
        The path from start_pos to target_pos as a set of ordered voxel positions.
    """

    if target_vox == DEPOT_POS:
        target_pos_neighbors = [target_vox]

    else:
        target_pos_neighbors = neighbors_3d(target_vox)

        target_pos_neighbors.extend(corner_neighbors(target_vox, target_pos_neighbors, built_voxels))

    for target_pos in target_pos_neighbors:

        # Priority queue stores (f_score, position) pairs
        # f_score = g_score + heuristic, prioritizes lower cost paths
        open_set = [(0, start_pos)]

        # Maps each position to the position it came from (for path reconstruction)
        position_came_from = {}

        # Maps each position to its actual cost from start
        position_cost_from_start = {start_pos: 0}

        # Set of positions already fully evaluated
        closed_positions = set()

        while open_set:
            # Pop position with lowest f-score from priority queue
            _, current_pos = heapq.heappop(open_set)

            # Skip if already processed
            if current_pos in closed_positions:
                continue

            closed_positions.add(current_pos)

            # If we reached the goal, reconstruct and return the path
            if current_pos == target_pos:
                reconstructed_path = []

                while current_pos in position_came_from:
                    reconstructed_path.append(current_pos)

                    current_pos = position_came_from[current_pos]

                return list(reversed(reconstructed_path))

            current_pos_neighbors = neighbors_3d(current_pos)

            current_pos_neighbors.extend(corner_neighbors(current_pos, current_pos_neighbors, built_voxels))    # Determines voxels around corners that can be walked to.

            # Explore all neighboring positions
            for neighbor_pos in current_pos_neighbors:
                # Skip neighbors already fully evaluated
                if neighbor_pos in closed_positions:
                    continue

                neighbor_x, neighbor_y, neighbor_z = neighbor_pos

                # Skip neighbors outside grid bounds
                if not (0 <= neighbor_x < GRID_SIZE[0] and
                        0 <= neighbor_y < GRID_SIZE[1] and
                        0 <= neighbor_z < GRID_SIZE[2]):
                    continue

                # Determine if neighbor is walkable:
                # - Built voxels act as surfaces (robots can walk on them, not through them)
                # - Ground level (z=0) is always walkable (unless obstructed by a built voxel)
                # - Empty space above ground is not walkable without an adjacent built voxel

                if neighbor_pos in built_voxels:
                    continue

                is_walkable = False

                if neighbor_z == 0:
                    is_walkable = True

                else:
                    neighbors_neighbors = neighbors_3d(neighbor_pos)

                    for neighbors_neighbor in neighbors_neighbors:
                        if neighbors_neighbor in built_voxels:
                            is_walkable = True

                            break

                if not is_walkable:
                    continue

                # Calculate cost to move this neighbor (always 1 step)
                cost = position_cost_from_start[current_pos] + 1

                # If this path to neighbor is better than previously found path, update it
                if neighbor_pos not in position_cost_from_start or cost < position_cost_from_start[neighbor_pos]:
                    position_cost_from_start[neighbor_pos] = cost

                    # Add neighbor to priority queue with f_score = g + heuristic
                    f_score = cost + heuristic(neighbor_pos, target_pos)

                    heapq.heappush(open_set, (f_score, neighbor_pos))

                    # Record where we came from
                    position_came_from[neighbor_pos] = current_pos

    return []