import heapq
from src.config import GRID_SIZE, DEPOT_POS


def heuristic(current_pos: tuple[int, int, int],
              target_pos: tuple[int, int, int]) -> int:
    """Manhattan Distance"""

    return abs(current_pos[0] - target_pos[0]) + abs(current_pos[1] - target_pos[1]) + abs(current_pos[2] - target_pos[2])

def neighbors(vertex: tuple[int, int, int]) -> list[tuple[int, int, int]]:
    """Returns the immediate neighbors of the given vertex."""

    x, y, z = vertex

    directions = [
        (1, 0, 0), (-1, 0, 0), (0, 1, 0),
        (0, -1, 0), (0, 0, 1), (0, 0, -1)
    ]

    return [(x + dx, y + dy, z + dz) for dx, dy, dz in directions]

# def corner_neighbors(vertex: tuple[int, int, int],
#                      vertex_neighbors: list[tuple[int, int, int]],
#                      built_voxels: set[tuple[int, int, int]]) -> list[tuple[int, int, int]]:
#     """
#     Returns the neighboring voxels that are around a corner.
#     This is used for when the robots must move to a new face of the same voxel.
#     """
#
#     x, y, z = vertex
#
#     # Determine where the anchor points are (where is the robot standing/attached).
#     # That will determine what corners are available to move to.
#
#     built_neighbors = [neighbor for neighbor in vertex_neighbors if neighbor in built_voxels]

def corner_neighbors(vertex: tuple[int, int, int],
                     vertex_neighbors: list[tuple[int, int, int]],
                     built_voxels: set[tuple[int, int, int]]) -> list[tuple[int, int, int]]:
    """
    Returns the neighboring voxels that are around a corner.
    This is used for when the robots must move to a new face of the same voxel.
    """

    x, y, z = vertex

    built_neighbors = [neighbor for neighbor in vertex_neighbors if neighbor in built_voxels]

    accessible_corners = []

    # For each built neighbor, find its corner voxels
    for neighbor in built_neighbors:
        neighbor_x, neighbor_y, neighbor_z = neighbor

        # Determine which dimension the neighbor differs in
        diff = (neighbor_x - x, neighbor_y - y, neighbor_z - z)

        # The four corners are formed by varying the other two dimensions by ±1
        if diff[0] != 0:  # neighbor differs in x
            corners = [(neighbor_x, neighbor_y - 1, neighbor_z), (neighbor_x, neighbor_y + 1, neighbor_z), (neighbor_x, neighbor_y, neighbor_z - 1), (neighbor_x, neighbor_y, neighbor_z + 1)]

        elif diff[1] != 0:  # neighbor differs in y
            corners = [(neighbor_x - 1, neighbor_y, neighbor_z), (neighbor_x + 1, neighbor_y, neighbor_z), (neighbor_x, neighbor_y, neighbor_z - 1), (neighbor_x, neighbor_y, neighbor_z + 1)]

        else:  # neighbor differs in z
            corners = [(neighbor_x - 1, neighbor_y, neighbor_z), (neighbor_x + 1, neighbor_y, neighbor_z), (neighbor_x, neighbor_y - 1, neighbor_z), (neighbor_x, neighbor_y + 1, neighbor_z)]

        # Check if each corner is accessible (not built and path is clear)
        for corner in corners:
            if corner not in built_voxels:
                # Check if the intermediate voxel is not built
                intermediate = tuple(n if n != v else c for n, v, c in zip(neighbor, vertex, corner))

                if intermediate not in built_voxels:
                    accessible_corners.append(corner)

    return accessible_corners

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
        target_pos_neighbors = neighbors(target_vox)

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

            current_pos_neighbors = neighbors(current_pos)

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
                    neighbors_neighbors = neighbors(neighbor_pos)

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

# def a_star(start_pos: tuple[int, int, int],
#            target_pos: tuple[int, int, int],
#            built_voxels: set[tuple[int, int, int]]) -> list[tuple[int, int, int]]:
#     # Find all valid positions in the cube surrounding the target
#     target_neighbors = []
#     for dx in [-1, 0, 1]:
#         for dy in [-1, 0, 1]:
#             for dz in [-1, 0, 1]:
#                 # Skip the center (the target voxel itself)
#                 if dx == 0 and dy == 0 and dz == 0:
#                     continue
#
#                 adj_pos = (target_pos[0] + dx, target_pos[1] + dy, target_pos[2] + dz)
#                 adj_x, adj_y, adj_z = adj_pos
#
#                 # Check bounds
#                 if not (0 <= adj_x < GRID_SIZE[0] and
#                         0 <= adj_y < GRID_SIZE[1] and
#                         0 <= adj_z < GRID_SIZE[2]):
#                     continue
#
#                 # Can't go below z=0
#                 if adj_z < 0:
#                     continue
#
#                 # Can't be inside a built voxel
#                 if adj_pos in built_voxels:
#                     continue
#
#                 # Must be walkable (z=0 or adjacent to a built voxel)
#                 if adj_z == 0:
#                     target_neighbors.append(adj_pos)
#                 else:
#                     # Check if adjacent to any built voxel
#                     is_supported = any(
#                         (adj_x + dx2, adj_y + dy2, adj_z + dz2) in built_voxels
#                         for dx2, dy2, dz2 in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
#                     )
#                     if is_supported:
#                         target_neighbors.append(adj_pos)
#
#     if not target_neighbors:
#         return []  # No valid position near target
#
#     open_set = [(0, start_pos)]
#     position_came_from = {}
#     position_cost_from_start = {start_pos: 0}
#     closed_positions = set()
#
#     while open_set:
#         _, current_pos = heapq.heappop(open_set)
#
#         if current_pos in closed_positions:
#             continue
#         closed_positions.add(current_pos)
#
#         # Check if we reached any valid position near target
#         if current_pos in target_neighbors:
#             reconstructed_path = []
#             while current_pos in position_came_from:
#                 reconstructed_path.append(current_pos)
#                 current_pos = position_came_from[current_pos]
#             return list(reversed(reconstructed_path))
#
#         for neighbor_pos in neighbors(current_pos):
#             if neighbor_pos in closed_positions:
#                 continue
#
#             neighbor_x, neighbor_y, neighbor_z = neighbor_pos
#
#             # Check z bounds
#             if neighbor_z < 0:
#                 continue
#
#             if not (0 <= neighbor_x < GRID_SIZE[0] and
#                     0 <= neighbor_y < GRID_SIZE[1] and
#                     0 <= neighbor_z < GRID_SIZE[2]):
#                 continue
#
#             # Can't walk inside built voxels
#             if neighbor_pos in built_voxels:
#                 continue
#
#             # Walkability: z=0 or adjacent to built voxel
#             if neighbor_z == 0:
#                 is_walkable = True
#             else:
#                 is_walkable = any(
#                     (neighbor_x + dx, neighbor_y + dy, neighbor_z + dz) in built_voxels
#                     for dx, dy, dz in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
#                 )
#
#             if not is_walkable:
#                 continue
#
#             cost = position_cost_from_start[current_pos] + 1
#
#             if neighbor_pos not in position_cost_from_start or cost < position_cost_from_start[neighbor_pos]:
#                 position_cost_from_start[neighbor_pos] = cost
#                 # Use closest target neighbor for heuristic
#                 f_score = cost + min(heuristic(neighbor_pos, t) for t in target_neighbors)
#                 heapq.heappush(open_set, (f_score, neighbor_pos))
#                 position_came_from[neighbor_pos] = current_pos
#
#     return []