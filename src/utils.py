"""This file contains utility functions used by the program."""

def valid_construction_locations(voxel: tuple[int, int, int], built_voxels: set[tuple[int, int, int]]) -> list[tuple[int, int, int]]:
    """
    Returns the locations near the target voxel that are valid for constructing the voxel.

    Args:
        voxel: Voxel to identify valid construction locations.
        built_voxels: The set of voxels that have already been built.

    Returns:
        The locations in the 3x3x3 grid around the target voxel that a robot can build the target voxel from.
    """

    x, y, z = voxel

    directions = [
        (-1, 0, 0), (1, 0, 0), (0, -1, 0), (-1, -1, 0), (1, -1, 0), (0, 1, 0), (-1, 1, 0), (1, 1, 0),
        (0, 0, -1), (-1, 0, -1), (1, 0, -1), (0, -1, -1), (-1, -1, -1), (1, -1, -1), (0, 1, -1),
        (-1, 1, -1), (1, 1, -1), (0, 0, 1), (-1, 0, 1), (1, 0, 1), (0, -1, 1), (-1, -1, 1), (1, -1, 1),
        (0, 1, 1), (-1, 1, 1), (1, 1, 1)
    ]

    positions = [(x + dx, y + dy, z + dz) for dx, dy, dz in directions]

    valid_positions = [position for position in positions if position not in built_voxels]

    return valid_positions

def neighbors_2d_horizontal(voxel: tuple[int, int, int]) -> list[tuple[int, int, int]]:
    """Returns the neighbors of a given voxel in the xy-plane."""

    x, y, z = voxel

    directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]

    return [(x + dx, y + dy, z) for dx, dy in directions]

def neighbors_3d(vertex: tuple[int, int, int]) -> list[tuple[int, int, int]]:
    """Returns the immediate neighbors of the given vertex in xyz-volume."""

    x, y, z = vertex

    directions = [
        (1, 0, 0), (-1, 0, 0), (0, 1, 0),
        (0, -1, 0), (0, 0, 1), (0, 0, -1)
    ]

    return [(x + dx, y + dy, z + dz) for dx, dy, dz in directions]

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