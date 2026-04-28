import networkx as nx
from collections import defaultdict

from src.config import GRID_SIZE
from src.structure.voxel_grid import VoxelGrid
from src.utils import neighbors_2d_horizontal

def connected_components(voxels: set[tuple[int, int, int]]) -> list[set[tuple[int, int, int]]]:
    """
    Find all connected components (groups of adjacent voxels) in a set of voxels.

    Uses DFS to group voxels that are horizontally connected.
    Two voxels are connected if they share a face in the x-y plane.

    Args:
        voxels: Set of 3D voxel coordinates.

    Returns:
        List of sets, where each set contains voxels forming one connected component.
    """

    visited = set()
    components = []

    for voxel in voxels:
        if voxel in visited:
            continue

        # Start DFS from this voxel
        dfs_stack = [voxel]
        current_component = []
        visited.add(voxel)

        while dfs_stack:
            current_voxel = dfs_stack.pop()
            current_component.append(current_voxel)

            # Explore all horizontal neighbors
            for neighbor in neighbors_2d_horizontal(current_voxel):
                if neighbor in voxels and neighbor not in visited:
                    visited.add(neighbor)
                    dfs_stack.append(neighbor)

        components.append(current_component)

    return components

def build_graph(voxel_grid: VoxelGrid) -> nx.DiGraph:
    """
    Build a dependency graph from the target structure following the ARMADAS algorithm.

    Creates a slice graph by partitioning voxels into layers by z-coordinate,
    then finding connected components within each layer. Edges represent
    vertical dependencies (voxel directly above another). Maps each voxel
    to its corresponding component in the dependency graph.

    Args:
        voxel_grid: VoxelGrid object with a 'target' set of voxels to be built.

    Returns:
        NetworkX directed graph where components represent connected components
        and edges represent vertical dependencies.
    """

    dependency_graph = nx.DiGraph()

    # Group voxels by z-coordinate (height layer)
    voxels_by_layer = defaultdict(list)

    for voxel in voxel_grid.target:
        height = voxel[2]

        voxels_by_layer[height].append(voxel)

    # Map each voxel to its component ID in the dependency graph
    voxel_to_component_id_map = {}
    next_component_id = 0

    for layer in voxels_by_layer:
        components_in_layer = connected_components(set(voxels_by_layer[layer]))

        # Create one component for each connected component
        for component_voxels in components_in_layer:
            dependency_graph.add_node(next_component_id, voxels=set(component_voxels))

            # Record mapping from each voxel to its component
            for voxel in component_voxels:
                voxel_to_component_id_map[voxel] = next_component_id

            next_component_id += 1

    # Create edges for vertical dependencies
    for target_voxel in voxel_grid.target:
        voxel_below = (target_voxel[0], target_voxel[1], target_voxel[2] - 1)

        # If there's a voxel directly below, create dependency edge
        if voxel_below in voxel_grid.target:
            component_below = voxel_to_component_id_map[voxel_below]
            component_target = voxel_to_component_id_map[target_voxel]

            if component_below != component_target:
                dependency_graph.add_edge(component_below, component_target)

    return dependency_graph

def contract_graph(dependency_graph: nx.DiGraph) -> nx.DiGraph:
    """
    Contract the dependency graph to merge linear chains of components.

    Repeatedly merges components where one component has exactly one successor
    and that successor has exactly one predecessor (forming a "pillar"
    with no branching). This increases parallelism by combining dependencies
    that must be sequential anyway.

    Args:
        dependency_graph: NetworkX directed graph to contract.

    Returns:
        Contracted dependency graph with linear chains merged into single components.
    """

    graph_changed = True

    while graph_changed:
        graph_changed = False

        # Iterate through all components in the graph
        for component_to_check in list(dependency_graph.nodes):
            successor_components = list(dependency_graph.successors(component_to_check))

            # Check if this component has exactly one successor
            if len(successor_components) == 1:
                successor_component = successor_components[0]

                predecessor_components = list(dependency_graph.predecessors(successor_component))

                # Check if that successor has exactly one predecessor (this component)
                if len(predecessor_components) == 1:
                    # Merge these two components into one
                    new_merged_component_id = max(dependency_graph.nodes) + 1

                    merged_voxels = (dependency_graph.nodes[component_to_check]['voxels']
                                     | dependency_graph.nodes[successor_component]['voxels'])

                    # Add merged component with combined voxels
                    dependency_graph.add_node(new_merged_component_id, voxels=merged_voxels)

                    # Add edges from predecessors of first component to merged component
                    for predecessor in dependency_graph.predecessors(component_to_check):
                        dependency_graph.add_edge(predecessor, new_merged_component_id)

                    # Add edges from merged component to successors of second component
                    for successor in dependency_graph.successors(successor_component):
                        dependency_graph.add_edge(new_merged_component_id, successor)

                    # Remove the two original components
                    dependency_graph.remove_node(component_to_check)
                    dependency_graph.remove_node(successor_component)

                    # Restart iteration since graph structure changed
                    graph_changed = True

                    break

    return dependency_graph

def no_tight_build_violated(voxel: tuple[int, int, int],
                            occupied: set[tuple[int, int, int]]) -> bool:
    """
    Returns true if placing this voxel would violate the no-tight-build constraint given the set of already-occupied
    positions.
    """

    x, y, z = voxel

    axis_pairs = [
        ((x + 1, y, z), (x - 1, y, z)),
        ((x, y + 1, z), (x, y - 1, z)),
        ((x, y, z + 1), (x, y, z - 1)),
    ]

    for a, b in axis_pairs:
        if a in occupied and b in occupied:
            return True

    return False

def find_scaffolding_column_position(target_component_voxels,
                                     voxel_grid: VoxelGrid,
                                     existing_scaffolding: set[tuple[int, int, int]]) -> tuple[int, int] | None:
    max_x, max_y, max_z = GRID_SIZE

    min_z = min(v[2] for v in target_component_voxels)

    footprint = set((v[0], v[1]) for v in target_component_voxels)

    existing_scaffold_footprint = set((v[0], v[1]) for v in existing_scaffolding)

    # Include ALL target voxels plus existing scaffold in occupied check
    occupied = voxel_grid.target | existing_scaffolding

    cx_mean = sum(fx for fx, fy in footprint) / len(footprint)
    cy_mean = sum(fy for fx, fy in footprint) / len(footprint)

    max_radius = max(max_x, max_y)

    for radius in range(0, max_radius + 1):
        ring = set()

        for fx in range(max_x):
            for fy in range(max_y):
                if abs(fx - round(cx_mean)) == radius or abs(fy - round(cy_mean)) == radius:
                    if (fx, fy) not in footprint and (fx, fy) not in existing_scaffold_footprint:
                        ring.add((fx, fy))

        candidates = sorted(ring,
            key=lambda p: abs(p[0] - cx_mean) + abs(p[1] - cy_mean))

        for (cx, cy) in candidates:
            column_voxels = [(cx, cy, z) for z in range(min_z)]

            valid = True

            tentative = set(occupied)

            for voxel in column_voxels:
                if voxel in tentative:
                    valid = False

                    break

                if no_tight_build_violated(voxel, tentative):
                    valid = False

                    break

                tentative.add(voxel)

            if valid:
                return (cx, cy)

    return None

def add_scaffolding(dependency_graph: nx.DiGraph, voxel_grid: VoxelGrid):
    """
    Augment the dependency graph with scaffold build and teardown nodes for the top k components that maximize:
        1. predecessor-chain distance in the dependency graph
        2. voxel count as a tiebreaker

    For each target component whose lowest voxel is above z = 0 and which has no ground-anchored predecessor path, a
    vertical scaffold column is generated.
        - ScaffoldBuild:    ground-anchored, must complete before the target component.
        - ScaffoldTear:     must complete after the target component (reverse build order).

    The scaffold voxels are registered in voxel_grid.scaffold so that can_build and the simulator treat them as valid,
    temporary build locations.

    Args:
         dependency_graph: The contracted dependency graph.
         voxel_grid: The voxel grid being built in.

        Returns:
            The augmented dependency graph.
    """

    target_node = select_scaffold_target(dependency_graph)

    if target_node is None:
        print("No eligible component found for scaffolding.")

        return dependency_graph

    target_voxels = dependency_graph.nodes[target_node]["voxels"]

    min_z = min(voxel[2] for voxel in target_voxels)

    existing_scaffold: set[tuple[int, int, int]] = set()

    next_node_id = max(dependency_graph.nodes) + 1

    column_xy = find_scaffolding_column_position(target_voxels, voxel_grid, existing_scaffold)

    if column_xy is None:
        print(f"Warning: Could not find valid scaffold column position for component {target_node}")

        return dependency_graph

    cx, cy = column_xy

    vertical_column = [(cx, cy, z) for z in range(min_z + 2)]

    tentative_existing = set(existing_scaffold)

    for voxel in vertical_column:
        tentative_existing.add(voxel)

    arm_voxels = find_scaffold_connection_path(
        target_component_voxels=target_voxels,
        column_xy=column_xy,
        min_z=min_z,
        voxel_grid=voxel_grid,
        existing_scaffolding=tentative_existing
    )

    if arm_voxels is None:
        print(f"Warning: Could not find valid horizontal scaffold arm for component {target_node}.")

        return dependency_graph

    build_order = vertical_column + arm_voxels
    tear_order = list(reversed(build_order))

    for voxel in build_order:
        voxel_grid.scaffold.add(voxel)
        existing_scaffold.add(voxel)

    base_voxel = (cx, cy, 0)

    if no_tight_build_violated(base_voxel, voxel_grid.target):
        print(f"Warning: Base voxel {base_voxel} violates tight-build rules.")

        return dependency_graph

    ground_roots = set()

    for node in dependency_graph.nodes:
        voxels = dependency_graph.nodes[node]["voxels"]

        if any(voxel[2] == 0 for voxel in voxels):
            ground_roots.add(node)

    anchor_node = None

    for node in ground_roots:
        if base_voxel in dependency_graph.nodes[node]["voxels"]:
            anchor_node = node

            break

    scaffold_build_id = next_node_id

    next_node_id += 1

    dependency_graph.add_node(
        scaffold_build_id,
        voxels=set(build_order),
        is_scaffold=True,
        scaffold_for=target_node,
        scaffold_order=build_order
    )

    scaffold_tear_id = next_node_id

    next_node_id += 1

    dependency_graph.add_node(
        scaffold_tear_id,
        voxels=set(tear_order),
        is_scaffold_teardown=True,
        scaffold_for=target_node,
        scaffold_order=tear_order
    )

    dependency_graph.add_edge(scaffold_build_id, target_node)
    dependency_graph.add_edge(target_node, scaffold_tear_id)

    if anchor_node is not None:
        dependency_graph.add_edge(anchor_node, scaffold_build_id)

    print(
        f"Scaffold added for component {target_node}: "
        f"column at ({cx}, {cy}), z=0 to z={min_z + 2}. "
        f"BuildNode={scaffold_build_id}, TearNode={scaffold_tear_id}."
    )

    return dependency_graph

def compute_predecessor_depths(dependency_graph: nx.DiGraph) -> dict[int, int]:
    depth_cache = {}

    def depth(node: int) -> int:
        if node in depth_cache:
            return depth_cache[node]

        preds = list(dependency_graph.predecessors(node))
        if not preds:
            depth_cache[node] = 0

        else:
            depth_cache[node] = 1 + max(depth(pred) for pred in preds)

        return depth_cache[node]

    for node in dependency_graph.nodes:
        depth(node)

    return depth_cache

def select_scaffold_target(dependency_graph: nx.DiGraph) -> int | None:
    eligible_nodes = []

    for node in dependency_graph.nodes:
        voxels = dependency_graph.nodes[node]["voxels"]

        min_z = min(voxel[2] for voxel in voxels)

        if min_z > 0:
            eligible_nodes.append(node)

    if not eligible_nodes:
        return None

    depths = compute_predecessor_depths(dependency_graph)

    return max(
        eligible_nodes,
        key=lambda n: (
            depths[n],
            len(dependency_graph.nodes[n]["voxels"])
        )
    )

def horizontal_path(start_xy: tuple[int, int], end_xy: tuple[int, int], z: int) -> list[tuple[int, int, int]]:
    x0, y0 = start_xy
    x1, y1 = end_xy

    path = []

    x, y = x0, y0

    while x != x1:
        x += 1 if x1 > x else -1

        path.append((x, y, z))

    while y != y1:
        y += 1 if y1 > y else -1

        path.append((x, y, z))

    return path

def find_scaffold_connection_path(
    target_component_voxels,
    column_xy: tuple[int, int],
    min_z: int,
    voxel_grid: VoxelGrid,
    existing_scaffolding: set[tuple[int, int, int]]
) -> list[tuple[int, int, int]] | None:
    """
    Return horizontal arm voxels that connect the top of the scaffold column
    to the target footprint at height min_z - 1, if needed.
    """

    if min_z <= 0:
        return []

    footprint = set((x, y) for x, y, _ in target_component_voxels)

    cx, cy = column_xy

    if (cx, cy) in footprint:
        return []

    occupied = voxel_grid.target | existing_scaffolding

    arm_z = min_z + 1

    candidate_targets = sorted(
        footprint,
        key=lambda p: abs(p[0] - cx) + abs(p[1] - cy)
    )

    for tx, ty in candidate_targets:
        path = horizontal_path((cx, cy), (tx, ty), arm_z)

        valid = True

        tentative = set(occupied)

        for voxel in path:
            if voxel in tentative:
                valid = False

                break

            if no_tight_build_violated(voxel, tentative):
                valid = False

                break

            tentative.add(voxel)

        if valid:
            return path

    return None