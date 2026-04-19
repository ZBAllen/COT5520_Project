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
    Augment the dependency graph with scaffold build and teardown nodes for components that have no ground-reachable
    path without temporary support.

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

    # Identify ground-rooted components
    ground_roots = set()

    for node in dependency_graph.nodes:
        voxels = dependency_graph.nodes[node]["voxels"]

        if any(voxel[2] == 0 for voxel in voxels):
            ground_roots.add(node)

    if not ground_roots:
        print("Warning: No ground-rooted components found. Cannot add scaffolding.")

        return dependency_graph

    # BFS to find all nodes reachable from any ground root
    reachable = set()

    queue = list(ground_roots)

    while queue:
        current = queue.pop()

        if current in reachable:
            continue

        reachable.add(current)

        for successor in dependency_graph.successors(current):
            if successor not in reachable:
                queue.append(successor)

    # Nodes not reachable from the ground are scaffolding candidates
    scaffold_candidates = [node for node in dependency_graph.nodes if node not in reachable]

    # Sort by minimum z so lower floating components get scaffolding first
    scaffold_candidates.sort(key = lambda n: min(voxel[2] for voxel in dependency_graph.nodes[n]["voxels"]))

    existing_scaffold: set[tuple[int, int, int]] = set()

    next_node_id = max(dependency_graph.nodes) + 1

    for target_node in scaffold_candidates:
        target_voxels = dependency_graph.nodes[target_node]["voxels"]

        min_z = min(voxel[2] for voxel in target_voxels)

        if min_z == 0:
            # Already ground-level, no scaffold needed
            continue

        column_xy = find_scaffolding_column_position(target_voxels, voxel_grid, existing_scaffold)

        if column_xy is None:
            print(f"Warning: Could not find valid scaffold column position for component {target_node}.")

            continue

        cx, cy = column_xy

        # Build column from z = 0 up to min_z
        build_order = [(cx, cy, z) for z in range(min_z + 2)]
        tear_order = list(reversed(build_order))

        # Register scaffold voxels
        for voxel in build_order:
            voxel_grid.scaffold.add(voxel)

            existing_scaffold.add(voxel)

        # The scaffold column starts at z=0 and is self-grounding.
        # Only anchor to an existing ground root if the column's base voxel spatially conflicts with one - otherwise
        # make it a free root.
        base_voxel = (cx, cy, 0)

        # After finding column_xy, verify no tight-build conflict with z=0 target voxels
        if no_tight_build_violated(base_voxel, voxel_grid.target):
            # Try another column position
            continue

        anchor_node = None

        for node in ground_roots:
            if base_voxel in dependency_graph.nodes[node]["voxels"]:
                anchor_node = node

                break

        # Add ScaffoldBuild node
        scaffold_build_id = next_node_id

        next_node_id += 1

        dependency_graph.add_node(
            scaffold_build_id,
            voxels=set(build_order),
            is_scaffold=True,
            scaffold_for=target_node,
            scaffold_order=build_order
        )

        # Add ScaffoldTear node
        scaffold_tear_id = next_node_id

        next_node_id += 1

        dependency_graph.add_node(
            scaffold_tear_id,
            voxels=set(tear_order),
            is_scaffold_teardown=True,
            scaffold_for=target_node,
            scaffold_order=tear_order
        )

        # Remove existing predecessor edges into the target node before adding scaffold dependency, so the scaffold
        # replaces them.
        existing_predecessors = list(dependency_graph.predecessors(target_node))

        for pred in existing_predecessors:
            dependency_graph.remove_edge(pred, target_node)

        # scaffold build -> target (target waits for scaffold)
        dependency_graph.add_edge(scaffold_build_id, target_node)

        # target -> scaffold tear (teardown waits for target completion)
        dependency_graph.add_edge(target_node, scaffold_tear_id)

        # Only anchor to a ground root if the base voxel is owned by one.
        # Otherwise the scaffold build node is a free root (self-grounding at z=0).
        if anchor_node is not None:
            dependency_graph.add_edge(anchor_node, scaffold_build_id)

        print(f"Scaffold added for component {target_node}: "
              f"column at ({cx}, {cy}), z=0 to z={min_z - 1}. "
              f"BuildNode={scaffold_build_id}, TearNode={scaffold_tear_id}.")

    return dependency_graph