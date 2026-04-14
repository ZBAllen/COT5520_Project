import networkx as nx
from collections import defaultdict

from src.structure.voxel_grid import VoxelGrid

def neighbors_2d_horizontal(voxel: tuple[int, int, int]) -> list[tuple[int, int, int]]:
    x, y, z = voxel

    directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]

    return [(x + dx, y + dy, z) for dx, dy in directions]

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