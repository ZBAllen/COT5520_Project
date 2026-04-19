import networkx as nx

from src.utils import bfs_component_voxels_from_starting_voxels_set, neighbors_3d

def order_component_voxels(voxels_in_component: set[tuple[int, int, int]],
                           dependency_graph: nx.DiGraph,
                           current_component_id: int) -> list[tuple[int, int, int]]:
    """
    Order voxels using BFS, then fix no-tight-building violations by flipping edges.
    Follows ARMADAS algorithm from the research paper.
    """

    predecessor_voxels = set()

    for predecessor_component_id in dependency_graph.predecessors(current_component_id):
        predecessor_voxels.update(dependency_graph.nodes[predecessor_component_id]['voxels'])

    # Find starting voxels adjacent to predecessors
    starting_voxels = []
    for voxel in voxels_in_component:
        neighbors = neighbors_3d(voxel)

        if any(neighbor in predecessor_voxels for neighbor in neighbors):
            starting_voxels.append(voxel)

    if not starting_voxels:
        starting_voxels = [v for v in voxels_in_component if v[2] == 0]

    if not starting_voxels:
        starting_voxels = [next(iter(voxels_in_component))]

    # Step 1: BFS to create initial ordering graph [1]
    bfs_order = bfs_component_voxels_from_starting_voxels_set(starting_voxels, voxels_in_component)

    # Create ordering graph: edge from earlier to later voxel in BFS order
    ordering_graph = nx.DiGraph()

    for voxel in bfs_order:
        ordering_graph.add_node(voxel)

    bfs_position = {voxel: i for i, voxel in enumerate(bfs_order)}

    for i, voxel in enumerate(bfs_order):
        for neighbor in neighbors_3d(voxel):
            if neighbor in voxels_in_component:
                if bfs_position[neighbor] > i:
                    ordering_graph.add_edge(voxel, neighbor)

    # Step 2: Fix no-tight-building constraint violations [1]
    # Repeatedly find violating voxels and flip their incoming edges
    max_iterations = len(voxels_in_component) * 10
    iteration = 0

    while iteration < max_iterations:
        iteration += 1

        # Find all voxels that violate the constraint
        violations = []

        find_violations(voxels_in_component, ordering_graph)

        if not violations:
            break

        # Pick violation with minimum (z, y, x) for determinism [1]
        violating_voxel = min(violations, key=lambda v: (v[2], v[1], v[0]))

        neighbors = neighbors_3d(violating_voxel)

        # Find axis with violation and flip edges [1]
        axis_pairs = [(0, 1), (2, 3), (4, 5)]

        for idx1, idx2 in axis_pairs:
            n1, n2 = neighbors[idx1], neighbors[idx2]

            if n1 in voxels_in_component and n2 in voxels_in_component:
                if ordering_graph.has_edge(n1, violating_voxel) and ordering_graph.has_edge(n2, violating_voxel):
                    # Flip: remove incoming edges, add outgoing edges [1]
                    ordering_graph.remove_edge(n1, violating_voxel)
                    ordering_graph.remove_edge(n2, violating_voxel)

                    ordering_graph.add_edge(violating_voxel, n1)
                    ordering_graph.add_edge(violating_voxel, n2)

                    break

    # Topological sort to get final ordering
    try:
        final_ordering = list(nx.topological_sort(ordering_graph))

    except nx.NetworkXUnfeasible:
        # If cycle exists, extract DAG
        dag = nx.DiGraph(ordering_graph)

        while True:
            try:
                cycle = nx.find_cycle(dag)

                dag.remove_edge(cycle[0][0], cycle[0][1])

            except nx.NetworkXNoCycle:
                break

        final_ordering = list(nx.topological_sort(dag))

    return final_ordering

def find_violations(voxels_in_component: set[tuple[int, int, int]], ordering_graph: nx.DiGraph):
    violations = []

    neighbors_map = {voxel: neighbors_3d(voxel) for voxel in voxels_in_component}

    axis_pairs = [(0, 1), (2, 3), (4, 5)]

    for voxel in voxels_in_component:
        neighbors = neighbors_map[voxel]

        for idx1, idx2 in axis_pairs:
            n1, n2 = neighbors[idx1], neighbors[idx2]

            if n1 in voxels_in_component and n2 in voxels_in_component and ordering_graph.has_edge(n1, voxel) and ordering_graph.has_edge(n2, voxel):
                violations.append(voxel)

                break

    return violations