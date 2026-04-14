import networkx as nx
from collections import deque

from src.pathfinding.a_star import corner_neighbors, neighbors


# def order_component_voxels(voxels_in_component: set[tuple[int, int, int]],
#                            dependency_graph: nx.DiGraph,
#                            current_component_id: int) -> list[tuple[int, int, int]]:
#     """
#     Order voxels within a component using BFS from predecessor neighbors,
#     then fix no-tight-building constraint violations.
#
#     Creates an ordering of voxels that respects the no-tight-building constraint,
#     which states that a voxel cannot pass through a unit-wide gap between two
#     other voxels. This is enforced by ensuring voxels are placed in an order
#     where each voxel has support from previously placed voxels.
#
#     Args:
#         voxels_in_component: Set of 3D voxel coordinates within this component.
#         dependency_graph: NetworkX directed graph of component dependencies.
#         current_component_id: ID of the current component being ordered.
#
#     Returns:
#         List of voxels coordinates in valid build order for this component.
#     """
#
#     # Find voxels in predecessor components that neighbor voxels in current component
#     starting_voxels = []
#     predecessor_voxels = set()
#
#     for predecessor_component_id in dependency_graph.predecessors(current_component_id):
#         predecessor_voxels.update(dependency_graph.nodes[predecessor_component_id]['voxels'])
#
#     # Identify voxels that are adjacent to predecessor voxels (good starting points)
#     for voxel in voxels_in_component:
#         x, y, z = voxel
#
#         neighbors = [
#             (x + 1, y, z), (x - 1, y, z),
#             (x, y + 1, z), (x, y - 1, z),
#             (x, y, z + 1), (x, y, z - 1)
#         ]
#
#         if any(neighbor in predecessor_voxels for neighbor in neighbors):
#             starting_voxels.append(voxel)
#
#     # Fallback: use ground-level voxels as starting points
#     if not starting_voxels:
#         starting_voxels = [voxel for voxel in voxels_in_component if voxel[2] == 0]
#
#     # Final fallback: pick any voxel if no ground voxels exist
#     if not starting_voxels:
#         starting_voxels = [next(iter(voxels_in_component))]
#
#     # BFS from starting voxels to create initial ordering
#     visited_voxels = set()
#     initial_ordering = []
#     bfs_queue = deque(starting_voxels)
#
#     while bfs_queue:
#         voxel = bfs_queue.popleft()
#
#         if voxel in visited_voxels or voxel not in voxels_in_component:
#             continue
#
#         visited_voxels.add(voxel)
#         initial_ordering.append(voxel)
#
#         x, y, z = voxel
#
#         neighbors = [
#             (x + 1, y, z), (x - 1, y, z),
#             (x, y + 1, z), (x, y - 1, z),
#             (x, y, z + 1), (x, y, z - 1)
#         ]
#
#         for neighbor in neighbors:
#             if neighbor in voxels_in_component and neighbor not in visited_voxels:
#                 bfs_queue.append(neighbor)
#
#     # Fix no-tight-building constraint violations by enforcing support requirements
#     # Repeatedly place voxels that support, ensuring the constraint is never violated
#     final_ordering = []
#     unordered_voxels = set(initial_ordering)
#     built_voxels = set(predecessor_voxels)  # Start with predecessor voxels as "built"
#
#     while unordered_voxels:
#         # Find voxels that can be placed without violating constraint
#         placeable_voxels = []
#
#         for voxel in unordered_voxels:
#             x, y, z = voxel
#
#             # Ground voxels (z=0) are always placeable
#             if z == 0:
#                 placeable_voxels.append(voxel)
#
#                 continue
#
#             # Check if supported by neighbors without violating no-tight-build constraint
#             neighbors = [
#                 (x + 1, y, z), (x - 1, y, z),
#                 (x, y + 1, z), (x, y - 1, z),
#                 (x, y, z + 1), (x, y, z - 1)
#             ]
#
#             has_neighbor = False
#
#             for n in neighbors:
#                 if n in built_voxels:
#                     has_neighbor = True
#
#             if neighbors[0] in built_voxels and neighbors[1] in built_voxels:
#                 continue
#
#             if neighbors[2] in built_voxels and neighbors[3] in built_voxels:
#                 continue
#
#             if neighbors[4] in built_voxels and neighbors[5] in built_voxels:
#                 continue
#
#             if not has_neighbor:
#                 continue
#
#             placeable_voxels.append(voxel)
#
#         # If no voxels can be placed, we have a deadlock: pick voxel with lowest coordinates
#         # as tiebreaker (z-coordinate first, then y, then x) as per the ARMADAS paper.
#         if not placeable_voxels:
#             voxel_with_min_coords = min(unordered_voxels, key=lambda v: (v[2], v[1], v[0]))
#
#             placeable_voxels = [voxel_with_min_coords]
#
#         # Add all placeable voxels to final ordering and mark as built
#         for voxel in placeable_voxels:
#             final_ordering.append(voxel)
#             built_voxels.add(voxel)
#             unordered_voxels.remove(voxel)
#
#     return final_ordering

# def order_component_voxels(voxels_in_component: set[tuple[int, int, int]],
#                            dependency_graph: nx.DiGraph,
#                            current_component_id: int) -> list[tuple[int, int, int]]:
#     """
#     Order voxels within a component using BFS from predecessor neighbors,
#     then fix no-tight-building constraint violations by flipping edges.
#
#     Args:
#         voxels_in_component: Set of 3D voxel coordinates within this component.
#         dependency_graph: NetworkX directed graph of component dependencies.
#         current_component_id: ID of the current component being ordered.
#
#     Returns:
#         List of voxel coordinates in valid build order for this component.
#     """
#     # Find voxels in predecessor components
#     predecessor_voxels = set()
#     for predecessor_component_id in dependency_graph.predecessors(current_component_id):
#         predecessor_voxels.update(dependency_graph.nodes[predecessor_component_id]['voxels'])
#
#     # Find starting voxels adjacent to predecessors
#     starting_voxels = []
#     for voxel in voxels_in_component:
#         x, y, z = voxel
#         neighbors = [(x + 1, y, z), (x - 1, y, z), (x, y + 1, z),
#                      (x, y - 1, z), (x, y, z + 1), (x, y, z - 1)]
#         if any(neighbor in predecessor_voxels for neighbor in neighbors):
#             starting_voxels.append(voxel)
#
#     # Fallbacks for starting voxels
#     if not starting_voxels:
#         starting_voxels = [v for v in voxels_in_component if v[2] == 0]
#     if not starting_voxels:
#         starting_voxels = [next(iter(voxels_in_component))]
#
#     # BFS to create initial ordering graph
#     visited = set()
#     bfs_order = []
#     queue = deque(starting_voxels)
#
#     while queue:
#         voxel = queue.popleft()
#         if voxel in visited or voxel not in voxels_in_component:
#             continue
#         visited.add(voxel)
#         bfs_order.append(voxel)
#
#         x, y, z = voxel
#         neighbors = [(x + 1, y, z), (x - 1, y, z), (x, y + 1, z),
#                      (x, y - 1, z), (x, y, z + 1), (x, y, z - 1)]
#         for neighbor in neighbors:
#             if neighbor in voxels_in_component and neighbor not in visited:
#                 queue.append(neighbor)
#
#     # Create initial ordering graph: edge from earlier to later voxel
#     ordering_graph = nx.DiGraph()
#     for voxel in bfs_order:
#         ordering_graph.add_node(voxel)
#
#     for i, voxel in enumerate(bfs_order):
#         x, y, z = voxel
#         neighbors = [(x + 1, y, z), (x - 1, y, z), (x, y + 1, z),
#                      (x, y - 1, z), (x, y, z + 1), (x, y, z - 1)]
#         for neighbor in neighbors:
#             if neighbor in voxels_in_component:
#                 neighbor_idx = bfs_order.index(neighbor)
#                 if neighbor_idx > i:
#                     ordering_graph.add_edge(voxel, neighbor)
#
#     # Fix no-tight-building violations by flipping edges
#     max_iterations = len(voxels_in_component) * 10
#     iteration_count = 0
#
#     while iteration_count < max_iterations:
#         iteration_count += 1
#
#         # Check for cycles
#         try:
#             nx.find_cycle(ordering_graph)
#             # If we get here, there's a cycle. Remove it by breaking the most recent flip.
#             # For now, we'll just break out and use topological sort on the DAG we can extract
#             break
#         except nx.NetworkXNoCycle:
#             # No cycle, continue
#             pass
#
#         # Find all violations
#         violations = []
#         for voxel in voxels_in_component:
#             x, y, z = voxel
#             neighbors = [(x + 1, y, z), (x - 1, y, z), (x, y + 1, z),
#                          (x, y - 1, z), (x, y, z + 1), (x, y, z - 1)]
#
#             # Check each axis for tight gaps
#             axis_pairs = [
#                 (neighbors[0], neighbors[1]),  # x-axis
#                 (neighbors[2], neighbors[3]),  # y-axis
#                 (neighbors[4], neighbors[5])  # z-axis
#             ]
#
#             for n1, n2 in axis_pairs:
#                 if n1 in voxels_in_component and n2 in voxels_in_component:
#                     # voxel is in the middle of n1 and n2
#                     # Violation: voxel depends on both n1 and n2
#                     has_edge_from_n1 = ordering_graph.has_edge(n1, voxel)
#                     has_edge_from_n2 = ordering_graph.has_edge(n2, voxel)
#
#                     if has_edge_from_n1 and has_edge_from_n2:
#                         violations.append(voxel)
#                         break
#
#         if not violations:
#             break
#
#         # Pick violation with minimum (z, y, x)
#         violating_voxel = min(violations, key=lambda v: (v[2], v[1], v[0]))
#
#         x, y, z = violating_voxel
#         neighbors = [(x + 1, y, z), (x - 1, y, z), (x, y + 1, z),
#                      (x, y - 1, z), (x, y, z + 1), (x, y, z - 1)]
#
#         # Find the axis where violation occurs and flip edges
#         axis_pairs = [
#             (neighbors[0], neighbors[1]),  # x-axis
#             (neighbors[2], neighbors[3]),  # y-axis
#             (neighbors[4], neighbors[5])  # z-axis
#         ]
#
#         for n1, n2 in axis_pairs:
#             if n1 in voxels_in_component and n2 in voxels_in_component:
#                 if ordering_graph.has_edge(n1, violating_voxel) and \
#                         ordering_graph.has_edge(n2, violating_voxel):
#                     # Flip edges: remove incoming, add outgoing
#                     ordering_graph.remove_edge(n1, violating_voxel)
#                     ordering_graph.remove_edge(n2, violating_voxel)
#                     ordering_graph.add_edge(violating_voxel, n1)
#                     ordering_graph.add_edge(violating_voxel, n2)
#                     break
#
#     # Remove cycles if they exist by extracting largest DAG
#     try:
#         final_ordering = list(nx.topological_sort(ordering_graph))
#     except nx.NetworkXUnfeasible:
#         # Extract a DAG from the cyclic graph
#         largest_dag = nx.DiGraph(ordering_graph)
#         while True:
#             try:
#                 nx.find_cycle(largest_dag)
#                 # Remove edge from cycle
#                 cycle = nx.find_cycle(largest_dag)
#                 largest_dag.remove_edge(cycle[0][0], cycle[0][1])
#             except nx.NetworkXNoCycle:
#                 break
#         final_ordering = list(nx.topological_sort(largest_dag))
#
#     return final_ordering


def order_component_voxels(voxels_in_component: set[tuple[int, int, int]],
                           dependency_graph: nx.DiGraph,
                           current_component_id: int) -> list[tuple[int, int, int]]:
    """
    Order voxels within a component using BFS from predecessor neighbors,
    then fix no-tight-building constraint violations by flipping edges.

    Args:
        voxels_in_component: Set of 3D voxel coordinates within this component.
        dependency_graph: NetworkX directed graph of component dependencies.
        current_component_id: ID of the current component being ordered.

    Returns:
        List of voxel coordinates in valid build order for this component.
    """
    # Find voxels in predecessor components
    predecessor_voxels = set()
    for predecessor_component_id in dependency_graph.predecessors(current_component_id):
        predecessor_voxels.update(dependency_graph.nodes[predecessor_component_id]['voxels'])

    # Find starting voxels adjacent to predecessors
    starting_voxels = []
    for voxel in voxels_in_component:
        x, y, z = voxel
        neighbors = [(x + 1, y, z), (x - 1, y, z), (x, y + 1, z),
                     (x, y - 1, z), (x, y, z + 1), (x, y, z - 1)]
        if any(neighbor in predecessor_voxels for neighbor in neighbors):
            starting_voxels.append(voxel)

    # Fallbacks for starting voxels
    if not starting_voxels:
        starting_voxels = [v for v in voxels_in_component if v[2] == 0]
    if not starting_voxels:
        starting_voxels = [next(iter(voxels_in_component))]

    # BFS to create initial ordering graph
    visited = set()
    bfs_order = []
    queue = deque(starting_voxels)

    while queue:
        voxel = queue.popleft()
        if voxel in visited or voxel not in voxels_in_component:
            continue
        visited.add(voxel)
        bfs_order.append(voxel)

        x, y, z = voxel
        neighbors = [(x + 1, y, z), (x - 1, y, z), (x, y + 1, z),
                     (x, y - 1, z), (x, y, z + 1), (x, y, z - 1)]
        for neighbor in neighbors:
            if neighbor in voxels_in_component and neighbor not in visited:
                queue.append(neighbor)

    # Create initial ordering graph: edge from earlier to later voxel
    ordering_graph = nx.DiGraph()
    for voxel in bfs_order:
        ordering_graph.add_node(voxel)

    for i, voxel in enumerate(bfs_order):
        x, y, z = voxel
        neighbors = [(x + 1, y, z), (x - 1, y, z), (x, y + 1, z),
                     (x, y - 1, z), (x, y, z + 1), (x, y, z - 1)]
        for neighbor in neighbors:
            if neighbor in voxels_in_component:
                neighbor_idx = bfs_order.index(neighbor)
                if neighbor_idx > i:
                    ordering_graph.add_edge(voxel, neighbor)

    # Fix no-tight-building violations by flipping edges
    max_iterations = len(voxels_in_component) * 10
    iteration_count = 0

    while iteration_count < max_iterations:
        iteration_count += 1

        # Check for cycles
        try:
            nx.find_cycle(ordering_graph)
            # If we get here, there's a cycle. Break out and handle it later.
            break
        except nx.NetworkXNoCycle:
            # No cycle, continue
            pass

        # Find all violations
        violations = []
        for voxel in voxels_in_component:
            x, y, z = voxel
            neighbors = [(x + 1, y, z), (x - 1, y, z), (x, y + 1, z),
                         (x, y - 1, z), (x, y, z + 1), (x, y, z - 1)]

            # Check each axis for tight gaps (three in a row)
            axis_pairs = [
                (0, 1),  # x-axis: neighbors[0] and neighbors[1]
                (2, 3),  # y-axis: neighbors[2] and neighbors[3]
                (4, 5)  # z-axis: neighbors[4] and neighbors[5]
            ]

            for idx1, idx2 in axis_pairs:
                n1, n2 = neighbors[idx1], neighbors[idx2]
                if n1 in voxels_in_component and n2 in voxels_in_component:
                    # voxel is in the middle; check if it has edges TO both neighbors
                    has_edge_to_n1 = ordering_graph.has_edge(voxel, n1)
                    has_edge_to_n2 = ordering_graph.has_edge(voxel, n2)

                    # Violation: voxel has NO edge to at least one neighbor
                    if not has_edge_to_n1 or not has_edge_to_n2:
                        violations.append(voxel)
                        break

        if not violations:
            break

        # Pick violation with minimum (z, y, x)
        violating_voxel = min(violations, key=lambda v: (v[2], v[1], v[0]))

        x, y, z = violating_voxel
        neighbors = [(x + 1, y, z), (x - 1, y, z), (x, y + 1, z),
                     (x, y - 1, z), (x, y, z + 1), (x, y, z - 1)]

        # Find the axis where violation occurs and fix it
        axis_pairs = [
            (0, 1),  # x-axis
            (2, 3),  # y-axis
            (4, 5)  # z-axis
        ]

        for idx1, idx2 in axis_pairs:
            n1, n2 = neighbors[idx1], neighbors[idx2]
            if n1 in voxels_in_component and n2 in voxels_in_component:
                has_edge_to_n1 = ordering_graph.has_edge(violating_voxel, n1)
                has_edge_to_n2 = ordering_graph.has_edge(violating_voxel, n2)

                if not has_edge_to_n1 or not has_edge_to_n2:
                    # Ensure voxel → n1 and voxel → n2
                    if not has_edge_to_n1:
                        if ordering_graph.has_edge(n1, violating_voxel):
                            ordering_graph.remove_edge(n1, violating_voxel)
                        ordering_graph.add_edge(violating_voxel, n1)

                    if not has_edge_to_n2:
                        if ordering_graph.has_edge(n2, violating_voxel):
                            ordering_graph.remove_edge(n2, violating_voxel)
                        ordering_graph.add_edge(violating_voxel, n2)
                    break

    # Remove cycles if they exist by extracting largest DAG
    try:
        final_ordering = list(nx.topological_sort(ordering_graph))
    except nx.NetworkXUnfeasible:
        # Extract a DAG from the cyclic graph
        largest_dag = nx.DiGraph(ordering_graph)
        while True:
            try:
                nx.find_cycle(largest_dag)
                # Remove edge from cycle
                cycle = nx.find_cycle(largest_dag)
                largest_dag.remove_edge(cycle[0][0], cycle[0][1])
            except nx.NetworkXNoCycle:
                break
        final_ordering = list(nx.topological_sort(largest_dag))

    # Ensure first voxel connects to predecessor if predecessors exist
    if predecessor_voxels:
        first_voxel = final_ordering[0]
        x, y, z = first_voxel
        neighbors = [(x + 1, y, z), (x - 1, y, z), (x, y + 1, z),
                     (x, y - 1, z), (x, y, z + 1), (x, y, z - 1)]

        if not any(neighbor in predecessor_voxels for neighbor in neighbors):
            # Reorder: find first connected voxel and move it to front
            for i, voxel in enumerate(final_ordering):
                x, y, z = voxel
                neighbors = [(x + 1, y, z), (x - 1, y, z), (x, y + 1, z),
                             (x, y - 1, z), (x, y, z + 1), (x, y, z - 1)]
                if any(neighbor in predecessor_voxels for neighbor in neighbors):
                    final_ordering[0], final_ordering[i] = final_ordering[i], final_ordering[0]
                    break

    return final_ordering

# def order_component_voxels(voxels_in_component: set[tuple[int, int, int]],
#                            previous_component_voxel_ordering: list[tuple[int, int, int]] = None) -> list[tuple[int, int, int]]:
#     """
#     Returns a build order for voxels in a component that satisfies the no-tight-build constraint.
#     """
#
#     voxels = list(voxels_in_component)
#
#     def is_valid_addition(voxel, order):
#         """Check if adding this voxel violates the no-tight-build constraint."""
#
#         for dim in range(3):
#             neighbor1 = tuple(voxel[i] + (1 if i == dim else 0) for i in range(3))
#             neighbor2 = tuple(voxel[i] - (1 if i == dim else 0) for i in range(3))
#
#             if neighbor1 in order and neighbor2 in order:
#                 return False
#
#         return True
#
#     def backtrack(order):
#         if len(order) == len(voxels):
#             return order
#
#         for voxel in voxels:
#             if voxel not in order:
#                 order.append(voxel)
#
#                 if is_valid_addition(voxel, set(order)):
#                     intermediate_result = backtrack(order)
#
#                     if intermediate_result is not None:
#                         return intermediate_result
#
#                 order.pop()
#
#         return None
#
#     # Try starting from each voxel that could be reachable from a previous component
#     for start_voxel in voxels:
#         # print("Trying a new start voxel...")
#         is_reachable = False
#
#         if not previous_component_voxel_ordering:
#             is_reachable = True
#
#         else:
#             for prev_voxel in reversed(previous_component_voxel_ordering):
#                 if start_voxel in corner_neighbors(prev_voxel, neighbors(prev_voxel), set(previous_component_voxel_ordering)):
#                     is_reachable = True
#
#                     break
#
#         if is_reachable:
#             # print("Getting build order for start voxel...")
#             result = backtrack([start_voxel])
#
#             if result is not None:
#                 return result
#
#     return []