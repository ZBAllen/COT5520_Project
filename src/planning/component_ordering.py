import random
import networkx as nx

from src.utils import bfs_component_voxels_from_starting_voxels_set, neighbors_3d, valid_construction_locations

def order_component_voxels(voxels_in_component: set[tuple[int, int, int]],
                           dependency_graph: nx.DiGraph,
                           current_component_id: int) -> list[tuple[int, int, int]]:
    """
    Order voxels using BFS, then fix no-tight-building violations by flipping edges.
    Follows ARMADAS algorithm from the research paper.
    """

    predecessor_voxels = set()
    normal_predecessor_voxels = set()
    scaffold_predecessor_voxels = set()

    # Get the set of voxels in predecessor components separated by normal structure voxels and scaffolding voxels.
    for predecessor_component_id in dependency_graph.predecessors(current_component_id):
        pred_voxels = dependency_graph.nodes[predecessor_component_id]['voxels']

        predecessor_voxels.update(pred_voxels)

        if dependency_graph.nodes[predecessor_component_id].get("is_scaffold"):
            scaffold_predecessor_voxels.update(pred_voxels)

        else:
            normal_predecessor_voxels.update(pred_voxels)

    # Prefer structural predecessors over scaffold predecessors
    preferred_predecessors = normal_predecessor_voxels if normal_predecessor_voxels else predecessor_voxels
    # preferred_predecessors = scaffold_predecessor_voxels if scaffold_predecessor_voxels else predecessor_voxels

    starting_voxels = []

    # Find the voxels in the component that would work as a valid starting voxel when building.
    for voxel in voxels_in_component:
        neighbors = neighbors_3d(voxel)

        if any(neighbor in preferred_predecessors for neighbor in neighbors):
            starting_voxels.append(voxel)

    # If no voxels are connected to predecessor components, then get all voxels in the component that are on the ground.
    if not starting_voxels:
        starting_voxels = [v for v in voxels_in_component if v[2] == 0]

    # If no valid starting voxels were found, the structure must be invalid.
    if not starting_voxels:
        raise RuntimeError("The structure given has a component that is not attached to any other component nor ground.")

    # Step 1: BFS to create initial ordering graph
    # TODO: This may be the problem. The BFS is performed on the set of starting voxels. This means that the starting voxels are all added first despite them potentially causing violations later when they meet. The violations wouldn't be caught since the edges connecting the violation voxels would be from separate BFS trees and thus would not be flagged as a violation
    bfs_order = bfs_component_voxels_from_starting_voxels_set(starting_voxels, voxels_in_component)

    # Create ordering graph: edge from earlier to later voxel in BFS order
    ordering_graph = nx.DiGraph()

    for voxel in bfs_order:
        ordering_graph.add_node(voxel)

    # Adds edges between voxels if the first voxel is before the second voxel in the BFS ordering.
    bfs_position = {voxel: i for i, voxel in enumerate(bfs_order)}

    for i, voxel in enumerate(bfs_order):
        for neighbor in neighbors_3d(voxel):
            if neighbor in voxels_in_component:
                if bfs_position[neighbor] > i:
                    ordering_graph.add_edge(voxel, neighbor)

    # Step 2: Fix no-tight-building constraint violations
    # Repeatedly find violating voxels and flip their incoming edges
    while True:
        # Find all voxels that violate the constraint
        violations = find_violations(voxels_in_component, ordering_graph)

        if not violations:
            break

        # Pick violation with minimum (z, y, x) for determinism
        violating_voxel = min(violations, key=lambda v: (v[2], v[1], v[0]))

        neighbors = neighbors_3d(violating_voxel)

        # Find axis with violation and flip edges
        axis_pairs = [(0, 1), (2, 3), (4, 5)]

        for idx1, idx2 in axis_pairs:
            n1, n2 = neighbors[idx1], neighbors[idx2]

            if n1 in voxels_in_component and n2 in voxels_in_component:
                if ordering_graph.has_edge(n1, violating_voxel) and ordering_graph.has_edge(n2, violating_voxel):
                    # Flip: remove one of the incoming edges, add a new outgoing edge
                    if random.randint(0, 1) == 0:
                        ordering_graph.remove_edge(n1, violating_voxel)
                        ordering_graph.add_edge(violating_voxel, n1)

                    else:
                        ordering_graph.remove_edge(n2, violating_voxel)
                        ordering_graph.add_edge(violating_voxel, n2)

                    break

            # if n1 in voxels_in_component and n2 in voxels_in_component and bfs_position[n1] < bfs_position[violating_voxel] and bfs_position[n2] < bfs_position[violating_voxel]:
            #     # Check if violating voxel is not built from the neighbors that are part of the violation
            #     if not ordering_graph.has_edge(n1, violating_voxel) and not ordering_graph.has_edge(n2, violating_voxel):
            #         # Remove edge to violating voxel and add edge from one of the neighbors and change BFS ordering.
            #         preds = list(ordering_graph.predecessors(violating_voxel))
            #
            #         if preds:
            #             p = preds[0]
            #
            #             ordering_graph.remove_edge(p, violating_voxel)
            #
            #             new_pred = random.choice([n1, n2])
            #
            #             ordering_graph.add_edge(new_pred, violating_voxel)
            #
            #             # Swap their BFS ordering
            #             violate_pos = bfs_position[violating_voxel]
            #
            #             bfs_position[violating_voxel] = bfs_position[new_pred]
            #             bfs_position[new_pred] = violate_pos
            #
            #     elif ordering_graph.has_edge(n1, violating_voxel) and ordering_graph.has_edge(n2, violating_voxel):
            #         neighbor = random.choice([n1, n2])
            #
            #         # Flip edge and swap BFS ordering
            #         ordering_graph.remove_edge(neighbor, violating_voxel)
            #         ordering_graph.add_edge(violating_voxel, neighbor)
            #
            #         # Swap their BFS ordering
            #         violate_pos = bfs_position[violating_voxel]
            #
            #         bfs_position[violating_voxel] = bfs_position[neighbor]
            #         bfs_position[neighbor] = violate_pos
            #
            #     elif ordering_graph.has_edge(n1, violating_voxel):
            #         if list(ordering_graph.predecessors(n2)):
            #             # Remove edge and add edge from violating voxel to n2
            #
            #         else:
            #             #
            #
            #         # Flip edge and swap BFS ordering
            #         ordering_graph.remove_edge(n1, violating_voxel)
            #         ordering_graph.add_edge(violating_voxel, n1)
            #
            #         # Swap their BFS ordering
            #         violate_pos = bfs_position[violating_voxel]
            #
            #         bfs_position[violating_voxel] = bfs_position[n1]
            #         bfs_position[n1] = violate_pos
            #
            #     elif ordering_graph.has_edge(n2, violating_voxel):
            #         # Flip edge and swap BFS ordering
            #         ordering_graph.remove_edge(n2, violating_voxel)
            #         ordering_graph.add_edge(violating_voxel, n2)
            #
            #         # Swap their BFS ordering
            #         violate_pos = bfs_position[violating_voxel]
            #
            #         bfs_position[violating_voxel] = bfs_position[n2]
            #         bfs_position[n2] = violate_pos

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

            # if n1 in voxels_in_component and n2 in voxels_in_component and bfs_position[n1] < bfs_position[voxel] and bfs_position[n2] < bfs_position[voxel]:
            #     violations.append(voxel)
            #
            #     break

    return violations