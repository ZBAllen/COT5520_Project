import networkx as nx

def compute_component_workload(dependency_graph: nx.DiGraph,
                               current_component_id: int,
                               component_voxel_counts: dict[int, int],
                               robots_assigned_to_component: dict[int, int]) -> float:
    """
    Calculates the workload metric for a component to determine robot assignment priority.

    The workload represents the total amount of work remaining for a component,
    including all voxels that must be built in nodes that depend on it.
    This metric is used to assign robots to the component with the highest
    remaining workload, maximizing parallelism.

    Args:
        dependency_graph: NetworkX directed graph of component dependencies.
        current_component_id: ID of the component to calculate workload for.
        component_voxel_counts: Dictionary of number of voxels in each component.
        robots_assigned_to_component: Dictionary of what component each robot is assigned to.

    Returns:
        Workload metric = total dependent voxels divided by (robots assigned + 1).
        Higher values indicate components with more remaining work relative to assigned robots.
    """

    # Start with voxel count of current component
    total_dependent_voxels = component_voxel_counts[current_component_id]

    # Add voxel counts from all descendant components (components that depend on this component)
    descendant_component_ids = nx.descendants(dependency_graph, current_component_id)

    for descendant_node_id in descendant_component_ids:
        total_dependent_voxels += component_voxel_counts[descendant_node_id]

    # Calculate workload per robot: divide total work by number of robots already assigned
    # Add 1 to denominator to avoid division by zero and to give preference to unassigned nodes
    robots_currently_assigned = robots_assigned_to_component[current_component_id]

    workload_per_robot = total_dependent_voxels / (robots_currently_assigned + 1)

    return workload_per_robot