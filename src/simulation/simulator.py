import random
import time
from collections import defaultdict
import matplotlib.pyplot as plt

from src.config import *
from src.structure.voxel_grid import VoxelGrid
from src.structure.dependency_graph import build_graph, contract_graph, add_scaffolding
from src.planning.component_ordering import order_component_voxels
from src.planning.workload import compute_component_workload
from src.pathfinding.a_star import a_star
from src.robots.robot import Robot
from src.utils import valid_construction_locations
from src.visualization.viewer import Viewer

class Simulator:
    def __init__(self):
        self.voxel_grid = VoxelGrid()
        self.component_dependency_graph = None
        self.generate_structures_and_graph()

        self.voxels_available = len(self.voxel_grid.target) + len(self.voxel_grid.scaffold)
        self.voxels_in_transit = 0

        self.component_orderings = {}
        self.component_sizes = {}
        self.calculate_component_orderings()

        self.completed_components = set()
        self.num_robots_assigned_to_components = defaultdict(int)

        self.robots = [Robot(i, (0, i % GRID_SIZE[1], 0)) for i in range(NUM_ROBOTS)]
        self.total_steps = 0

        self.viewer = Viewer()

    def update(self):
        """Updates the state of the simulation by one step."""

        bots = self.robots.copy()
        random.shuffle(bots)

        for robot in bots:
            if robot.component is not None and self.is_teardown_component(robot.component):
                self.update_teardown_robot(robot)

            else:
                self.update_construction_robot(robot)

    def update_construction_robot(self, robot: Robot):
        """Updates the state of the current robot for construction."""

        # If robot doesn't have a voxel, send to voxel depot
        if not robot.has_voxel:
            if self.voxels_available > 0:
                if robot.position != DEPOT_POS:
                    if not robot.path:
                        robot.path = a_star(robot.position, DEPOT_POS, self.voxel_grid.built)

                    if robot.path:
                        robot.step()

                        self.total_steps += 1

                else:
                    # At depot, pick up voxel
                    self.pickup_voxel(robot)

                    robot.component = None
                    robot.voxel_index = 0  # TODO: Do you need to set voxel_index to 0 here? It gets set to 0 in assign when a component becomes available.

            else:
                # No more voxels available, return to the voxel depot and wait
                if robot.position != DEPOT_POS:
                    if not robot.path:
                        robot.path = a_star(robot.position, DEPOT_POS, self.voxel_grid.built)

                    if robot.path:
                        robot.step()

                        self.total_steps += 1

            return  # Move on to next robot

        # Robot has voxel, assign to component if needed
        if robot.component is None:
            self.assign(robot)

        # TODO: Robots shouldn't drop the voxel before reaching the storage depot (to simulate them having to put voxels back in storage).
        # If still no assignment after trying, drop voxel and return to depot
        if robot.component is None:
            self.drop_voxel(robot)

            return  # Move on to next robot

        # If assigned to a teardown component, drop voxel first and switch to teardown path
        if self.is_teardown_component(robot.component):
            self.drop_voxel(robot)

            self.update_teardown_robot(robot)

            return

        # Robot has voxel, component assigned, assign a voxel in the component for the robot to build
        ordered_voxels_in_component = self.component_orderings[robot.component]

        # Skip voxels already built
        while robot.voxel_index < len(ordered_voxels_in_component) and ordered_voxels_in_component[
            robot.voxel_index] in self.voxel_grid.built:
            robot.voxel_index += 1

        # If component complete, return voxel and find new component
        if robot.voxel_index >= len(ordered_voxels_in_component):
            self.num_robots_assigned_to_components[robot.component] -= 1
            robot.component = None

            self.drop_voxel(
                robot)  # TODO: I added this since this condition suggests that the robot didn't get to use its voxel. Like before this needs replaced with the robot explicitly walking back to storage to return the voxel.

            return

        # At this point, the robot has a voxel, the target voxel is not already completed.

        target_voxel = ordered_voxels_in_component[robot.voxel_index]

        # Debugging print for target voxels at z=1 with no valid path returned by A*
        # if target_voxel[2] == 1 and a_star(robot.position, target_voxel, self.voxel_grid.built) == []:
        #     print(f"Robot {robot.id} | pos={robot.position} | target_voxel={target_voxel} | can_build={self.voxel_grid.can_build(target_voxel, robot.voxel_index, self.robots, ordered_voxels_in_component)} | path={robot.path}")
        #     print(self.voxel_grid.built)

        valid_build_locations = valid_construction_locations(target_voxel, self.voxel_grid.built)

        # If the block can't be built due to obstructions, print a statement
        if not valid_build_locations:
            print("All valid build locations obstructed!")

        # If the robot is not near the target voxel location, calculate the path if needed, then take a step towards the target.
        if robot.position not in valid_build_locations:
            if robot.path and robot.path[0] not in self.voxel_grid.built:
                robot.step()

                self.total_steps += 1

            else:
                # Pick a random valid construction location to start moving to
                destination_voxel = valid_build_locations[random.randint(0, len(valid_build_locations) - 1)]

                robot.path = a_star(robot.position, destination_voxel, self.voxel_grid.built)

        else:
            # If the robot is near to the target voxel position, check if it can be built.
            # If it can be built, update relevant lists and variables.
            if self.voxel_grid.can_build(target_voxel, robot.voxel_index, self.robots, ordered_voxels_in_component):
                self.voxel_grid.built.add(target_voxel)

                robot.has_voxel = False
                self.voxels_in_transit -= 1

                if robot.voxel_index == 0:
                    print(f"{robot.id} says: I just placed voxel 0 of component {robot.component}")
                    print(f"Now built contains: {self.voxel_grid.built}")

                robot.voxel_index = 0  # Reset voxel index

                # If all voxels in the component are built, mark it as completed.
                if all(v in self.voxel_grid.built for v in ordered_voxels_in_component):
                    self.completed_components.add(robot.component)

                self.num_robots_assigned_to_components[robot.component] -= 1
                robot.component = None  # Remove robot's assigned component, so it can work on other things if needed

    def update_teardown_robot(self, robot: Robot):
        """
        Handle a robot assigned to a scaffold teardown component.
        The robot moves to each scaffold voxel in reverse (top-down) order and removes it.
        The robot does not carry a voxel during teardown.
        """

        # Ensure the robot is not carrying a voxel during teardown
        if robot.has_voxel:
            self.drop_voxel(robot)

        ordered_voxels = self.component_orderings[robot.component]

        # Skip voxels already removed (not in built)
        while (robot.voxel_index < len(ordered_voxels)) and ordered_voxels[robot.voxel_index] not in self.voxel_grid.built:
            robot.voxel_index += 1

        # If all scaffold voxels have been removed, mark component complete
        if robot.voxel_index >= len(ordered_voxels):
            self.num_robots_assigned_to_components[robot.component] -= 1

            robot.component = None

            return

        target_voxel = ordered_voxels[robot.voxel_index]

        valid_locations = valid_construction_locations(target_voxel, self.voxel_grid.built)

        if not valid_locations:
            print(f"Teardown: no valid locations near {target_voxel}")

            return

        if robot.position not in valid_locations:
            if robot.path and robot.path[0] not in self.voxel_grid.built:
                robot.step()

                self.total_steps += 1

            else:
                destination = valid_locations[random.randint(0, len(valid_locations) - 1)]

                robot.path = a_star(robot.position, destination, self.voxel_grid.built)

        else:
            # Remove the scaffold voxel
            self.voxel_grid.built.discard(target_voxel)
            self.voxel_grid.scaffold.discard(target_voxel)

            robot.has_voxel = True
            self.voxels_in_transit += 1

            if robot.voxel_index == 0:
                print(f"{robot.id} says: I removed scaffold voxel {target_voxel}")

            robot.voxel_index = 0

            # Check component completion after removal
            if all(v not in self.voxel_grid.built for v in ordered_voxels):
                self.completed_components.add(robot.component)

            self.num_robots_assigned_to_components[robot.component] -= 1
            robot.component = None

    def run(self):
        plt.ion()

        try:
            while len(self.completed_components) < len(self.component_dependency_graph.nodes) or not all(robot.position == DEPOT_POS for robot in self.robots):
                self.update()

                self.viewer.draw(self.voxel_grid, self.robots, self.voxels_available, self.voxels_in_transit, self.total_steps)

                time.sleep(STEP_DELAY)

        except (KeyboardInterrupt, Exception):
            print("\nClosing visualization...")
            plt.close('all')

        print("Done")

        # Leave the figure open to be able to continue to rotate the completed structure.
        try:
            while True:
                self.viewer.draw(self.voxel_grid, self.robots)
                plt.pause(0.01)

        except (KeyboardInterrupt, Exception):
            print("\nClosing visualization...")
            plt.close('all')


    ##################### Helper functions #####################


    def generate_structures_and_graph(self):
        """
        Adds structures to the voxel grid according to the config file.
        Then, creates the dependency graph for the components of the structures.
        """

        # Add target structures to the voxel grid
        for structure in STRUCTURES:
            self.voxel_grid.add_structure(structure)

        # Build the component dependency graph
        voxel_dependency_graph = build_graph(self.voxel_grid)

        self.component_dependency_graph = contract_graph(voxel_dependency_graph)

        # Add scaffolding for unreachable components if selected
        if SCAFFOLDING:
            self.component_dependency_graph = add_scaffolding(self.component_dependency_graph, self.voxel_grid)

    def calculate_component_orderings(self):
        """Generates the voxel orderings for each component of the dependency graph."""

        for component in self.component_dependency_graph:

            node_data = self.component_dependency_graph.nodes[component]
            component_voxels = node_data['voxels']

            print(f"\nOrdering component {component} with {len(component_voxels)} voxels...")

            # Scaffold nodes have a pre-determined order; skip the BFS ordering for them
            if node_data.get("is_scaffold") or node_data.get("is_scaffold_teardown"):
                component_voxel_build_order = node_data["scaffold_order"]

            else:
                component_voxel_build_order = order_component_voxels(
                    component_voxels, self.component_dependency_graph, component
                )

            self.component_orderings[component] = component_voxel_build_order
            self.component_sizes[component] = len(component_voxels)

            if not component_voxel_build_order:
                print(f"\nDEBUG: Component {component} has empty ordering!")
                print(f"  Voxels in component: {component_voxels}")
                print(f"  Number of voxels: {len(component_voxels)}")

            else:
                print(f"\nDEBUG: Component {component} ordering successful. {len(component_voxel_build_order)} voxels ordered.")

    def available_components(self):
        """Returns the set of components that are not completed yet and are reachable."""

        components = []

        for component in self.component_dependency_graph.nodes:
            if component in self.completed_components:
                continue

            if all(comp in self.completed_components for comp in self.component_dependency_graph.predecessors(component)):
                components.append(component)

        random.shuffle(components)

        return components

    def assign(self, robot: Robot):
        """
        Assigns the given robot to an available voxel.

        If no voxels exist that aren't built and are reachable, then returns without assignment.
        """

        # Get the components that are currently buildable
        available_comps = self.available_components()

        # If no component available, set robot's assigned component to None and return
        if not available_comps:
            # print(f"Robot {robot.id}: No available nodes")
            robot.component = None
            robot.voxel_index = 0

            return

        reachable_components = []

        # Find the set of components that are reachable
        for component in available_comps:
            if not self.component_orderings[component]:
                continue

            first_voxel = self.component_orderings[component][0]

            if a_star(robot.position, first_voxel, self.voxel_grid.built):
                reachable_components.append(component)

            # else:
            #     print(f"Robot {robot.id}: Cannot reach component {component}, first voxel {first_voxel}")

        if not reachable_components:
            # print(f"Robot {robot.id}: No reachable nodes from {len(available_comps)} available")
            robot.component = None
            robot.voxel_index = 0

            return

        # best_component = max(reachable_components, key=lambda comp: compute_component_workload(self.component_dependency_graph, comp, self.component_sizes, self.num_robots_assigned_to_components))

        best_component = random.choice(reachable_components)

        robot.component = best_component
        print(f"{robot.id} says: I've been assigned to component {robot.component}")
        robot.voxel_index = 0

        self.num_robots_assigned_to_components[best_component] += 1

        # print(f"Robot {robot.id}: Assigned to component {best_component}")

    def pickup_voxel(self, robot: Robot):
        robot.has_voxel = True
        self.voxels_available -= 1
        self.voxels_in_transit += 1

    def drop_voxel(self, robot: Robot):
        robot.has_voxel = False
        self.voxels_available += 1
        self.voxels_in_transit -= 1

    def is_teardown_component(self, component: int) -> bool:
        """Returns true if the given component is a scaffold teardown node."""

        return self.component_dependency_graph.nodes[component].get("is_scaffold_teardown", False)