import networkx as nx
import re
from typing import List, Tuple
from functools import lru_cache
from itertools import combinations
from collections import defaultdict

def extract_joints_and_dependencies(pddl_text: str, exclude_joint0: bool =True) -> Tuple[List[str], List[Tuple[str, str]]]:
    """
    Extracts joints and 'depends_on' predicates from a PDDL problem definition.
    
    Args:
        pddl_text (str): The content of the PDDL file.
        exclude_joint0 (bool): Whether to exclude joint0 (if used as base/orientation).

    Returns:
        joints (List[str]): All joint names (excluding joint0 if specified)
        dependencies (List[Tuple[str, str]]): List of (from_joint, to_joint) dependencies
    """
    # Extract all objects of type 'joint'
    joint_block = re.search(r':objects(.*?)\)', pddl_text, re.DOTALL)
    joints = []
    if joint_block:
        object_lines = joint_block.group(1)
        joint_matches = re.search(r'([\w\s]+)(?=\s+-\s+joint)', object_lines)
        joint_matches = joint_matches.group(1).split() if joint_matches else []
        for match in joint_matches:
            joints += match.split()

    # Optionally remove joint0
    if exclude_joint0:
        joints = [j for j in joints if j != "joint0"]

    # Extract all (depends_on jointA jointB)
    dependencies = re.findall(r'\(depends_on\s+(\w+)\s+(\w+)\)', pddl_text)
    dependencies = [(j2, j1) for j1, j2 in dependencies]  # Reverse the order to match (from, to)

    return joints, dependencies

def create_seam_dag(seams: List[str], dependencies: List[str]) -> nx.DiGraph:
    """
    Create a DAG from seam dependencies.
    
    :param num_seams: Total number of seams (nodes)
    :param dependencies: List of (i, j) tuples meaning seam i must precede seam j
    :return: A NetworkX DAG object
    """
    dag = nx.DiGraph()
    dag.add_nodes_from(seams)
    dag.add_edges_from(dependencies)

    if not nx.is_directed_acyclic_graph(dag):
        raise ValueError("The dependencies form a cycle. DAG must be acyclic.")

    return dag

def count_topological_sorts(dag: nx.DiGraph) -> int:
    """
    Count the number of valid topological orderings of the DAG.

    Uses memoized DFS for exact count.
    """
    nodes = tuple(dag.nodes)

    @lru_cache(maxsize=None)
    def dfs(remaining_nodes, in_degrees_tuple):
        if not remaining_nodes:
            return 1
        in_degrees = dict(in_degrees_tuple)
        count = 0
        for i, node in enumerate(remaining_nodes):
            if in_degrees[node] == 0:
                next_nodes = remaining_nodes[:i] + remaining_nodes[i+1:]
                next_in_degrees = in_degrees.copy()

                # Reduce in-degree of node's neighbors
                for neighbor in dag.successors(node):
                    next_in_degrees[neighbor] -= 1

                count += dfs(next_nodes, tuple(next_in_degrees.items()))

        return count

    # Initial in-degree map
    in_degrees = {node: 0 for node in nodes}
    for u, v in dag.edges:
        in_degrees[v] += 1

    # Start DFS with all nodes in a fixed order
    return dfs(nodes, tuple(in_degrees.items()))

# 🧪 Example usage
if __name__ == "__main__":
    import os
    pddl_file = os.path.join(os.path.dirname(__file__), "../pddl/test1/weldcell_problem_no_workpiece_1.pddl")
    with open(pddl_file, "r") as file:
        pddl_text = file.read()
    
    joints, dependencies = extract_joints_and_dependencies(pddl_text)
    # print(f"Extracted joints: {joints}")
    # print(f"Extracted dependencies: {dependencies}")

    dag = create_seam_dag(joints, dependencies)
    total_sequences = count_topological_sorts(dag)

    print(f"Number of valid welding sequences: {total_sequences}")

    # Optional: visualize the graph
    import matplotlib.pyplot as plt
    nx.draw(dag, with_labels=True, arrows=True, node_color='lightgreen')
    plt.title("Seam Dependency DAG")
    plt.show()
