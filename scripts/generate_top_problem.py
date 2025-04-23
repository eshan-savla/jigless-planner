import csv

# filepath: /home/mdh-es/multirobot_ws/src/jigless-planner/scripts/generate_pddl.py
def generate_pddl_from_csv(csv_file, output_pddl_file):
    # Read the CSV file
    with open(csv_file, 'r') as file:
        reader = csv.reader(file)
        rows = list(reader)

    # Extract joint names from the first row and column
    joints = rows[0][1:]  # Skip the first empty cell in the header row

    # Initialize the PDDL content
    pddl_content = """(define (problem welding-problem) (:domain welding-top)
(:objects
"""
    # Add joint objects
    pddl_content += "    " + " ".join(joints) + " - joint\n"
    pddl_content += "    pos1 - position\n)\n\n"

    # Add the initial state
    pddl_content += "(:init\n"
    pddl_content += "    (at pos1)\n\n"
    pddl_content += "    (not_executed)\n\n"
    pddl_content += "    (not_executing)\n\n"
    pddl_content += "    (= (avg_joints_per_group) " + str(len(joints)) + ")\n\n"

    # Add dependencies
    for i, row in enumerate(rows[1:]):  # Skip the header row
        joint_row = row[0]
        for j, cell in enumerate(row[1:]):  # Skip the first column
            if cell.strip().lower() == 'x':
                pddl_content += f"    (depends_on {joints[i]} {joints[j]})\n"

    # Add default facts
    for joint in joints:
        pddl_content += f"    (reachable_at {joint} pos1)\n"
    for joint in joints:
        pddl_content += f"    (not_welded {joint})\n"

    pddl_content += ")\n\n"

    # Add the goal section as a placeholder
    pddl_content += """; (:goal
;     ;todo: put the goal condition here
;     (and 
"""
    for joint in joints:
        pddl_content += f";         (welded {joint})\n"
    pddl_content += """;     )
; )

;un-comment the following line if metric is needed
;(:metric minimize (???))
)
"""

    # Write the PDDL content to the output file
    with open(output_pddl_file, 'w') as file:
        file.write(pddl_content)

def main():
    import os
    csv_file = os.path.join(os.path.dirname(__file__), '../pddl/joint_dependencies.csv')
    output_pddl_file = os.path.join(os.path.dirname(__file__), '../pddl/test1/top_welding_problem_20.pddl')
    generate_pddl_from_csv(csv_file, output_pddl_file)

if __name__ == "__main__":
    main()