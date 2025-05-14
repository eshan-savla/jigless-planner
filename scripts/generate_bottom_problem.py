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
    pddl_content = """(define (problem weldcell_problem_joints) (:domain weld_domain)
(:objects
"""
    # Add joint objects
    pddl_content += "     joint0 " + " ".join(joints) + " - joint\n"
    pddl_content += "    ;joint0 refers to initial position/orientation and is not actually a joint\n)\n\n"

    # Add the initial state
    pddl_content += "(:init\n"
    pddl_content += "    (joint_orientation joint0)\n\n"

    # Add dependencies
    for i, row in enumerate(rows[1:]):  # Skip the header row
        joint_row = row[0]
        for j, cell in enumerate(row[1:]):  # Skip the first column
            if cell.strip().lower() == 'x':
                pddl_content += f"    (depends_on {joints[i]} {joints[j]})\n"

    # Add default facts
    for joint in joints:
        pddl_content += f"    (not_joint_measured {joint})\n"
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
    output_pddl_file = os.path.join(os.path.dirname(__file__),'../pddl/test1_complexity/weldcell_problem_no_workpiece_26.pddl')
    generate_pddl_from_csv(csv_file, output_pddl_file)

if __name__ == "__main__":
    main()