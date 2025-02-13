(define (problem weldcell_problem_popf) (:domain combined_domain_popf)
(:objects
    workpiece1 workpiece2  - workpiece ; workpiece3
    joint0 joint1 joint2 joint3 joint4 joint5 joint6 joint7 joint8 - joint
    ;joint0 refers to initial position/orientation and is not actually a joint
    robot1 robot2 - robot
)

(:init
    ;todo: put the initial state's facts and numeric values here
    
    (is_free robot1)
    (is_free robot2)

    (workpiece_orientation workpiece1 joint0)
    (workpiece_orientation workpiece2 joint1)
    ; (workpiece_orientation workpiece3 joint0)

    (not_workpiece_held workpiece1)
    (not_workpiece_held workpiece2)
    ; (not_workpiece_held workpiece3)

    (has_joint workpiece1 joint0)
    (has_joint workpiece1 joint1)
    (has_joint workpiece1 joint2)
    (has_joint workpiece1 joint3)
    (has_joint workpiece1 joint4)
    (has_joint workpiece1 joint5)
    (has_joint workpiece1 joint6)
    (has_joint workpiece1 joint7)
    (has_joint workpiece1 joint8)

    (has_joint workpiece2 joint0)
    (has_joint workpiece2 joint1)
    (has_joint workpiece2 joint2)
    (has_joint workpiece2 joint3)
    (has_joint workpiece2 joint4)
    (has_joint workpiece2 joint5)

    ; (has_joint workpiece3 joint0)
    ; (has_joint workpiece3 joint6)
    ; (has_joint workpiece3 joint7)
    ; (has_joint workpiece3 joint8)

    (depends_on joint3 joint1)
    (depends_on joint4 joint2)
    (depends_on joint5 joint3)
    (depends_on joint5 joint4)
    (depends_on joint6 joint1)
    (depends_on joint7 joint1)
    (depends_on joint8 joint7)
    (depends_on joint8 joint6)

    (not_seam_measured joint1)
    (not_seam_measured joint2)
    (not_seam_measured joint3)
    (not_seam_measured joint4)
    (not_seam_measured joint5)
    (not_seam_measured joint6)
    (not_seam_measured joint7)
    (not_seam_measured joint8)

    (not_welded joint1)
    (not_welded joint2)
    (not_welded joint3)
    (not_welded joint4)
    (not_welded joint5)
    (not_welded joint6)
    (not_welded joint7)
    (not_welded joint8)
)

(:goal
    ;todo: put the goal condition here
    (and 
        (welded joint1)
        (welded joint2)
        (welded joint3)
        (welded joint4)
        (welded joint5)
        ; (welded joint6)
        ; (welded joint7)
        ; (welded joint8)
    )
)

;un-comment the following line if metric is needed
;(:metric minimize (???))
)
