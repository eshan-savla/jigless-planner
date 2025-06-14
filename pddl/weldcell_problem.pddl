(define (problem weldcell_problem) (:domain combined_domain)
(:objects
    workpiece1 workpiece2 workpiece3 - workpiece
    joint0 joint1 joint2 joint3 joint4 joint5 joint6 joint7 joint8 - joint
    ;joint0 refers to initial position/orientation and is not actually a joint
)

(:init
    ;todo: put the initial state's facts and numeric values here
    (= (robots_available) 2)
    (= (robots_needed workpiece1) 1)
    (= (robots_needed workpiece2) 1)
    (= (robots_needed workpiece3) 1)

    (workpiece_orientation workpiece1 joint0)
    (workpiece_orientation workpiece2 joint1)
    (workpiece_orientation workpiece3 joint0)

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

    (has_joint workpiece3 joint0)
    (has_joint workpiece3 joint6)
    (has_joint workpiece3 joint7)
    (has_joint workpiece3 joint8)

    (depends_on joint3 joint1)
    (depends_on joint4 joint2)
    (depends_on joint5 joint3)
    (depends_on joint5 joint4)
    (depends_on joint6 joint1)
    (depends_on joint7 joint1)
    (depends_on joint8 joint7)
    (depends_on joint8 joint6)
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
