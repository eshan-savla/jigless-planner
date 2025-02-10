(define (problem weldcell_problem) (:domain combined_domain)
(:objects
    workpiece1 workpiece2 - workpiece
    joint1 joint2 joint3 joint4 joint5 - joint
)

(:init
    ;todo: put the initial state's facts and numeric values here
    (= (robots_available) 2)
    (= (robots_needed workpiece1) 1)
    (= (robots_needed workpiece2) 1)

    (workpiece_orientation workpiece1 joint1)
    (workpiece_orientation workpiece2 joint1)

    (not (workpiece_held workpiece1))
    (not (workpiece_held workpiece2))

    (transit_allowed workpiece1)
    (transit_allowed workpiece2)

    (has_joint workpiece1 joint1)
    (has_joint workpiece1 joint2)
    (has_joint workpiece1 joint3)
    (has_joint workpiece1 joint4)
    (has_joint workpiece1 joint5)

    (has_joint workpiece2 joint1)
    (has_joint workpiece2 joint2)
    (has_joint workpiece2 joint3)
    (has_joint workpiece2 joint4)
    (has_joint workpiece2 joint5)

    (not (seam_measured joint1))
    (not (seam_measured joint2))
    (not (seam_measured joint3))
    (not (seam_measured joint4))
    (not (seam_measured joint5))

    (depends_on joint3 joint1)
    (depends_on joint4 joint2)
    (depends_on joint5 joint3)
    (depends_on joint5 joint4)

    (not (welded joint1))
    (not (welded joint2))
    (not (welded joint3))
    (not (welded joint4))
    (not (welded joint5))
)

(:goal
    ;todo: put the goal condition here
    (and 
        (welded joint1)
        (welded joint2)
        (welded joint3)
        (welded joint4)
        (welded joint5)
    )
)

;un-comment the following line if metric is needed
;(:metric minimize (???))
)
