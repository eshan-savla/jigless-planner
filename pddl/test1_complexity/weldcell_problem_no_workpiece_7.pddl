(define (problem weldcell_problem_joints) (:domain weld_domain)
(:objects
     joint0 joint1 joint2 joint3 joint4 joint5 joint6 joint7 joint8 joint9 joint10 - joint
    ;joint0 refers to initial position/orientation and is not actually a joint
)

(:init
    (joint_orientation joint0)

    (depends_on joint2 joint1)
    (depends_on joint3 joint2)
    (depends_on joint4 joint3)
    (not_joint_measured joint0)
    (not_joint_measured joint1)
    (not_joint_measured joint2)
    (not_joint_measured joint3)
    (not_joint_measured joint4)
    (not_joint_measured joint5)
    (not_joint_measured joint6)
    (not_joint_measured joint7)
    (not_joint_measured joint8)
    (not_joint_measured joint9)
    (not_joint_measured joint10)
    (not_welded joint1)
    (not_welded joint2)
    (not_welded joint3)
    (not_welded joint4)
    (not_welded joint5)
    (not_welded joint6)
    (not_welded joint7)
    (not_welded joint8)
    (not_welded joint9)
    (not_welded joint10)
)

; (:goal
;     ;todo: put the goal condition here
;     (and 
;         (welded joint1)
;         (welded joint2)
;         (welded joint3)
;         (welded joint4)
;         (welded joint5)
;         (welded joint6)
;         (welded joint7)
;         (welded joint8)
;         (welded joint9)
;         (welded joint10)
;     )
; )

;un-comment the following line if metric is needed
;(:metric minimize (???))
)
