(define (problem welding-problem) (:domain welding-top)
(:objects
    joint1 joint2 joint3 joint4 joint5 joint6 joint7 joint8 joint9 joint10 - joint
    pos1 - position
)

(:init
    (at pos1)

    (not_executed)

    (not_executing)

    (= (avg_joints_per_group) 10)

    (depends_on joint2 joint1)
    (depends_on joint3 joint1)
    (depends_on joint4 joint1)
    (depends_on joint5 joint1)
    (depends_on joint6 joint1)
    (depends_on joint7 joint1)
    (depends_on joint8 joint1)
    (depends_on joint9 joint1)
    (depends_on joint10 joint1)
    (depends_on joint10 joint2)
    (depends_on joint10 joint3)
    (depends_on joint10 joint4)
    (depends_on joint10 joint5)
    (depends_on joint10 joint6)
    (depends_on joint10 joint7)
    (depends_on joint10 joint8)
    (depends_on joint10 joint9)
    (reachable_at joint1 pos1)
    (reachable_at joint2 pos1)
    (reachable_at joint3 pos1)
    (reachable_at joint4 pos1)
    (reachable_at joint5 pos1)
    (reachable_at joint6 pos1)
    (reachable_at joint7 pos1)
    (reachable_at joint8 pos1)
    (reachable_at joint9 pos1)
    (reachable_at joint10 pos1)
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
