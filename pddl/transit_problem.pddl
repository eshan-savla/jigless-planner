(define (problem transit_problem) (:domain transit_domain)
(:objects
    robot1 - robot
    robot2 - robot
    station1 - station
    station2 - station
    station3 - station
    workpiece1 - workpiece
    workpiece2 - workpiece
    workpiece3 - workpiece 
)

(:init
    ;todo: put the initial state's facts and numeric values here
    (robot_at robot1 station1)
    (robot_at robot2 station1)
    (robot_available robot1)   
    (robot_available robot2)
    (piece_at workpiece1 station1)
    (piece_at workpiece2 station2)
    (piece_at workpiece3 station2)
)

(:goal (and
    ;todo: put the goal condition here
    (piece_at workpiece1 station3)
    (piece_at workpiece2 station3)
    (piece_at workpiece3 station3)
    (robot_available robot1)
    (robot_available robot2)
))

;un-comment the following line if metric is needed
;(:metric minimize (???))
)
