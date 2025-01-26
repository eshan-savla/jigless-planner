(define (domain transit_domain)
  (:requirements :strips :typing :adl :fluents :durative-actions)
  
  (:types 
    robot station workpiece
  )
  
  (:predicates
    (robot_at ?r - robot ?s - station)
    (robot_available ?r - robot)
    (holding ?r - robot ?w - workpiece)
    (piece_at ?w - workpiece ?s - station)
  )

  (:durative-action move_robot
      :parameters (?r - robot ?from ?to - station)
      :duration (= ?duration 5)
      :condition (and 
          (at start (and 
            (robot_at ?r ?from)
            (robot_available ?r)
          ))
      )
      :effect (and 
          (at start (and 
            (not (robot_available ?r))
          ))
          (at end (and 
            (robot_at ?r ?to)
            (not (robot_at ?r ?from))
            (robot_available ?r)
          ))
      )
  )
  
  
  (:durative-action move_workpiece
      :parameters (?r - robot ?w - workpiece ?from ?to - station)
      :duration (= ?duration 5)
      :condition (and 
          (at start (and 
            (piece_at ?w ?from)
            (robot_available ?r)
            (robot_at ?r ?from)
          ))
      )
      :effect (and 
          (at start (and 
            (not (robot_available ?r))
            (holding ?r ?w)
          ))
          (at end (and 
            (robot_at ?r ?to)
            (not (robot_at ?r ?from))
            (not (piece_at ?w ?from))
            (piece_at ?w ?to)
            (robot_available ?r)
            (not(holding ?r ?w))
          ))
      )
  )
)