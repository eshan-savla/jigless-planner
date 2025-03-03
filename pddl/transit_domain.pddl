(define (domain transit_domain)
  (:requirements :strips :typing :adl :durative-actions :fluents :conditional-effects)
  
  (:types 
    joint
  )
  
  (:predicates
    (joint_orientation ?j - joint)
  )

  (:durative-action transit
    :parameters (?from ?to - joint)
    :duration (= ?duration 5)
    :condition (and
          (at start (and
            (joint_orientation ?from)
          ))
          )
      
    :effect (and
        (at start (not (joint_orientation ?from)))

        (at end (joint_orientation ?to))
    )
  ) 
)