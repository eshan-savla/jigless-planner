(define (domain transit_domain)
  (:requirements :strips :typing :adl :durative-actions :fluents :conditional-effects)
  
  (:types 
    joint
  )
  
  (:predicates
    (joint_orientation ?j - joint)
    (not_joint_measured ?j - joint)
  )

  (:durative-action transit
    :parameters (?from ?to - joint)
    :duration (= ?duration 5)
    :condition (and
          (at start (and
            (joint_orientation ?from)
          ))
          (over all (and
            (not_joint_measured ?from)
          ))
    )
      
    :effect (and
        (at start (not (joint_orientation ?from)))

        (at end (joint_orientation ?to))
    )
  ) 
)