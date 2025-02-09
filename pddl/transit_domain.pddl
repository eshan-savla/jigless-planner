(define (domain transit_domain)
  (:requirements :strips :typing :adl :fluents :durative-actions :negative-preconditions)
  
  (:types 
    joint workpiece
  )

  (:functions 
    (robots_available)
    (robots_needed ?w - workpiece)
  )
  
  (:predicates
    (workpiece_orientation ?w - workpiece ?j - joint)
    (workpiece_held ?w - workpiece)
    (transit_allowed ?w - workpiece)
    (has_joint ?w - workpiece ?j - joint)
  )
  
  (:durative-action transit
      :parameters (?w - workpiece ?from ?to - joint)
      :duration (= ?duration 5)
      :condition (and
            (at start (and
              (workpiece_orientation ?w ?from)
              (>=(robots_available) (robots_needed ?w))
              (transit_allowed ?w)
              (not (workpiece_held ?w))
              (has_joint ?w ?from)
              (has_joint ?w ?to)
              )
            ))
            
      
      :effect (and
          (at start(and
            (decrease (robots_available) (robots_needed ?w))
            (workpiece_held ?w)
            )
          )
          (at end(and
            (workpiece_orientation ?w ?to)
            (increase (robots_available) (robots_needed ?w))
            (not (workpiece_held ?w))
            (not (transit_allowed ?w))
            )
          )
      )
  )

  (:durative-action clear_for_transit
      :parameters (?w - workpiece)
      :duration (= ?duration 3)
      :condition (and 
          (at start (and 
            (not (workpiece_held ?w))
            (not (transit_allowed ?w))
            )
          )
      )
      :effect (and 
          (at end (transit_allowed ?w))
      )
  ) ; is this action even needed at this level? Cant it be resolved at task layer
  
)
