(define (domain combined_domain)
  (:requirements :strips :typing :adl :fluents :durative-actions :negative-preconditions :conditional-effects)
  
  (:types 
    joint workpiece
  )
  
  (:predicates
    (workpiece_orientation ?w - workpiece ?j - joint)
    (workpiece_held ?w - workpiece)
    (transit_allowed ?w - workpiece)
    (has_joint ?w - workpiece ?j - joint)
    (seam_measured ?j - joint)
    (depends_on ?j1 ?j2 - joint)
    (welded ?j - joint)
  )

  (:functions 
    (robots_available)
    (robots_needed ?w - workpiece)
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
            )
          )
          (at end(and
            (workpiece_orientation ?w ?to)
            (increase (robots_available) (robots_needed ?w))
            (not (transit_allowed ?w))
            )
          )
      )
  )

  (:durative-action clear_for_transit
      :parameters (?w - workpiece)
      :duration (= ?duration 1)
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

    (:durative-action weld
      :parameters (?j - joint)
      :duration (= ?duration 10)
      :condition (and (at start (and 
              (seam_measured ?j)
              (forall (?j2 - joint) (imply (depends_on ?j ?j2) (welded ?j2)))
              (forall (?w - workpiece) (imply (has_joint ?w ?j) (workpiece_orientation ?w ?j)))
              (>=(robots_available) 2)
                (not (welded ?j))
            ))
            (over all (forall (?w - workpiece) 
              (imply (has_joint ?w ?j) (workpiece_held ?w))
            ))
      )
      :effect (and 
          (at start (decrease (robots_available) 2))
          (at end (and 
                (welded ?j)
                (increase (robots_available) 2)
          ))
      )
  )
    
  (:durative-action hold
      :parameters (?w - workpiece)
      :duration (= ?duration 1)
      :condition 
          (at start (not (workpiece_held ?w)))
      :effect
          (at end (workpiece_held ?w))
  )

  (:durative-action release
      :parameters (?w - workpiece)
      :duration (= ?duration 1)
      :condition 
          (at start (workpiece_held ?w))
      :effect
          (at end (not (workpiece_held ?w)))
  )
  
  

  (:durative-action validate
      :parameters (?j - joint)
      :duration (= ?duration 4)
      :condition (and 
          (at start (and 
              (not (welded ?j))
              (not (seam_measured ?j))
              (>= (robots_available) 2)
              (forall (?w - workpiece) (imply (has_joint ?w ?j) (workpiece_orientation ?w ?j)))
          ))
          (over all (forall (?w - workpiece) 
              (imply (has_joint ?w ?j) (workpiece_held ?w))
          ))
      )
      :effect (and 
          (at start (decrease (robots_available) 2))
          (at end (and 
              (seam_measured ?j)
              (increase (robots_available) 2)
          ))
      )
  )
  
  
)
