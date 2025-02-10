;Header and description

(define (domain weld_domain)

  ;remove requirements that are not needed
  (:requirements :strips :fluents :durative-actions :typing :adl :negative-preconditions :conditional-effects)

  (:types
    joint workpiece
  )

  (:functions
    (robots_available)
  )

  (:predicates ;todo: define predicates here
    (workpiece_orientation ?w - workpiece ?j - joint)
    (workpiece_held ?w - workpiece)
    (transit_allowed ?w - workpiece)
    (has_joint ?w - workpiece ?j - joint)
    (seam_measured ?j - joint)
    (depends_on ?j1 ?j2 - joint)
    (welded ?j - joint)
  )

  ;define actions here

(:durative-action weld
      :parameters (?j - joint)
      :duration (= ?duration 10)
      :condition (and (over all (and 
              (seam_measured ?j)
              (forall (?j2 - joint) (imply (depends_on ?j ?j2) (welded ?j2)))
              (forall (?w - workpiece) (imply (has_joint ?w ?j) (workpiece_orientation ?w ?j)))
              (not (welded ?j))
              (forall (?w - workpiece) 
              (imply (has_joint ?w ?j) (workpiece_held ?w)))
            ))
            (at start (>= (robots_available) 2))
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
          (at end (and 
            (workpiece_held ?w)
            (not (transit_allowed ?w))
          ))
  )

  (:durative-action release
      :parameters (?w - workpiece)
      :duration (= ?duration 1)
      :condition 
          (at start (workpiece_held ?w))
      :effect
          (at end (and 
            (not (workpiece_held ?w))
            (transit_allowed ?w)
          ))
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