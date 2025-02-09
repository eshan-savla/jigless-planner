;Header and description

(define (domain weld_domain)

  ;remove requirements that are not needed
  (:requirements :strips :fluents :durative-actions :typing :adl :negative-preconditions)

  (:types
    joint workpiece
  )

  (:functions
    (robots_available)
  )

  (:predicates ;todo: define predicates here
    (seam_measured ?j - joint)
    (depends_on ?j1 ?j2 - joint)
    (welded ?j - joint)
    (workpiece_orientation ?w - workpiece ?j - joint)
    (has_joint ?w - workpiece ?j - joint)

  )

  ;define actions here

  (:durative-action weld
      :parameters (?j - joint)
      :duration (= ?duration 10)
      :condition (at start (and 
              (seam_measured ?j)
              (forall (?j2 - joint) (imply (depends_on ?j ?j2) (welded ?j2)))
              (forall (?w - workpiece) (imply (has_joint ?w ?j) (workpiece_orientation ?w ?j)))
              (>=(robots_available) 2)
                (not (welded ?j))
          ))
      :effect (and 
          (at start (decrease (robots_available) 2))
          (at end (and 
                (welded ?j)
                (increase (robots_available) 2)
          ))
      )
  )

  (:action validate
      :parameters (?j - joint)
      :precondition (and 
          (not (welded ?j))
          (not (seam_measured ?j))
          (>= (robots_available) 2)
      )
      :effect (seam_measured ?j)
  )
  
)