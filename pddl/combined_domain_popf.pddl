(define (domain combined_domain_popf)
  (:requirements :strips :typing :adl :durative-actions :fluents :conditional-effects)
  
  (:types 
    joint workpiece robot
  )
  
  (:predicates
    (workpiece_orientation ?w - workpiece ?j - joint)
    (workpiece_held ?w - workpiece)
    (not_workpiece_held ?w - workpiece)
    (has_joint ?w - workpiece ?j - joint)
    (seam_measured ?j - joint)
    (not_seam_measured ?j - joint)
    (workpiece_validated ?w - workpiece)
    (depends_on ?j1 ?j2 - joint)
    (welded ?j - joint)
    (not_welded ?j - joint)
    ; (workpiece_unjoined ?w - workpiece)
    (fused ?w1 ?w2 - workpiece)
    (not_fused ?w1 ?w2 - workpiece)
    (is_free ?r - robot)
    (workpiece_held_by ?w - workpiece ?r - robot)
    (not_workpiece_moved ?w - workpiece)
  )

  (:durative-action transit
    :parameters (?w - workpiece ?r - robot ?from ?to - joint)
    :duration (= ?duration 5)
    :condition (and
          (at start (and
            (workpiece_orientation ?w ?from)
            (is_free ?r)
          ))
          (over all (and
            (forall (?w2 - workpiece)
              (imply (and (not (= ?w ?w2))) (not_fused ?w ?w2))
            )
            (not_workpiece_held ?w)
            (has_joint ?w ?from)
            (has_joint ?w ?to)
          ))
          )
      
    :effect (and
        (at start (not (is_free ?r)))
        (at start (not (workpiece_orientation ?w ?from)))
        (at start (workpiece_held_by ?w ?r))
        ; (at start (not (not_workpiece_moved ?w)))
        (at end(is_free ?r))
        (at end (workpiece_orientation ?w ?to))
        (at end (not (workpiece_held_by ?w ?r)))
    )
  )

  (:durative-action transit-fused-two
      :parameters (?w1 ?w2 - workpiece ?r - robot ?from ?to - joint)
      :duration (= ?duration 5)
      :condition (and 
          (at start (and 
            (fused ?w1 ?w2)
            (forall (?w3 - workpiece)
              (imply (and (not (= ?w3 ?w1)) (not (= ?w3 ?w2))) (and (not_fused ?w2 ?w3) (not_fused ?w1 ?w3)) )
            )
            (workpiece_orientation ?w1 ?from)
            (workpiece_orientation ?w2 ?from)
            (is_free ?r)
          ))
          (over all (and
            (not_workpiece_held ?w1)
            (not_workpiece_held ?w2)
            (has_joint ?w1 ?from) ; as only one of the fused pieces needs to have this condition
            (has_joint ?w1 ?to)
          ))
      )
      :effect (and 
          (at start (and 
            (not (is_free ?r))
            (not (workpiece_orientation ?w1 ?from))
            (not (workpiece_orientation ?w2 ?from))
            (workpiece_held_by ?w1 ?r)
            (workpiece_held_by ?w2 ?r)
          ))
          (at end (and 
            (is_free ?r)
            (workpiece_orientation ?w1 ?to)
            (workpiece_orientation ?w2 ?to)
            (not (workpiece_held_by ?w1 ?r))
            (not (workpiece_held_by ?w2 ?r))
          ))
      )
  )

  (:durative-action transit-fused-three
      :parameters (?w1 ?w2 ?w3 - workpiece ?r - robot ?from ?to - joint)
      :duration (= ?duration 5)
      :condition (and 
          (at start (and 
            (fused ?w1 ?w2)
            (fused ?w1 ?w3)
            (workpiece_orientation ?w1 ?from)
            (workpiece_orientation ?w2 ?from)
            (workpiece_orientation ?w3 ?from)
            (is_free ?r)
          ))
          (over all (and 
            (not_workpiece_held ?w1)
            (not_workpiece_held ?w2)
            (not_workpiece_held ?w3)
            (has_joint ?w1 ?from) ; as only one of the fused pieces needs to have this condition
            (has_joint ?w1 ?to)
          ))
      )
      :effect (and 
          (at start (and 
            (not (is_free ?r))
            (not (workpiece_orientation ?w1 ?from))
            (not (workpiece_orientation ?w2 ?from))
            (not (workpiece_orientation ?w3 ?from))
            (workpiece_held_by ?w1 ?r)
            (workpiece_held_by ?w2 ?r)
            (workpiece_held_by ?w3 ?r)
          ))
          (at end (and 
            (is_free ?r)
            (workpiece_orientation ?w1 ?to)
            (workpiece_orientation ?w2 ?to)
            (workpiece_orientation ?w3 ?to)
            (not (workpiece_held_by ?w1 ?r))
            (not (workpiece_held_by ?w2 ?r))
            (not (workpiece_held_by ?w3 ?r))
          ))
      )
  )
  
    

  (:durative-action weld
    :parameters (?j - joint ?w1 ?w2 - workpiece)
    :duration (= ?duration 10)
    :condition (and (over all (and 
            (not_welded ?j)
            (not (= ?w1 ?w2))
            (seam_measured ?j)
            (workpiece_validated ?w1)
            (workpiece_validated ?w2)
            (has_joint ?w1 ?j)
            (has_joint ?w2 ?j)
            (workpiece_orientation ?w1 ?j)
            (workpiece_orientation ?w2 ?j)
            (workpiece_held ?w1)
            (workpiece_held ?w2)
            ; (not_workpiece_moved ?w1)
            ; (not_workpiece_moved ?w2)
            (forall (?j2 - joint) (imply (depends_on ?j ?j2) (welded ?j2)))
          ))
        )
    :effect (and 
          (at start (and
            (fused ?w1 ?w2)
            (fused ?w2 ?w1)
            (not (not_fused ?w1 ?w2))
            (not (not_fused ?w2 ?w1))
            ; (not (workpiece_unjoined ?w1))
            ; (not (workpiece_unjoined ?w2))
          ))
          (at end (and 
                (welded ?j)
                (not (not_welded ?j))
          ))
        )
  )
    
  (:durative-action hold
      :parameters (?w - workpiece ?r - robot)
      :duration (= ?duration 1)
      :condition(and
            (at start (not_workpiece_held ?w))
            (at start (is_free ?r))
          )
      :effect(and 
            (at start (and
              (not (is_free ?r))
              (workpiece_held_by ?w ?r)
            ))
            (at end (and
              (not_workpiece_moved ?w)
              (workpiece_held ?w)
              (not (not_workpiece_held ?w))
            ))
          )
  )

  (:durative-action release
      :parameters (?w - workpiece ?r - robot)
      :duration (= ?duration 1)
      :condition(and 
            (at start (workpiece_held ?w))
            (at start (workpiece_held_by ?w ?r))
          )
      :effect(and
            (at start (and
              (not (not_workpiece_moved ?w))
              (not (workpiece_validated ?w))
            ))
            (at end (and 
              (not (workpiece_held ?w))
              (not (workpiece_held_by ?w ?r))
              (not_workpiece_held ?w)
              (is_free ?r)
            ))
          )
  )
  
  (:durative-action validate
      :parameters (?j - joint ?w1 ?w2 - workpiece)
      :duration (= ?duration 4)
      :condition (and 
          (at start (and 
              (not_seam_measured ?j)
              (workpiece_orientation ?w1 ?j)
              (workpiece_orientation ?w2 ?j)
              (workpiece_held ?w1)
              (workpiece_held ?w2)              
          ))
          (over all (and
              (not_welded ?j)
              (not (= ?w1 ?w2))
              (has_joint ?w1 ?j)
              (has_joint ?w2 ?j)
              (workpiece_orientation ?w1 ?j)
              (workpiece_orientation ?w2 ?j)
              (workpiece_held ?w1)
              (workpiece_held ?w2)
              (not_workpiece_moved ?w1)
              (not_workpiece_moved ?w2)
          ))
      )
      :effect (and 
          (at end (and 
              (seam_measured ?j)
              (not (not_seam_measured ?j))
              (workpiece_validated ?w1)
              (workpiece_validated ?w2)
          ))
      )
  )
  
  
)
