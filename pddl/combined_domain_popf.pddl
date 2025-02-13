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
    (depends_on ?j1 ?j2 - joint)
    (welded ?j - joint)
    (not_welded ?j - joint)
    (fused ?w1 ?w2 - workpiece)
    (is_free ?r - robot)
    (workpiece_held_by ?w - workpiece ?r - robot)
  )

  ; (:functions 
  ;   (robots_available)
  ;   (robots_needed ?w - workpiece)
  ; )

  (:durative-action transit
    :parameters (?w - workpiece ?r - robot ?from ?to - joint)
    :duration (= ?duration 5)
    :condition (and
          (at start (and
            (workpiece_orientation ?w ?from)
            (is_free ?r)
          ))
          (over all (and
            (not_workpiece_held ?w)            
            (has_joint ?w ?from)
            (has_joint ?w ?to)
            (workpiece_held_by ?w ?r)
          ))
          )
            
      
    :effect (and
        (at start (not (is_free ?r)))
        (at start (not (workpiece_orientation ?w ?from)))
        (at start (workpiece_held_by ?w ?r))
        (at end(is_free ?r))
        (at end (workpiece_orientation ?w ?to))
        (at end (not (workpiece_held_by ?w ?r)))
    )
  )  

  (:durative-action weld
    :parameters (?j - joint ?w1 ?w2 - workpiece)
    :duration (= ?duration 10)
    :condition (and (over all (and 
            (not_welded ?j)
            (not (= ?w1 ?w2))
            (seam_measured ?j)
            (has_joint ?w1 ?j)
            (has_joint ?w2 ?j)
            (workpiece_orientation ?w1 ?j)
            (workpiece_orientation ?w2 ?j)
            (workpiece_held ?w1)
            (workpiece_held ?w2)
            (forall (?j2 - joint) (imply (depends_on ?j ?j2) (welded ?j2)))
          ))
        )
    :effect (and 
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
              (workpiece_held ?w)
              (workpiece_held_by ?w ?r)
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
          ))
      )
      :effect (and 
          (at end (and 
              (seam_measured ?j)
              (not (not_seam_measured ?j))
          ))
      )
  )
  
  
)
