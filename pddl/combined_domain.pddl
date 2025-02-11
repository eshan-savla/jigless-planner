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
    (fused ?w1 ?w2 - workpiece)
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
            (forall (?w2 - workpiece)
                (imply (and (has_joint ?w2 ?from) (has_joint ?w2 ?to)) (workpiece_orientation ?w2 ?from))
            )
            (has_joint ?w ?from)
            (has_joint ?w ?to)
            )
          )
          (over all (forall (?w2 - workpiece) 
            (imply (and (has_joint ?w2 ?from) (has_joint ?w2 ?to)) (transit_allowed ?w2))
          ))
          
    )
            
      
    :effect (and
        (at start (decrease (robots_available) (robots_needed ?w)))
        (at start (not (workpiece_orientation ?w ?from)))
        (at end(increase (robots_available) (robots_needed ?w)))
        (at end (workpiece_orientation ?w ?to))
        (forall (?w2 - workpiece)
          (and 
            (when (at start (fused ?w ?w2))
            (at start (not (workpiece_orientation ?w2 ?from)))
            )
            (when (at start (fused ?w ?w2))
              (at end (workpiece_orientation ?w2 ?to))
            )
          )
        )
        (forall (?w2 - workpiece)
          (forall (?w3 - workpiece)
            (and
              (when (at start (and (not (= ?w2 ?w3)) (not (= ?w ?w2)) (fused ?w2 ?w3)))
                (at start (not (workpiece_orientation ?w3 ?from)))
              )
              (when (at start (and (not (= ?w2 ?w3)) (not (= ?w ?w2)) (fused ?w2 ?w3)))
                (at end (workpiece_orientation ?w3 ?to))
              )
            )
            
          )
        )
    )
  )

  (:durative-action weld
    :parameters (?j - joint)
    :duration (= ?duration 10)
    :condition (and (over all (and 
            (seam_measured ?j)
            (forall (?j2 - joint) (imply (depends_on ?j ?j2) (welded ?j2)))
            (forall (?w - workpiece) (imply (has_joint ?w ?j) (workpiece_orientation ?w ?j)))
            (not (welded ?j))
            (forall (?w - workpiece) (imply (has_joint ?w ?j) (workpiece_held ?w)))
          ))
          (at start (>= (robots_available) 2))
    )
    :effect (and 
          (at start (decrease (robots_available) 2))
          (at end (and 
                (welded ?j)
                (increase (robots_available) 2)
          ))
          (forall (?w1 ?w2 - workpiece)(and
            (when (and (at start (has_joint ?w1 ?j)) (at start (has_joint ?w2 ?j)) (and (at start (not (= ?w1 ?w2))) (at end (not (= ?w1 ?w2)))))
              (at end (fused ?w1 ?w2))
            )
            (when (and (at start (has_joint ?w1 ?j)) (at start (has_joint ?w2 ?j)) (and (at start (not (= ?w1 ?w2))) (at end (not (= ?w1 ?w2)))))
              (at end (fused ?w2 ?w1))
            )
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
