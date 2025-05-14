(define (domain combined_domain)
  (:requirements :strips :typing :adl :fluents :durative-actions :negative-preconditions :conditional-effects)
  
  (:types 
    joint workpiece
  )
  
  (:predicates
    (workpiece_orientation ?w - workpiece ?j - joint)
    (workpiece_held ?w - workpiece)
    (has_joint ?w - workpiece ?j - joint)
    (joint_measured ?j - joint)
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
            (forall (?w2 - workpiece)
              (imply (and (has_joint ?w2 ?to)) (workpiece_orientation ?w2 ?from)) ; problematic. Cant remove
            )
            (forall (?w2 ?w3 - workpiece)
              (or
                (imply (or (fused ?w ?w2) (and (fused ?w ?w3) (fused ?w3 ?w2))) (workpiece_orientation ?w2 ?from))
                (imply (or (fused ?w ?w3) (and (fused ?w ?w2) (fused ?w2 ?w3))) (workpiece_orientation ?w3 ?from))
              )
            )
          ))
          (over all (and
            (forall (?w2 ?w3 - workpiece)
              (or
                (imply (or (fused ?w ?w2) (and (fused ?w ?w3) (fused ?w3 ?w2))) (not (workpiece_held ?w2)))
                (imply (or (fused ?w ?w3) (and (fused ?w ?w2) (fused ?w2 ?w3))) (not (workpiece_held ?w3)))
              )
            )
            (not (workpiece_held ?w))
            (has_joint ?w ?from)
            (has_joint ?w ?to)
          ))
          
    )
            
      
    :effect (and
        (at start (decrease (robots_available) (robots_needed ?w)))
        (at start (not (workpiece_orientation ?w ?from)))
        (at end(increase (robots_available) (robots_needed ?w)))
        (at end (workpiece_orientation ?w ?to))
        ; (forall (?w2 - workpiece)
        ;   (and 
        ;     (when (over all (fused ?w ?w2))
        ;     (at start (not (workpiece_orientation ?w2 ?from)))
        ;     )
        ;     (when (over all (fused ?w ?w2))
        ;       (at end (workpiece_orientation ?w2 ?to))
        ;     )
        ;   )
        ; )
        (forall (?w2 ?w3 - workpiece)
          (and
            (when (over all (fused ?w ?w2))
              (at start (not (workpiece_orientation ?w2 ?from)))
            )
            (when (over all (fused ?w ?w2))
              (at end (workpiece_orientation ?w2 ?to))
            )
            ; (when (at end (or (fused ?w ?W2) (and (fused ?w ?w3) (fused ?w3 ?w2))))
            ;   (at start (not (workpiece_orientation ?w2 ?from)))
            ; )
            ; (when (at start (or (fused ?w ?w2) (and (fused ?w ?w3) (fused ?w3 ?w2))))
            ;   (at end (workpiece_orientation ?w2 ?to))
            ; )
            ; (when (at end (or (fused ?w ?w3) (and (fused ?w ?w2) (fused ?w2 ?w3))))
            ;   (at start (not (workpiece_orientation ?w3 ?from)))
            ; )
            ; (when (at start (or (fused ?w ?w3) (and (fused ?w ?w2) (fused ?w2 ?w3))))
            ;   (at end (workpiece_orientation ?w3 ?to))
            ; )
          )
        )
        ; (forall (?w2 - workpiece)
        ;   (forall (?w3 - workpiece)
        ;     (and
        ;       (when (at start (and (not (= ?w2 ?w3)) (not (= ?w ?w2)) (fused ?w ?w2) (fused ?w2 ?w3)))
        ;         (at start (not (workpiece_orientation ?w3 ?from)))
        ;       )
        ;       (when (at start (and (not (= ?w2 ?w3)) (not (= ?w ?w2)) (fused ?w ?w2) (fused ?w2 ?w3)))
        ;         (at end (workpiece_orientation ?w3 ?to))
        ;       )
        ;     )
            
        ;   )
        ; )
    )
  )

  ; (:durative-action transit
  ;     :parameters (?w - workpiece ?from ?to - joint)
  ;     :duration (= ?duration 5)
  ;     :condition (and 
  ;         (at start (and
  ;           (workpiece_orientation ?w ?from)
  ;           (>= (robots_available) (robots_needed ?w))
  ;           (forall (?w2 - workpiece)
  ;             (imply (and (has_joint ?w2 ?to)) (workpiece_orientation ?w2 ?from)) ; problematic
  ;           )
  ;           (forall (?w2 - workpiece)
  ;             (forall (?w3 - workpiece)
  ;               (imply (and (fused ?w2 ?w3) (fused ?w ?w2)) (workpiece_orientation ?w3 ?from))
  ;             )
  ;           )
  ;         ))
  ;         (over all (and
  ;           (not (workpiece_held ?w))
  ;           (has_joint ?w ?to)
  ;           (has_joint ?w ?from)
  ;           (forall (?w2 - workpiece)
  ;             (imply (fused ?w ?w2) (not (workpiece_held ?w2)))
  ;           )
  ;           (forall (?w2 - workpiece)
  ;             (forall (?w3 - workpiece)
  ;               (imply (and (fused ?w2 ?w3) (fused ?w ?w2)) (not (workpiece_held ?w3)))
  ;             )
  ;           )
  ;         ))
  ;     )
  ;     :effect (and 
  ;         (at start (decrease (robots_available) (robots_needed ?w)))
  ;         (at start (not (workpiece_orientation ?w ?from)))
  ;         (forall (?w2 - workpiece)
  ;           (when (at start (fused ?w ?w2))
  ;             (at start (not (workpiece_orientation ?w2 ?from)))
  ;           )
  ;         )
  ;         (forall (?w2 - workpiece)
  ;           (forall (?w3 - workpiece)
  ;             (when (at start (and (fused ?w ?w2) (fused ?w2 ?w3)))
  ;               (at start (not (workpiece_orientation ?w3 ?from)))
  ;             )
  ;           )
  ;         )
  ;         (at end (workpiece_orientation ?w ?to))
  ;         (at end (increase (robots_available) (robots_needed ?w)))
  ;         (forall (?w2 - workpiece)
  ;           (when (at start (fused ?w ?w2))
  ;             (at end (workpiece_orientation ?w2 ?to))
  ;           )
  ;         )
  ;         (forall (?w2 - workpiece)
  ;           (forall (?w3 - workpiece)
  ;             (when (at start (and (fused ?w ?w2) (fused ?w2 ?w3)))
  ;               (at end (workpiece_orientation ?w3 ?to))
  ;             )
  ;           )
  ;         )
  ;     )
  ; )
  

  (:durative-action weld
    :parameters (?j - joint)
    :duration (= ?duration 10)
    :condition (and (over all (and 
            (joint_measured ?j)
            (forall (?j2 - joint) (imply (depends_on ?j ?j2) (welded ?j2)))
            (forall (?w - workpiece) (imply (has_joint ?w ?j) (workpiece_orientation ?w ?j)))
            (not (welded ?j))
            (forall (?w - workpiece) (imply (has_joint ?w ?j) (workpiece_held ?w)))
          ))
          ; (at start (>= (robots_available) 2))
        )
    :effect (and 
          ; (at start (decrease (robots_available) 2))
          (at end (and 
                (welded ?j)
                (increase (robots_available) 2) ; problematic. Cant remove
          ))
          (forall (?w1 ?w2 - workpiece)
            (when (and (at start (has_joint ?w1 ?j)) (at start (has_joint ?w2 ?j)) (and (at start (not (= ?w1 ?w2))) (at end (not (= ?w1 ?w2)))))
              (at end (and (fused ?w1 ?w2) (fused ?w2 ?w1)))
            )
          )
          ; (forall (?w1 - workpiece)
          ;   (when (at start (has_joint ?w1 ?j))
          ;     (forall (?w2 - workpiece)
          ;       (when (at start (has_joint ?w2 ?j))
          ;         (at end (and (fused ?w1 ?w2) (fused ?w2 ?w1)))
          ;       )
          ;     )
          ;   )
          ; )
        )
  )
    
  (:durative-action hold
      :parameters (?w - workpiece)
      :duration (= ?duration 1)
      :condition(and
            (at start (not (workpiece_held ?w)))
            (at start (>= (robots_available) (robots_needed ?w)))
          )
      :effect(and 
            (at start (and
              (not (workpiece_held ?w))
              (decrease (robots_available) (robots_needed ?w))
            ))
            (at end (and 
              (workpiece_held ?w)
            ))
          )
  )

  (:durative-action release
      :parameters (?w - workpiece)
      :duration (= ?duration 1)
      :condition(and 
            (at start (workpiece_held ?w))
          )
      :effect(and
            (at start (workpiece_held ?w))
            (at end (and 
              (not (workpiece_held ?w))
              (increase (robots_available) (robots_needed ?w))
            ))
          )
  )
  
  (:durative-action validate
      :parameters (?j - joint)
      :duration (= ?duration 4)
      :condition (and 
          (at start (and 
              (not (welded ?j))
              (not (joint_measured ?j))
              ; (>= (robots_available) 2)
          ))
          (over all (forall (?w - workpiece) (and
              (imply (has_joint ?w ?j) (workpiece_held ?w))
              (imply (has_joint ?w ?j) (workpiece_orientation ?w ?j)
          ))))
      )
      :effect (and 
          ; (at start (decrease (robots_available) 2))
          (at end (and 
              (joint_measured ?j)
              ; (increase (robots_available) 2)
          ))
      )
  )
  
  
)
