(define (domain combined_domain_joints)
  (:requirements :strips :typing :adl :durative-actions :fluents :conditional-effects)
  
  (:types 
    joint
  )
  
  (:predicates
    (joint_orientation ?j - joint)
    (seam_measured ?j - joint)
    (not_seam_measured ?j - joint)
    (depends_on ?j1 ?j2 - joint)
    (welded ?j - joint)
    (not_welded ?j - joint)
  )

  (:durative-action transit
    :parameters (?from ?to - joint)
    :duration (= ?duration 1)
    :condition (and
          (at start (and
            (joint_orientation ?from)
          ))
          )
      
    :effect (and
        (at start (and
          (not (joint_orientation ?from))
          ; (not_seam_measured ?from)
          ; (not (seam_measured ?from))
        ))

        (at end (and
          (joint_orientation ?to)
        ))
    ) 
  )

  (:durative-action weld
    :parameters (?j - joint)
    :duration (= ?duration 1)
    :condition (and (over all (and 
            (not_welded ?j)
            (seam_measured ?j)
            (joint_orientation ?j)
          ))
        )
    :effect (and
          (at end (and 
                (welded ?j)
                (not (not_welded ?j))
          ))
        )
  )
  
  (:durative-action validate
      :parameters (?j - joint)
      :duration (= ?duration 1)
      :condition (and 
          (at start (and 
              (not_seam_measured ?j)
          ))
          (over all (and
              (not_welded ?j)
              (joint_orientation ?j)
              (forall (?j2 - joint) (imply (depends_on ?j ?j2) (welded ?j2)))
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
