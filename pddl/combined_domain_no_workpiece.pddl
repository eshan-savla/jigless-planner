(define (domain combined_domain_joints)
  (:requirements :strips :typing :adl :durative-actions :fluents :conditional-effects)
  
  (:types 
    joint
  )
  
  (:predicates
    (joint_orientation ?j - joint)
    (joint_measured ?j - joint)
    (not_joint_measured ?j - joint)
    (depends_on ?j1 ?j2 - joint)
    (welded ?j - joint)
    (not_welded ?j - joint)
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

  (:durative-action weld
    :parameters (?j - joint)
    :duration (= ?duration 10)
    :condition (and (over all (and 
            (not_welded ?j)
            (joint_measured ?j)
            (joint_orientation ?j)
          ))
        )
    :effect (and 
          (at end (and 
                (welded ?j)
                (not (not_welded ?j))
                (not_joint_measured ?j)
                (not (joint_measured ?j))
          ))
        )
  )
  
  (:durative-action validate
      :parameters (?j - joint)
      :duration (= ?duration 4)
      :condition (and 
          (at start (and 
              (not_joint_measured ?j)
          ))
          (over all (and
              (not_welded ?j)
              (joint_orientation ?j)
              (forall (?j2 - joint) (imply (depends_on ?j ?j2) (welded ?j2)))
          ))
      )
      :effect (and 
          (at end (and 
              (joint_measured ?j)
              (not (not_joint_measured ?j))
          ))
      )
  )
)
