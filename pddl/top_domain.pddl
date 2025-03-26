(define (domain welding-top)
  (:requirements :strips :typing :adl :durative-actions :fluents :conditional-effects)
  (:types joint position)

  (:predicates
    ;; Indicates that a joint has already been welded.
    (not_welded ?j - joint)
    (welded ?j - joint)
    ;; Indicates that a joint is reachable from the robot’s current position.
    (reachable_at ?j - joint ?p - position)
    ;; Indicates that a joint is scheduled (commanded) to be welded.
    (commanded ?j - joint)
    (commandable ?j - joint)
    ;; Execution status
    (not_executed)
    (executed)
    ;; Indicates the current position of a welding robot.
    (at ?p - position)
    ;; Indicates dependency between two joints.
    (depends_on ?j1 ?j2 - joint)
  )

  ; Updating of welding commands by addition or ommission of joints is to be handled by a seperate thread and not part of the planning problem.

  
  (:durative-action command
      :parameters (?j - joint ?p - position)
      :duration (= ?duration 1)
      :condition (and 
          (over all (and 
            (commandable ?j)
          ))
      )
      :effect (and 
          (at end (and 
            (commanded ?j)
          ))
      )
  )

  (:durative-action set_commandable
      :parameters (?j - joint ?p - position)
      :duration (= ?duration 1)
      :condition (and 
          (over all (and 
            (not_executed)
            (at ?p)
            (not_welded ?j)
            (reachable_at ?j ?p)
            (forall (?j2 - joint) (imply (depends_on ?j ?j2) (commanded ?j2)))        
          ))
      )
      :effect (and 
          (at end (and 
            (commandable ?j)
          ))
      )
  )
  
  

  ;; This durative action models an external call to the bottom planner
  ;; to weld a single joint. It requires that the joint is reachable,
  ;; commanded, and not yet welded. 
  (:durative-action execute
    :parameters (?p - position)
    :duration (= ?duration 20)
    :condition (and 
                (over all (and
                  (at ?p)
                  (not_executed)
                  (forall (?j - joint)  
                    (imply (reachable_at ?j ?p) (commandable ?j))  
                  )
                ))
              )
    :effect (at end (and
      (executed)
      (not (not_executed))  
    ))
  )

  ;; This durative action sets the status of welded joints after the call to the bottom planner.
  (:durative-action set_status
      :parameters (?j - joint)
      :duration (= ?duration 1)
      :condition (and 
          (over all (and 
            (commanded ?j)
            (executed)
          ))
      )
      :effect (and 
          (at end (and 
            (welded ?j)
            (not (not_welded ?j))
          ))
      )
  )
  
  ;; This durative action moves the welding robot from one position to another.
  (:durative-action move_robot
    :parameters (?from ?to - position)
    :duration (= ?duration 5)
    :condition (and (at start (at ?from)))
    :effect (and 
              (at start (and
                (not (at ?from))
                (not_executed)
                (not (executed))
              ))
              (at end (at ?to)))
  )
)
