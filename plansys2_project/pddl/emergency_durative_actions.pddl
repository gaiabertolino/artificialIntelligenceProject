(define (domain durativeEmergency)
  (:requirements :typing :adl :universal-preconditions :durative-actions :fluents)
  (:types agent box location person content vehicle place)
    (:predicates (filled ?c - content ?b - box)
                (empty ?b - box)
                (hasContent ?p - person ?c - content)
                (placeVehicle ?p - place ?v - vehicle)
                (placeAvailable ?p - place)
                (placeOccupied ?p - place)
                (boxloaded ?b - box ?v - vehicle)
                (at ?x - (either agent person box content vehicle) ?l - location)
                (needContent ?c - content ?p - person)
                (agentFree ?a - agent)
    )
  
  (:functions
    (weight ?c - content)
    (vehicleWeight ?v - vehicle)
    (boxWeight ?b - box)
    (pathCost ?a - agent)
  )
  
 (:durative-action fill
        :parameters(?l - location ?c - content ?b - box ?a - agent)
        :duration(= ?duration 1)
        :condition(and (at start (empty ?b))
                        (over all (at ?c ?l))
                        (over all (at ?b ?l))
                        (over all (at ?a ?l))
                        (at start (agentFree ?a))
        )
        :effect(and
                 (at start(not(agentFree ?a)))
                 (at end (agentFree ?a))
                 (at end (filled ?c ?b))
                 (at start (not(empty ?b)))
                 (at end (assign (boxWeight ?b) (weight ?c)))
                 (at end (increase (pathCost ?a)1))
        )
      )
 
    (:durative-action charge
            :parameters(?l - location ?a - agent
                ?b - box ?p - place ?v - vehicle)
            :duration(= ?duration 1)
            :condition(and 
                        (over all (at ?v ?l))
                        (at start (at ?b ?l))
                        (over all (at ?a ?l))
                        (over all (placeVehicle ?p ?v))
                        (at start (placeAvailable ?p))
                        (at start (agentFree ?a))
             )
            :effect(and
                 (at start(not(agentFree ?a)))
                 (at end (agentFree ?a))
                 (at start (not(at ?b ?l)))
                 (at end (boxloaded ?b ?v))
                 (at start(not(placeAvailable ?p)))
                 (at start (placeOccupied ?p))
                 (at end (increase (vehicleWeight ?v) (boxWeight ?b)))
                 (at end (increase (pathCost ?a)1))
        )
    )
    
    (:durative-action discharge
            :parameters(?l - location ?a - agent
                ?b - box ?p - place ?v - vehicle)
            :duration(= ?duration 1)
            :condition(and 
                        (over all (at ?v ?l))
                        (over all (at ?a ?l))
                        (at start(boxloaded ?b ?v))
                        (over all (placeVehicle ?p ?v))
                        (over all (placeOccupied ?p))
                        (over all (empty ?b))
                        (at start (agentFree ?a))
            )
            :effect(and
                 (at start(not(agentFree ?a)))
                 (at end (agentFree ?a))
                 (at end (at ?b ?l))
                 (at start (not(boxloaded ?b ?v)))
                 (at end(not(placeOccupied ?p)))
                 (at end (placeAvailable ?p))
                 
                 (at end (decrease(vehicleWeight ?v) (boxWeight ?b)))
                 (at end (decrease (pathCost ?a)1))
        )
    )

    (:durative-action move_vehicle
        :parameters(?a - agent ?from ?to - location ?v - vehicle)
        :duration(= ?duration ( *( vehicleWeight ?v ) 2))
        :condition(and
                    (at start (at ?a ?from))
                    (at start (at ?v ?from))
                    (at start(agentFree ?a)))
        :effect(and
                (at start (not(agentFree ?a)))
                (at end(agentFree ?a))
                (at start (not(at ?a ?from)))
                (at end (at ?a ?to))
                (at start (not(at ?v ?from)))
                (at end (at ?v ?to))
                (at end (increase (pathCost ?a)(*( vehicleWeight ?v ) 2)))
                )
    )
    
    (:durative-action move_agent
        :parameters(?a - agent ?from ?to - location )
        :duration(= ?duration 2)
        :condition(and
                    (at start (at ?a ?from))
                    (at start(agentFree ?a))
        )
        :effect(and
                (at start (not(agentFree ?a)))
                (at end(agentFree ?a))
                (at start (not(at ?a ?from)))
                (at end (at ?a ?to))
                (at end (increase (pathCost ?a) 2))
                )
    )

     (:durative-action give_content
        :parameters(?l - location ?c - content
                    ?b - box ?a - agent ?p - person ?v - vehicle)
                    
        :duration(= ?duration 1)
        
        :condition(and 
                    (at start(filled ?c ?b))
                    (over all (at ?v ?l))
                    (over all (at ?p ?l))
                    (over all (at ?a ?l))
                    (over all (boxLoaded ?b ?v))
                    (over all (needContent ?c ?p))
                    (at start (agentFree ?a))
        )
        :effect(and
                    (at start (not(agentFree ?a)))
                    (at end (agentFree ?a))
                    (at start (not(filled ?c ?b)))
                    (at end(empty ?b))
                    (at end (hasContent ?p ?c))
                    
                    (at end (assign (boxWeight ?b)0))
                    (at end (increase (pathCost ?a)1))
        )
    )
)





