
(define (domain emergency)
    (:requirements :strips :typing :equality)
    (:types agent box location person content veichle place)
    (:predicates (filled ?c - content ?b - box) 
                (empty ?b - box) 
                (hasContent ?p - person ?c - content) 
                (veichlePlace ?p - place ?v - veichle) 
                (placeAvailable ?p - place) 
                (placeOccupied ?p - place)
                (boxLoaded ?b - box ?v - veichle) 
                (at ?x - (either agent person box content veichle) ?l - location)
                (needContent ?c - content ?p - person) 
                (noContent ?p - person ?c1 - content ?c2 - content ) 
                (gotContent ?p - person ?c1 - content ?c2 - content ) 
    )

    (:action fill
        :parameters (?l - location ?c - content ?b - box ?a - agent)
        :precondition ( and (at ?c ?l)
                            (at ?b ?l)
                            (at ?a ?l)
                            (empty ?b))
        :effect ( and  (filled ?c ?b)
                       (not(empty ?b))
                )
    )

    (:action charge
            :parameters(?l - location ?b - box ?p - place ?v - veichle ?a - agent ?c - content)
            :precondition(and (placeAvailable ?p)
                              (veichlePlace ?p ?v)
                              (at ?b ?l)
                              (at ?a ?l)
                              (at ?v ?l)
                              (filled ?c ?b))  
            :effect(and(not(placeAvailable ?p))
                    (placeOccupied ?p)
                    (not (at ?b ?l))
                    (boxLoaded ?b ?v))
    )
  
    
    
    (:action moveAgent
        :parameters(?a - agent ?from ?to - location)
        :precondition( at ?a ?from )
        :effect(and (not(at ?a ?from))
                    (at ?a ?to))
    )

    (:action moveVeichle
        :parameters(?a - agent ?from ?to - location ?v - veichle)
        :precondition(and(at ?a ?from)
                         (at ?v ?from)
                      )
        :effect(and (not(at ?a ?from))
                    (at ?a ?to)
                    (not(at ?v ?from))
                    (at ?v ?to))
    )
    
       (:action discharge
            :parameters (?l - location ?b - box ?p - place ?v - veichle ?a - agent ?c - content)
            :precondition (and (placeOccupied ?p)
                            (at ?a ?l)
                            (at ?v ?l)
                            (boxLoaded ?b ?v)
                            (veichlePlace ?p ?v)
                            (empty ?b)
            )
            :effect ( and (placeAvailable ?p)
                          (not (placeOccupied ?p))
                          (at ?b ?l)
                          (not(boxLoaded ?b ?v))
            )
    )
    
    (:action giveContent
        :parameters(?l - location ?c - content

                    ?b - box ?a - agent ?p - person ?v - veichle)
        :precondition(and (filled ?c ?b)
                          (at ?a ?l)
                          (at ?p ?l)
                          (at ?v ?l)
                          (boxLoaded ?b ?v)
                          (needContent ?c ?p)
        )
        :effect(and (not(needContent ?c ?p))
                    (not(filled ?c ?b))
                    (empty ?b)
                    (hasContent ?p ?c))
    )
    
    (:action satisfiedWithOne
        :parameters(?c1 - content ?c2 - content ?p - person)
        :precondition(and

                          (hasContent ?p ?c1)
                          (noContent ?p ?c1 ?c2)
        )
        :effect(and

                    (gotContent ?p ?c1 ?c2)
                    (not (noContent ?p ?c1 ?c2))
                    (not (noContent ?p ?c2 ?c1)) 
                    (gotContent ?p ?c2 ?c1))
    )
   
)
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
