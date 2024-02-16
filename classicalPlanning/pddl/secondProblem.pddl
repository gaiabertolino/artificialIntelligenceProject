( define (problem emergencyDelivery)
    ( :domain emergency )
    ( :objects
        b1 b2 b3 - box
        dep loc1 loc2 loc3 loc4 loc5 - location
        p1 p2 p3 p4 p5 p6 - person
        a1 a2 - agent
        drugs food tools - content
        veic1 veic2 - veichle
        pl1veic1 pl2veic1 pl1veic2 pl2veic2 - place
    )

    ( :init

        ( at a1 dep )
        ( at a2 dep )

        ( at b1 dep )
        ( at b2 dep )
        ( at b3 dep )

        ( at veic1 dep )
        ( at veic2 dep )

        ( at food dep )
        ( at drugs dep )
        ( at tools dep )

        ( at p1 loc1 )
        ( at p2 loc1 )
        ( at p3 loc2 )
        ( at p4 loc3 )
        ( at p5 loc4 )
        ( at p6 loc5 )

        (empty b1)
        (empty b2)
        (empty b3)

        ( needContent food p1)
        ( needContent tools p1)
        (noContent p1 food tools)
        (noContent p1 tools food)
        ( needContent drugs p2)
        ( needContent drugs p3)
        ( needContent food p4)
        ( needContent drugs p4)
        ( needContent food p5)
        ( needContent drugs p5)
        ( needContent tools p5)
        ( needContent food p6)
        ( needContent drugs p6)
        ( needContent tools p6)
 
        ( placeAvailable pl1veic1 )
        ( placeAvailable pl2veic1 )
        ( placeAvailable pl1veic2 )
        ( placeAvailable pl2veic2 )

        ( veichlePlace pl1veic1 veic1 )
        ( veichlePlace pl2veic1 veic1 )
        ( veichlePlace pl1veic2 veic2 )
        ( veichlePlace pl2veic2 veic2 )
    )
    ( :goal

        (and

            ( gotContent p1 food tools)
            ( gotContent p1 tools food)
            ( hasContent p2 drugs )
            ( hasContent p3 drugs )
            ( hasContent p4 drugs )
            ( hasContent p4 food )
            ( hasContent p5 food )
            ( hasContent p5 drugs )
            ( hasContent p5 tools )
            ( hasContent p6 food )
            ( hasContent p6 tools )
            ( hasContent p6 drugs )
        )
    )
)