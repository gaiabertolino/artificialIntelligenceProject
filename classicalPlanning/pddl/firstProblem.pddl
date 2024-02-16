( define (problem emergencyDelivery)
    ( :domain emergency )
    ( :objects
        dep loc1 loc2 - location
        b1 b2 b3 b4 b5 - box
        p1 p2 p3 - person
        a - agent
        veic - veichle
        pl1 pl2 pl3 pl4 - place
        drugs food tools - content
    )

    ( :init

        ( at a dep )
        ( at veic dep )

        ( at b1 dep )
        ( at b2 dep )
        ( at b3 dep )
        ( at b4 dep )
        ( at b5 dep )

        ( at food dep )
        ( at tools dep )
        ( at drugs dep )

        ( at p1 loc1 )
        ( at p2 loc1)
        ( at p3 loc2 )

        ( needContent food p1)
        ( needContent drugs p1)
        ( needContent drugs p2)
        ( needContent food p3)

        (empty b1)
        (empty b2)
        (empty b3)
        (empty b4)
        (empty b5)

        ( placeAvailable pl1 )
        ( placeAvailable pl2 )
        ( placeAvailable pl3 )
        ( placeAvailable pl4 )
        ( veichlePlace pl1 veic )
        ( veichlePlace pl2 veic )
        ( veichlePlace pl3 veic )
        ( veichlePlace pl4 veic )
    )
    ( :goal
        (and
            ( hasContent p1 food )
            ( hasContent p1 drugs )
            ( hasContent p2 drugs )
            ( hasContent p3 food ) 
        )

    )

)
