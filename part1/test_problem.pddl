; Oct 11, 2022

; TO DO: TEST EACH ACTION INDIVIDUALLY

(define (problem test_problem)
    (:domain test_domain_2)
    (:objects
        franka - robot
        burner countertop - surface
        spam_box sugar_box - item
        indigo_drawer_top - drawer
    )
    (:init
        
        (robot-near franka countertop)
        (robot-hand-empty franka)
        (item-holder-closed indigo_drawer_top)

        (item-holder-full countertop)
        (item-in-holder spam_box countertop)
        
        
    )

    (:goal (and

        (item-in-holder spam_box indigo_drawer_top)

    ))
)