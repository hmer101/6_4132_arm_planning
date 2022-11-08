; Oct 11, 2022

; TO DO: TEST EACH ACTION INDIVIDUALLY

(define (problem test_problem)
    (:domain test_domain)
    (:objects
        franka - robot
        burner countertop - surface
        spam_box sugar_box - item
        indigo_drawer_top - drawer
    )
    (:init
        (robot-hand-empty franka)
        (robot-near franka countertop)
        ; all item-holder-closed are false

        (item-in-holder sugar_box countertop)
        ;(item-in-holder spam_box countertop)
        ;(item-holder-full burner)
        ;(item-holder-full countertop)
        ;(item-holder-closed indigo_drawer_top)
    )

    (:goal (and
        (item-holder-full franka)
    ))
)