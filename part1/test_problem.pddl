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
        ;(robot-hand-empty franka)
        ;(robot-near franka sugar_box)
        ;(robot-near franka countertop)
        ;(item-in-holder sugar_box countertop)
        ; item-holder-closed is false

        (robot-near franka countertop)

        
    )

    (:goal (and
        ;(item-in-grasp sugar_box franka)
        (robot-near franka indigo_drawer_top)


    ))
)