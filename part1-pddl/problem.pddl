; Oct 11, 2022

(define 
    (problem pdl)
    (:domain kitchen)
    (:objects
        spam_box sugar_box - item
        burner countertop - surface
        indigo_drawer_top - drawer
        robot_name - robot
    )
    (:init
        (robot-hand-empty robot_name)
        (item-in-holder sugar_box burner)
        (item-in-holder spam_box countertop)
        (item-holder-closed indigo_drawer_top)
    )
    (:goal (and
        (item-in-holder sugar_box countertop)
        (item-in-holder spam_box  indigo_drawer_top)
        (item-holder-closed indigo_drawer_top)
    ))
)


