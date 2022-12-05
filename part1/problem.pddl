; Oct 11, 2022

(define 
    (problem move_boxes_kitchen)
    (:domain kitchen)
    (:objects
        spam_box sugar_box - item
        nowhere burner countertop - surface
        indigo_drawer - drawer
        indigo_drawer_handle - item_holder_manipulator
        franka - robot
    )
    (:init
        (robot-hand-empty franka)
        (robot-near franka nowhere)
        ; all item-holder-closed are false
        
        (item-in-holder sugar_box burner)
        (item-in-holder spam_box countertop)
        (item-holder-full burner)
        (item-holder-full countertop)
        (item-holder-closed indigo_drawer)
        (can-manipulate indigo_drawer_handle indigo_drawer)

    )
    (:goal (and
        (item-in-holder sugar_box countertop)
        (item-in-holder spam_box  indigo_drawer)
        (item-holder-closed indigo_drawer)
    ))
)