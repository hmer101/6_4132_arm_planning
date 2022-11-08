(define (domain test_domain_2)
    (:requirements :strips :typing :negative-preconditions)
    (:types
        robot - object
        environment - object
        item item_holder - environment
        surface drawer - item_holder
    )

    (:predicates
        (robot-near ?r - robot ?e - environment)
        (robot-hand-empty ?r - robot)

        (item-holder-closed ?ih - item_holder)
        (item-holder-full ?ih - item_holder)
        
        (item-in-holder ?i - item ?ih - item-holder) 
        (item-in-grasp ?i - item ?r - robot) ; Can we use a 'null' item to remove the need for 'robot-hand-empty'?
    )
    
    ; Checked, works!
    (:action NAVIGATE
        :parameters (?r - robot ?source - environment ?destination - environment)
        :precondition (and (not (robot-near ?r ?destination)) (robot-near ?r ?source))
        :effect (and (not (robot-near ?r ?source)) (robot-near ?r ?destination))
    )


    (:action PICK_UP
    :parameters (?r - robot ?i - item ?ih - item-holder)
    :precondition (and
        ;(robot-hand-empty ?r) ; robot is not already grasping anything
        ;(robot-near ?r ?i) ; robot is near item to pick up
        ;(item-in-holder ?i ?ih) ; an item is in the holder
        ;(not(item-holder-closed ?ih)); the holder is open
    )
    :effect (and
        (not (robot-hand-empty ?r)) ; robot hand is not empty
        (item-in-grasp ?i ?r) ; item is in grasp
        (not (item-in-holder ?i ?ih)); item is no longer in holder
        (not (item-holder-full ?ih)) ; item holder is no longer full
    )
    )

    ; NOTE: surfaces are item-holders, so they theoretically be "closed"
    ;   To get around this, no action exists allowing a surface to close.
    ;   The default init condition of item-holder-closed is false, which is good
)

