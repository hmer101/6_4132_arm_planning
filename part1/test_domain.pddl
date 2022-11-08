(define (domain test_domain)
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
    
    (:action NAVIGATE
        :parameters (?r - robot ?source - environment ?destination - environment)
        :precondition (and (not (robot-near ?r ?destination)) (robot-near ?r ?source))
        :effect (and (not (robot-near ?r ?source)) (robot-near ?r ?destination))
    )

    (:action OPEN-DRAWER
    :parameters (?r - robot ?d - drawer)
    :precondition (and
        (robot-near ?r ?d) ; robot is near the drawer
        (robot-hand-empty ?r) ; robot is not currently grasping an item
    )   
    :effect (and
        (not (item-holder-closed ?d)) ; drawer is open
    )
    )
    
    (:action CLOSE-DRAWER
    :parameters (?r - robot ?d - drawer)
    :precondition (and
        (robot-near ?r ?d) ; robot is near the drawer
        (robot-hand-empty ?r) ; robot is not currently grasping an item
    )   
    :effect (and
        (item-holder-closed ?d) ; drawer is closed
    )
    )

    (:action PICK_UP
    :parameters (?r - robot ?i - item ?ih - item-holder)
    :precondition (and
        (robot-hand-empty ?r) ; robot is not already grasping anything
        (robot-near ?r ?i) ; robot is near item to pick up
        (item-in-holder ?i ?ih) ; an item is in the holder
        (not(item-holder-closed ?ih)); the holder is open
    )
    :effect (and
        (not (robot-hand-empty ?r)) ; robot hand is not empty
        (item-in-grasp ?i ?r) ; item is in grasp
        (not (item-in-holder ?i ?ih)); item is no longer in holder
        (not (item-holder-full ?ih)) ; item holder is no longer full
    )
    )

    (:action PLACE
    :parameters (?r - robot ?i - item ?ih - item-holder)
    :precondition (and
        (not (robot-hand-empty ?r)) ; robot has something in arm
        (robot-near ?r ?ih) ; robot is near item holder to place in
        (not (item-holder-closed ?ih)) ; Item holder must be open
        (not (item-holder-full ?ih)) ; item holder does not have an item in it

        ;(forall (?its - item)
         ;   (not (item-in-holder ?its ?ih))  
        ;)  
    )   
    :effect (and
        (robot-hand-empty ?r) ; robot hand is empty
        (not (item-in-grasp ?i ?r)) ; item is no longer in grasp
        (item-in-holder ?i ?ih) ; item is now in holder
        (item-holder-full ?ih) ; item holder is now full
    )
    )

    ; NOTE: surfaces are item-holders, so they theoretically be "closed"
    ;   To get around this, no action exists allowing a surface to close.
    ;   The default init condition of item-holder-closed is false, which is good


)

