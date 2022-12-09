(define (domain kitchen)
    (:requirements :strips :typing :negative-preconditions)
    (:types
        robot - object
        environment - object
        item item_holder item_holder_manipulator - environment
        surface drawer - item_holder
        handle - item_holder_manipulator
    )

    (:predicates
        (robot-near ?r - robot ?e - environment)    
        (robot-hand-empty ?r - robot)
        (item-holder-closed ?ih - item_holder)
        (item-holder-full ?ih - item_holder)
        (can-manipulate ?m - item_holder_manipulator ?ih - item_holder )
        (item-in-holder ?i - item ?ih - item_holder) 
        (item-in-grasp ?i - item ?r - robot) ; Can we use a 'null' item to remove the need for 'robot-hand-empty'?
    )
    
    ; Functional
    (:action NAVIGATE
        :parameters (?r - robot ?source - environment ?destination - environment)
        :precondition (and (not (robot-near ?r ?destination)) (robot-near ?r ?source))
        :effect (and (not (robot-near ?r ?source)) (robot-near ?r ?destination))
    )
    ; Functional
    (:action OPEN-DRAWER
        :parameters (?r - robot ?d - item_holder ?m - item_holder_manipulator)
        :precondition (and
            (robot-near ?r ?m) ; robot is near the drawer
            (robot-hand-empty ?r) ; robot is not currently grasping an item
            (can-manipulate ?m ?d)
        )   
        :effect (and
            (not (item-holder-closed ?d)) ; drawer is open
        )
    )
    ; Functional
    (:action CLOSE-DRAWER
        :parameters (?r - robot ?d - item_holder ?m - item_holder_manipulator)
        :precondition (and
            (robot-near ?r ?m) ; robot is near the drawer
            (robot-hand-empty ?r) ; robot is not currently grasping an item
        )   
        :effect (and
            (item-holder-closed ?d) ; drawer is closed
            (can-manipulate ?m ?d)
        )
    )
    ; Functional
    (:action PICK-UP
        :parameters (?r - robot ?i - item ?ih - item_holder)
        :precondition (and
            (robot-near ?r ?ih) ; robot is near the item-holder it wishes to grab from

            (robot-hand-empty ?r) ; robot can pick something up
            (item-in-holder ?i ?ih) ; the item is in the ih
            (not (item-holder-closed ?ih)) ; the item holder is open (if drawer)
            ;(item-holder-full ?ih) ; the item holder should be full
        )   
        :effect (and
            (not (robot-hand-empty ?r)) ; robot is now carrying something
            (item-in-grasp ?i ?r)
            (not (item-in-holder ?i ?ih))
            (not (item-holder-full ?ih))
            
        )
    )
    ; Functional
    (:action PLACE
        :parameters (?r - robot ?i - item ?ih - item_holder)
        :precondition (and
            (robot-near ?r ?ih) ; robot is near the item-holder it wishes to interact with
            
            ;(not (robot-hand-empty ?r)) ; robot has something to put down
            (item-in-grasp ?i ?r) ; the item is in the robots grasp
            (not (item-holder-closed ?ih)) ; the item holder is open (if drawer)
            (not (item-holder-full ?ih)) ; the item holder is not full
        )   
        :effect (and
            (robot-hand-empty ?r) ; robot is now carrying something
            (not (item-in-grasp ?i ?r))
            (item-in-holder ?i ?ih)
            (item-holder-full ?ih)
            
        )
    )


)

