(define
    (domain kitchen)
    (:requirements :typing :strips :negative-preconditions)
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
    (item-in-holder ?i - item ?ih - item-holder)
    (item-in-grasp ?i - item ?r - robot)
    )

    ;(drawer-open ?d - drawer)
    ;(item-on-burner ?i - item)
    ;(item-on-counter ?i - item)
    ;(item-in-drawer ?i - item)
    ;(item-holder-available ?ih - item_holder)
    

    ; ACTIONS
    (:action NAVIGATE
    :parameters (?r - robot ?source - environment ?destination - environment)
    :precondition (and
        (not (robot-near ?r ?destination)) ; not already at final location
        (robot-near ?r ?source) ; at source
    )
    :effect (and
        (not (robot-near ?r ?source)) ; not at old location
        (robot-near ?r ?destination) ; at final location 
    )
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
    )
    )

    (:action PLACE
    :parameters (?r - robot ?i - item ?ih - item-holder)
    :precondition (and
        (not (robot-hand-empty ?r)) ; robot has something in arm
        ; item holder does not have an item in it
        (forall (?its - item)
            (not (item-in-holder ?its ?ih))  
        )
        (not (item-holder-closed ?ih)) ; Item holder must be open

    )   
    :effect (and
        (robot-hand-empty ?r) ; robot hand is empty
        (not (item-in-grasp ?i ?r)) ; item is no longer in grasp
        (item-in-holder ?i ?ih) ; item is now in holder
    )
    )

    ; NOTE: surfaces are item-holders, so they theoretically be "closed"
    ;   To get around this, no action exists allowing a surface to close.
    ;   The default init condition of item-holder-closed is false, which is good



















    ;(:action PICK_UP_SURFACE
    ;:parameters (?r - robot ?i - item ?s - surface)
    ;:precondition (and
    ;    (robot-hand-empty ?r) ; robot is not already grasping anything
    ;    (robot-near ?r ?i) ; robot is near item to pick up
    ;    (item-in-holder ?i ?s) ; an item is on the surface
    ;)
    ;:effect (and
    ;    (not (robot-hand-empty ?r)) ; robot hand is not empty
    ;    (item-in-grasp ?ri ?i) ; item is in grasp
    ;    (not (item-in-holder ?i ?s)); item is no longer in holder
    ;)
    ;)
    
    ;(:action PICK_UP_CLOSABLE
    ;:parameters (?r - robot ?i - item ?d - drawer)
    ;:precondition (and
    ;    (robot-hand-empty ?r) ; robot is not already grasping anything
    ;    (robot-near ?r ?i) ; robot is near item to pick up
    ;    (item-in-holder ?i ?d) ; an item is in the holder
    ;    (drawer-open ?d) ; the drawer is open

        ; if the item-holder is a drawer, the drawer must be open

    ;)
    ;:effect (and
    ;    (not (robot-hand-empty ?r)) ; robot hand is not empty
    ;    (item-in-grasp ?r ?i) ; item is in grasp
    ;    (not (item-in-holder ?i ?s)); item is no longer in holder
    ;)
    ;)




;    ; Place on surface
;    (:action PLACE_ON
;    :parameters (?r - robot ?i - item ?s - surface)
;    :precondition (and
;        (not (robot-hand-empty ?r)) ; robot has something in arm
;
 ;   )   
 ;   :effect (and
 ;       (robot-hand-empty ?r) ; robot hand is empty
 ;       (not (item-in-grasp ?r ?i)) ; item is no longer in grasp
 ;       (item-in-holder ?i ?ih) ; item is now in holder
 ;   )
 ;   )
 ;   
 ;   ; Place in closeable
 ;   (:action PLACE_IN_CLOSEABLE
 ;   :parameters (?r - robot ?i - item ?d - drawer)
 ;   :precondition (and
 ;       (not (robot-hand-empty ?r)) ; robot has something in arm
 ;       ()
 ;       ; item holder does not have an item in it
 ;       (forall (?its - item)
 ;           (not (item-in-holder ?its ?d))  
 ;       )
;
;        ; if the item-holder is a drawer, the drawer must be open
;
;
    ;)   
    ;:effect (and
    ;    (robot-hand-empty ?r) ; robot hand is empty
    ;    (not (item-in-grasp ?r ?i)) ; item is no longer in grasp
    ;    (item-in-holder ?i ?id) ; item is now in holder
    ;)
    ;)


    ;(:axiom
     ;   :vars (?d drawer)
      ;  :context (and ; drawer must be open and empty
       ;     (drawer-open ?d)
        ;    (forall (?i - item)
         ;       (not (item-in-holder ?i ?d))  
          ;  )
       ; )
       ; :implies (item-holder-available ?d)
    ;)

    ;(:axiom
     ;   :vars (?s surface)
      ;  :context (and ; drawer must be open and empty
       ;     (forall (?i - item)
        ;        (not (item-in-holder ?i ?s))  
         ;   )
        ;)
        ;:implies (item-holder-available ?s)
    ;)


    
)
