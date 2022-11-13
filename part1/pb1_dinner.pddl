(define (problem pb1_dinner)
  (:domain dinner)
  (:init
    (garbage)
    (clean)
    (quiet)
  )
  (:goal (and
    (dinner)
    (present)
    (not (garbage))
  ))
)