;; Auto-generated. Do not edit!


(when (boundp 'exercise1::Greeting)
  (if (not (find-package "EXERCISE1"))
    (make-package "EXERCISE1"))
  (shadow 'Greeting (find-package "EXERCISE1")))
(unless (find-package "EXERCISE1::GREETING")
  (make-package "EXERCISE1::GREETING"))

(in-package "ROS")
;;//! \htmlinclude Greeting.msg.html


(defclass exercise1::Greeting
  :super ros::object
  :slots (_content ))

(defmethod exercise1::Greeting
  (:init
   (&key
    ((:content __content) "")
    )
   (send-super :init)
   (setq _content (string __content))
   self)
  (:content
   (&optional __content)
   (if __content (setq _content __content)) _content)
  (:serialization-length
   ()
   (+
    ;; string _content
    4 (length _content)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _content
       (write-long (length _content) s) (princ _content s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _content
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _content (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get exercise1::Greeting :md5sum-) "c2e84951ee6d0addf437bfddd5b19734")
(setf (get exercise1::Greeting :datatype-) "exercise1/Greeting")
(setf (get exercise1::Greeting :definition-)
      "string content


")



(provide :exercise1/Greeting "c2e84951ee6d0addf437bfddd5b19734")


