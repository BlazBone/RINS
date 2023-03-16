;; Auto-generated. Do not edit!


(when (boundp 'exercise1::Reverse)
  (if (not (find-package "EXERCISE1"))
    (make-package "EXERCISE1"))
  (shadow 'Reverse (find-package "EXERCISE1")))
(unless (find-package "EXERCISE1::REVERSE")
  (make-package "EXERCISE1::REVERSE"))
(unless (find-package "EXERCISE1::REVERSEREQUEST")
  (make-package "EXERCISE1::REVERSEREQUEST"))
(unless (find-package "EXERCISE1::REVERSERESPONSE")
  (make-package "EXERCISE1::REVERSERESPONSE"))

(in-package "ROS")





(defclass exercise1::ReverseRequest
  :super ros::object
  :slots (_content ))

(defmethod exercise1::ReverseRequest
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

(defclass exercise1::ReverseResponse
  :super ros::object
  :slots (_comment ))

(defmethod exercise1::ReverseResponse
  (:init
   (&key
    ((:comment __comment) "")
    )
   (send-super :init)
   (setq _comment (string __comment))
   self)
  (:comment
   (&optional __comment)
   (if __comment (setq _comment __comment)) _comment)
  (:serialization-length
   ()
   (+
    ;; string _comment
    4 (length _comment)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _comment
       (write-long (length _comment) s) (princ _comment s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _comment
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _comment (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass exercise1::Reverse
  :super ros::object
  :slots ())

(setf (get exercise1::Reverse :md5sum-) "a6ae0dfa99b6e1c22e0f4dd5d3d7311b")
(setf (get exercise1::Reverse :datatype-) "exercise1/Reverse")
(setf (get exercise1::Reverse :request) exercise1::ReverseRequest)
(setf (get exercise1::Reverse :response) exercise1::ReverseResponse)

(defmethod exercise1::ReverseRequest
  (:response () (instance exercise1::ReverseResponse :init)))

(setf (get exercise1::ReverseRequest :md5sum-) "a6ae0dfa99b6e1c22e0f4dd5d3d7311b")
(setf (get exercise1::ReverseRequest :datatype-) "exercise1/ReverseRequest")
(setf (get exercise1::ReverseRequest :definition-)
      "#request constants
#int8 FOO=1
#int8 BAR=2
#request fields
string content
#int32 y
#another_pkg/AnotherMessage msg
---
#response constants
#uint32 SECRET=123456
#response fields
#another_pkg/YetAnotherMessage val
#CustomMessageDefinedInThisPackage value
#int32 result
string comment
")

(setf (get exercise1::ReverseResponse :md5sum-) "a6ae0dfa99b6e1c22e0f4dd5d3d7311b")
(setf (get exercise1::ReverseResponse :datatype-) "exercise1/ReverseResponse")
(setf (get exercise1::ReverseResponse :definition-)
      "#request constants
#int8 FOO=1
#int8 BAR=2
#request fields
string content
#int32 y
#another_pkg/AnotherMessage msg
---
#response constants
#uint32 SECRET=123456
#response fields
#another_pkg/YetAnotherMessage val
#CustomMessageDefinedInThisPackage value
#int32 result
string comment
")



(provide :exercise1/Reverse "a6ae0dfa99b6e1c22e0f4dd5d3d7311b")


