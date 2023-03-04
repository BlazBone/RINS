;; Auto-generated. Do not edit!


(when (boundp 'my_package::MyMessage)
  (if (not (find-package "MY_PACKAGE"))
    (make-package "MY_PACKAGE"))
  (shadow 'MyMessage (find-package "MY_PACKAGE")))
(unless (find-package "MY_PACKAGE::MYMESSAGE")
  (make-package "MY_PACKAGE::MYMESSAGE"))

(in-package "ROS")
;;//! \htmlinclude MyMessage.msg.html


(defclass my_package::MyMessage
  :super ros::object
  :slots (_message _id ))

(defmethod my_package::MyMessage
  (:init
   (&key
    ((:message __message) "")
    ((:id __id) 0)
    )
   (send-super :init)
   (setq _message (string __message))
   (setq _id (round __id))
   self)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:serialization-length
   ()
   (+
    ;; string _message
    4 (length _message)
    ;; int32 _id
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;; int32 _id
       (write-long _id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get my_package::MyMessage :md5sum-) "693c355a78e66eabcabcc6b2654ed44e")
(setf (get my_package::MyMessage :datatype-) "my_package/MyMessage")
(setf (get my_package::MyMessage :definition-)
      "string message
int32 id
")



(provide :my_package/MyMessage "693c355a78e66eabcabcc6b2654ed44e")


