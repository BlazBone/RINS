;; Auto-generated. Do not edit!


(when (boundp 'my_package::Sum)
  (if (not (find-package "MY_PACKAGE"))
    (make-package "MY_PACKAGE"))
  (shadow 'Sum (find-package "MY_PACKAGE")))
(unless (find-package "MY_PACKAGE::SUM")
  (make-package "MY_PACKAGE::SUM"))
(unless (find-package "MY_PACKAGE::SUMREQUEST")
  (make-package "MY_PACKAGE::SUMREQUEST"))
(unless (find-package "MY_PACKAGE::SUMRESPONSE")
  (make-package "MY_PACKAGE::SUMRESPONSE"))

(in-package "ROS")





(defclass my_package::SumRequest
  :super ros::object
  :slots (_nums ))

(defmethod my_package::SumRequest
  (:init
   (&key
    ((:nums __nums) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _nums __nums)
   self)
  (:nums
   (&optional __nums)
   (if __nums (setq _nums __nums)) _nums)
  (:serialization-length
   ()
   (+
    ;; int32[] _nums
    (* 4    (length _nums)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32[] _nums
     (write-long (length _nums) s)
     (dotimes (i (length _nums))
       (write-long (elt _nums i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32[] _nums
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _nums (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _nums i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(defclass my_package::SumResponse
  :super ros::object
  :slots (_sum ))

(defmethod my_package::SumResponse
  (:init
   (&key
    ((:sum __sum) 0)
    )
   (send-super :init)
   (setq _sum (round __sum))
   self)
  (:sum
   (&optional __sum)
   (if __sum (setq _sum __sum)) _sum)
  (:serialization-length
   ()
   (+
    ;; int32 _sum
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _sum
       (write-long _sum s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _sum
     (setq _sum (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass my_package::Sum
  :super ros::object
  :slots ())

(setf (get my_package::Sum :md5sum-) "ea07415f981ad352cedb6941ee6c9fd6")
(setf (get my_package::Sum :datatype-) "my_package/Sum")
(setf (get my_package::Sum :request) my_package::SumRequest)
(setf (get my_package::Sum :response) my_package::SumResponse)

(defmethod my_package::SumRequest
  (:response () (instance my_package::SumResponse :init)))

(setf (get my_package::SumRequest :md5sum-) "ea07415f981ad352cedb6941ee6c9fd6")
(setf (get my_package::SumRequest :datatype-) "my_package/SumRequest")
(setf (get my_package::SumRequest :definition-)
      "int32[] nums
---
int32 sum
")

(setf (get my_package::SumResponse :md5sum-) "ea07415f981ad352cedb6941ee6c9fd6")
(setf (get my_package::SumResponse :datatype-) "my_package/SumResponse")
(setf (get my_package::SumResponse :definition-)
      "int32[] nums
---
int32 sum
")



(provide :my_package/Sum "ea07415f981ad352cedb6941ee6c9fd6")


