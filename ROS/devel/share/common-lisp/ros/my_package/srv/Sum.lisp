; Auto-generated. Do not edit!


(cl:in-package my_package-srv)


;//! \htmlinclude Sum-request.msg.html

(cl:defclass <Sum-request> (roslisp-msg-protocol:ros-message)
  ((nums
    :reader nums
    :initarg :nums
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass Sum-request (<Sum-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Sum-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Sum-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name my_package-srv:<Sum-request> is deprecated: use my_package-srv:Sum-request instead.")))

(cl:ensure-generic-function 'nums-val :lambda-list '(m))
(cl:defmethod nums-val ((m <Sum-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_package-srv:nums-val is deprecated.  Use my_package-srv:nums instead.")
  (nums m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Sum-request>) ostream)
  "Serializes a message object of type '<Sum-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'nums))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'nums))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Sum-request>) istream)
  "Deserializes a message object of type '<Sum-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'nums) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'nums)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Sum-request>)))
  "Returns string type for a service object of type '<Sum-request>"
  "my_package/SumRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Sum-request)))
  "Returns string type for a service object of type 'Sum-request"
  "my_package/SumRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Sum-request>)))
  "Returns md5sum for a message object of type '<Sum-request>"
  "ea07415f981ad352cedb6941ee6c9fd6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Sum-request)))
  "Returns md5sum for a message object of type 'Sum-request"
  "ea07415f981ad352cedb6941ee6c9fd6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Sum-request>)))
  "Returns full string definition for message of type '<Sum-request>"
  (cl:format cl:nil "int32[] nums~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Sum-request)))
  "Returns full string definition for message of type 'Sum-request"
  (cl:format cl:nil "int32[] nums~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Sum-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'nums) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Sum-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Sum-request
    (cl:cons ':nums (nums msg))
))
;//! \htmlinclude Sum-response.msg.html

(cl:defclass <Sum-response> (roslisp-msg-protocol:ros-message)
  ((sum
    :reader sum
    :initarg :sum
    :type cl:integer
    :initform 0))
)

(cl:defclass Sum-response (<Sum-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Sum-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Sum-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name my_package-srv:<Sum-response> is deprecated: use my_package-srv:Sum-response instead.")))

(cl:ensure-generic-function 'sum-val :lambda-list '(m))
(cl:defmethod sum-val ((m <Sum-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_package-srv:sum-val is deprecated.  Use my_package-srv:sum instead.")
  (sum m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Sum-response>) ostream)
  "Serializes a message object of type '<Sum-response>"
  (cl:let* ((signed (cl:slot-value msg 'sum)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Sum-response>) istream)
  "Deserializes a message object of type '<Sum-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sum) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Sum-response>)))
  "Returns string type for a service object of type '<Sum-response>"
  "my_package/SumResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Sum-response)))
  "Returns string type for a service object of type 'Sum-response"
  "my_package/SumResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Sum-response>)))
  "Returns md5sum for a message object of type '<Sum-response>"
  "ea07415f981ad352cedb6941ee6c9fd6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Sum-response)))
  "Returns md5sum for a message object of type 'Sum-response"
  "ea07415f981ad352cedb6941ee6c9fd6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Sum-response>)))
  "Returns full string definition for message of type '<Sum-response>"
  (cl:format cl:nil "int32 sum~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Sum-response)))
  "Returns full string definition for message of type 'Sum-response"
  (cl:format cl:nil "int32 sum~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Sum-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Sum-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Sum-response
    (cl:cons ':sum (sum msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Sum)))
  'Sum-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Sum)))
  'Sum-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Sum)))
  "Returns string type for a service object of type '<Sum>"
  "my_package/Sum")