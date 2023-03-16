; Auto-generated. Do not edit!


(cl:in-package exercise1-msg)


;//! \htmlinclude Greeting.msg.html

(cl:defclass <Greeting> (roslisp-msg-protocol:ros-message)
  ((content
    :reader content
    :initarg :content
    :type cl:string
    :initform ""))
)

(cl:defclass Greeting (<Greeting>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Greeting>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Greeting)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name exercise1-msg:<Greeting> is deprecated: use exercise1-msg:Greeting instead.")))

(cl:ensure-generic-function 'content-val :lambda-list '(m))
(cl:defmethod content-val ((m <Greeting>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader exercise1-msg:content-val is deprecated.  Use exercise1-msg:content instead.")
  (content m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Greeting>) ostream)
  "Serializes a message object of type '<Greeting>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'content))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'content))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Greeting>) istream)
  "Deserializes a message object of type '<Greeting>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'content) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'content) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Greeting>)))
  "Returns string type for a message object of type '<Greeting>"
  "exercise1/Greeting")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Greeting)))
  "Returns string type for a message object of type 'Greeting"
  "exercise1/Greeting")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Greeting>)))
  "Returns md5sum for a message object of type '<Greeting>"
  "c2e84951ee6d0addf437bfddd5b19734")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Greeting)))
  "Returns md5sum for a message object of type 'Greeting"
  "c2e84951ee6d0addf437bfddd5b19734")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Greeting>)))
  "Returns full string definition for message of type '<Greeting>"
  (cl:format cl:nil "string content~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Greeting)))
  "Returns full string definition for message of type 'Greeting"
  (cl:format cl:nil "string content~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Greeting>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'content))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Greeting>))
  "Converts a ROS message object to a list"
  (cl:list 'Greeting
    (cl:cons ':content (content msg))
))
