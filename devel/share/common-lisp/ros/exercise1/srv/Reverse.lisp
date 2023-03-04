; Auto-generated. Do not edit!


(cl:in-package exercise1-srv)


;//! \htmlinclude Reverse-request.msg.html

(cl:defclass <Reverse-request> (roslisp-msg-protocol:ros-message)
  ((content
    :reader content
    :initarg :content
    :type cl:string
    :initform ""))
)

(cl:defclass Reverse-request (<Reverse-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Reverse-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Reverse-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name exercise1-srv:<Reverse-request> is deprecated: use exercise1-srv:Reverse-request instead.")))

(cl:ensure-generic-function 'content-val :lambda-list '(m))
(cl:defmethod content-val ((m <Reverse-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader exercise1-srv:content-val is deprecated.  Use exercise1-srv:content instead.")
  (content m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Reverse-request>) ostream)
  "Serializes a message object of type '<Reverse-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'content))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'content))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Reverse-request>) istream)
  "Deserializes a message object of type '<Reverse-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Reverse-request>)))
  "Returns string type for a service object of type '<Reverse-request>"
  "exercise1/ReverseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Reverse-request)))
  "Returns string type for a service object of type 'Reverse-request"
  "exercise1/ReverseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Reverse-request>)))
  "Returns md5sum for a message object of type '<Reverse-request>"
  "a6ae0dfa99b6e1c22e0f4dd5d3d7311b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Reverse-request)))
  "Returns md5sum for a message object of type 'Reverse-request"
  "a6ae0dfa99b6e1c22e0f4dd5d3d7311b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Reverse-request>)))
  "Returns full string definition for message of type '<Reverse-request>"
  (cl:format cl:nil "#request constants~%#int8 FOO=1~%#int8 BAR=2~%#request fields~%string content~%#int32 y~%#another_pkg/AnotherMessage msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Reverse-request)))
  "Returns full string definition for message of type 'Reverse-request"
  (cl:format cl:nil "#request constants~%#int8 FOO=1~%#int8 BAR=2~%#request fields~%string content~%#int32 y~%#another_pkg/AnotherMessage msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Reverse-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'content))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Reverse-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Reverse-request
    (cl:cons ':content (content msg))
))
;//! \htmlinclude Reverse-response.msg.html

(cl:defclass <Reverse-response> (roslisp-msg-protocol:ros-message)
  ((comment
    :reader comment
    :initarg :comment
    :type cl:string
    :initform ""))
)

(cl:defclass Reverse-response (<Reverse-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Reverse-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Reverse-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name exercise1-srv:<Reverse-response> is deprecated: use exercise1-srv:Reverse-response instead.")))

(cl:ensure-generic-function 'comment-val :lambda-list '(m))
(cl:defmethod comment-val ((m <Reverse-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader exercise1-srv:comment-val is deprecated.  Use exercise1-srv:comment instead.")
  (comment m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Reverse-response>) ostream)
  "Serializes a message object of type '<Reverse-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'comment))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'comment))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Reverse-response>) istream)
  "Deserializes a message object of type '<Reverse-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'comment) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'comment) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Reverse-response>)))
  "Returns string type for a service object of type '<Reverse-response>"
  "exercise1/ReverseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Reverse-response)))
  "Returns string type for a service object of type 'Reverse-response"
  "exercise1/ReverseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Reverse-response>)))
  "Returns md5sum for a message object of type '<Reverse-response>"
  "a6ae0dfa99b6e1c22e0f4dd5d3d7311b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Reverse-response)))
  "Returns md5sum for a message object of type 'Reverse-response"
  "a6ae0dfa99b6e1c22e0f4dd5d3d7311b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Reverse-response>)))
  "Returns full string definition for message of type '<Reverse-response>"
  (cl:format cl:nil "#response constants~%#uint32 SECRET=123456~%#response fields~%#another_pkg/YetAnotherMessage val~%#CustomMessageDefinedInThisPackage value~%#int32 result~%string comment~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Reverse-response)))
  "Returns full string definition for message of type 'Reverse-response"
  (cl:format cl:nil "#response constants~%#uint32 SECRET=123456~%#response fields~%#another_pkg/YetAnotherMessage val~%#CustomMessageDefinedInThisPackage value~%#int32 result~%string comment~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Reverse-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'comment))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Reverse-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Reverse-response
    (cl:cons ':comment (comment msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Reverse)))
  'Reverse-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Reverse)))
  'Reverse-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Reverse)))
  "Returns string type for a service object of type '<Reverse>"
  "exercise1/Reverse")