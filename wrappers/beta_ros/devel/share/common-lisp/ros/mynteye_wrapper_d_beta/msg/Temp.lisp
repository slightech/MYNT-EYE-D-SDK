; Auto-generated. Do not edit!


(cl:in-package mynteye_wrapper_d_beta-msg)


;//! \htmlinclude Temp.msg.html

(cl:defclass <Temp> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (data
    :reader data
    :initarg :data
    :type cl:float
    :initform 0.0))
)

(cl:defclass Temp (<Temp>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Temp>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Temp)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mynteye_wrapper_d_beta-msg:<Temp> is deprecated: use mynteye_wrapper_d_beta-msg:Temp instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Temp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mynteye_wrapper_d_beta-msg:header-val is deprecated.  Use mynteye_wrapper_d_beta-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <Temp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mynteye_wrapper_d_beta-msg:data-val is deprecated.  Use mynteye_wrapper_d_beta-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Temp>) ostream)
  "Serializes a message object of type '<Temp>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Temp>) istream)
  "Deserializes a message object of type '<Temp>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'data) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Temp>)))
  "Returns string type for a message object of type '<Temp>"
  "mynteye_wrapper_d_beta/Temp")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Temp)))
  "Returns string type for a message object of type 'Temp"
  "mynteye_wrapper_d_beta/Temp")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Temp>)))
  "Returns md5sum for a message object of type '<Temp>"
  "ef848af8cf12f6df11682cc76fba477b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Temp)))
  "Returns md5sum for a message object of type 'Temp"
  "ef848af8cf12f6df11682cc76fba477b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Temp>)))
  "Returns full string definition for message of type '<Temp>"
  (cl:format cl:nil "std_msgs/Header header~%float32 data~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Temp)))
  "Returns full string definition for message of type 'Temp"
  (cl:format cl:nil "std_msgs/Header header~%float32 data~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Temp>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Temp>))
  "Converts a ROS message object to a list"
  (cl:list 'Temp
    (cl:cons ':header (header msg))
    (cl:cons ':data (data msg))
))
