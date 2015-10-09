; Auto-generated. Do not edit!


(cl:in-package laser_sub-msg)


;//! \htmlinclude lasArray.msg.html

(cl:defclass <lasArray> (roslisp-msg-protocol:ros-message)
  ((laserArray
    :reader laserArray
    :initarg :laserArray
    :type (cl:vector laser_sub-msg:Num)
   :initform (cl:make-array 0 :element-type 'laser_sub-msg:Num :initial-element (cl:make-instance 'laser_sub-msg:Num))))
)

(cl:defclass lasArray (<lasArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <lasArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'lasArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name laser_sub-msg:<lasArray> is deprecated: use laser_sub-msg:lasArray instead.")))

(cl:ensure-generic-function 'laserArray-val :lambda-list '(m))
(cl:defmethod laserArray-val ((m <lasArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader laser_sub-msg:laserArray-val is deprecated.  Use laser_sub-msg:laserArray instead.")
  (laserArray m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <lasArray>) ostream)
  "Serializes a message object of type '<lasArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'laserArray))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'laserArray))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <lasArray>) istream)
  "Deserializes a message object of type '<lasArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'laserArray) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'laserArray)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'laser_sub-msg:Num))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<lasArray>)))
  "Returns string type for a message object of type '<lasArray>"
  "laser_sub/lasArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'lasArray)))
  "Returns string type for a message object of type 'lasArray"
  "laser_sub/lasArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<lasArray>)))
  "Returns md5sum for a message object of type '<lasArray>"
  "5bd8316bb420e1b5b48d1bf192e5bf85")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'lasArray)))
  "Returns md5sum for a message object of type 'lasArray"
  "5bd8316bb420e1b5b48d1bf192e5bf85")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<lasArray>)))
  "Returns full string definition for message of type '<lasArray>"
  (cl:format cl:nil "Num[] laserArray~%~%~%================================================================================~%MSG: laser_sub/Num~%float32 centerX~%float32 centerY~%float32 radius~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'lasArray)))
  "Returns full string definition for message of type 'lasArray"
  (cl:format cl:nil "Num[] laserArray~%~%~%================================================================================~%MSG: laser_sub/Num~%float32 centerX~%float32 centerY~%float32 radius~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <lasArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'laserArray) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <lasArray>))
  "Converts a ROS message object to a list"
  (cl:list 'lasArray
    (cl:cons ':laserArray (laserArray msg))
))
