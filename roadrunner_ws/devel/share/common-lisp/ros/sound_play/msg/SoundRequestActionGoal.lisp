; Auto-generated. Do not edit!


(cl:in-package sound_play-msg)


;//! \htmlinclude SoundRequestActionGoal.msg.html

(cl:defclass <SoundRequestActionGoal> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (goal_id
    :reader goal_id
    :initarg :goal_id
    :type actionlib_msgs-msg:GoalID
    :initform (cl:make-instance 'actionlib_msgs-msg:GoalID))
   (goal
    :reader goal
    :initarg :goal
    :type sound_play-msg:SoundRequestGoal
    :initform (cl:make-instance 'sound_play-msg:SoundRequestGoal)))
)

(cl:defclass SoundRequestActionGoal (<SoundRequestActionGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SoundRequestActionGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SoundRequestActionGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sound_play-msg:<SoundRequestActionGoal> is deprecated: use sound_play-msg:SoundRequestActionGoal instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SoundRequestActionGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sound_play-msg:header-val is deprecated.  Use sound_play-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'goal_id-val :lambda-list '(m))
(cl:defmethod goal_id-val ((m <SoundRequestActionGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sound_play-msg:goal_id-val is deprecated.  Use sound_play-msg:goal_id instead.")
  (goal_id m))

(cl:ensure-generic-function 'goal-val :lambda-list '(m))
(cl:defmethod goal-val ((m <SoundRequestActionGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sound_play-msg:goal-val is deprecated.  Use sound_play-msg:goal instead.")
  (goal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SoundRequestActionGoal>) ostream)
  "Serializes a message object of type '<SoundRequestActionGoal>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal_id) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SoundRequestActionGoal>) istream)
  "Deserializes a message object of type '<SoundRequestActionGoal>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal_id) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SoundRequestActionGoal>)))
  "Returns string type for a message object of type '<SoundRequestActionGoal>"
  "sound_play/SoundRequestActionGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SoundRequestActionGoal)))
  "Returns string type for a message object of type 'SoundRequestActionGoal"
  "sound_play/SoundRequestActionGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SoundRequestActionGoal>)))
  "Returns md5sum for a message object of type '<SoundRequestActionGoal>"
  "263b69ef77b0c5355d0375d85def66da")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SoundRequestActionGoal)))
  "Returns md5sum for a message object of type 'SoundRequestActionGoal"
  "263b69ef77b0c5355d0375d85def66da")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SoundRequestActionGoal>)))
  "Returns full string definition for message of type '<SoundRequestActionGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%SoundRequestGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: sound_play/SoundRequestGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%SoundRequest sound_request~%~%================================================================================~%MSG: sound_play/SoundRequest~%# IMPORTANT: You should never have to generate this message yourself.~%# Use the sound_play::SoundClient C++ helper or the~%# sound_play.libsoundplay.SoundClient Python helper.~%~%# Sounds~%int8 BACKINGUP = 1~%int8 NEEDS_UNPLUGGING = 2~%int8 NEEDS_PLUGGING = 3~%int8 NEEDS_UNPLUGGING_BADLY = 4~%int8 NEEDS_PLUGGING_BADLY = 5~%~%# Sound identifiers that have special meaning~%int8 ALL = -1 # Only legal with PLAY_STOP~%int8 PLAY_FILE = -2~%int8 SAY = -3~%~%int8 sound # Selects which sound to play (see above)~%~%# Commands~%int8 PLAY_STOP = 0 # Stop this sound from playing~%int8 PLAY_ONCE = 1 # Play the sound once~%int8 PLAY_START = 2 # Play the sound in a loop until a stop request occurs~%~%int8 command # Indicates what to do with the sound~%~%string arg # file name or text to say~%string arg2 # other arguments~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SoundRequestActionGoal)))
  "Returns full string definition for message of type 'SoundRequestActionGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%SoundRequestGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: sound_play/SoundRequestGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%SoundRequest sound_request~%~%================================================================================~%MSG: sound_play/SoundRequest~%# IMPORTANT: You should never have to generate this message yourself.~%# Use the sound_play::SoundClient C++ helper or the~%# sound_play.libsoundplay.SoundClient Python helper.~%~%# Sounds~%int8 BACKINGUP = 1~%int8 NEEDS_UNPLUGGING = 2~%int8 NEEDS_PLUGGING = 3~%int8 NEEDS_UNPLUGGING_BADLY = 4~%int8 NEEDS_PLUGGING_BADLY = 5~%~%# Sound identifiers that have special meaning~%int8 ALL = -1 # Only legal with PLAY_STOP~%int8 PLAY_FILE = -2~%int8 SAY = -3~%~%int8 sound # Selects which sound to play (see above)~%~%# Commands~%int8 PLAY_STOP = 0 # Stop this sound from playing~%int8 PLAY_ONCE = 1 # Play the sound once~%int8 PLAY_START = 2 # Play the sound in a loop until a stop request occurs~%~%int8 command # Indicates what to do with the sound~%~%string arg # file name or text to say~%string arg2 # other arguments~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SoundRequestActionGoal>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal_id))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SoundRequestActionGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'SoundRequestActionGoal
    (cl:cons ':header (header msg))
    (cl:cons ':goal_id (goal_id msg))
    (cl:cons ':goal (goal msg))
))