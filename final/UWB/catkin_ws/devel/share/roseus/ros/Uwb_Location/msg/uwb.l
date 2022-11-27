;; Auto-generated. Do not edit!


(when (boundp 'Uwb_Location::uwb)
  (if (not (find-package "UWB_LOCATION"))
    (make-package "UWB_LOCATION"))
  (shadow 'uwb (find-package "UWB_LOCATION")))
(unless (find-package "UWB_LOCATION::UWB")
  (make-package "UWB_LOCATION::UWB"))

(in-package "ROS")
;;//! \htmlinclude uwb.msg.html


(defclass Uwb_Location::uwb
  :super ros::object
  :slots (_time _x _y _z ))

(defmethod Uwb_Location::uwb
  (:init
   (&key
    ((:time __time) (instance ros::time :init))
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    )
   (send-super :init)
   (setq _time __time)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   self)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:serialization-length
   ()
   (+
    ;; time _time
    8
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _z
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _time
       (write-long (send _time :sec) s) (write-long (send _time :nsec) s)
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z
       (sys::poke _z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _time
     (send _time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z
     (setq _z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get Uwb_Location::uwb :md5sum-) "41573204035ba6cb84327d8f3ec13839")
(setf (get Uwb_Location::uwb :datatype-) "Uwb_Location/uwb")
(setf (get Uwb_Location::uwb :definition-)
      "time time
float32 x
float32 y
float32 z

")



(provide :Uwb_Location/uwb "41573204035ba6cb84327d8f3ec13839")


