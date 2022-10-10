;; Auto-generated. Do not edit!


(when (boundp 'bliss::Point3f)
  (if (not (find-package "BLISS"))
    (make-package "BLISS"))
  (shadow 'Point3f (find-package "BLISS")))
(unless (find-package "BLISS::POINT3F")
  (make-package "BLISS::POINT3F"))

(in-package "ROS")
;;//! \htmlinclude Point3f.msg.html


(defclass bliss::Point3f
  :super ros::object
  :slots (_x _y _z ))

(defmethod bliss::Point3f
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   self)
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
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z
     (setq _z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get bliss::Point3f :md5sum-) "cc153912f1453b708d221682bc23d9ac")
(setf (get bliss::Point3f :datatype-) "bliss/Point3f")
(setf (get bliss::Point3f :definition-)
      "#class cv::Point3f
#{
#    float x;
#    float y;
#    float z;
#}

float32 x
float32 y
float32 z
")



(provide :bliss/Point3f "cc153912f1453b708d221682bc23d9ac")


