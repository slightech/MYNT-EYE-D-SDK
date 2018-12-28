
(cl:in-package :asdf)

(defsystem "mynteye_wrapper_d_beta-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Temp" :depends-on ("_package_Temp"))
    (:file "_package_Temp" :depends-on ("_package"))
  ))