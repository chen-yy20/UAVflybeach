
(cl:in-package :asdf)

(defsystem "Uwb_Location-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "uwb" :depends-on ("_package_uwb"))
    (:file "_package_uwb" :depends-on ("_package"))
  ))