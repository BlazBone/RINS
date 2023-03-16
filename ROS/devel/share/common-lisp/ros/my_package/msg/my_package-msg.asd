
(cl:in-package :asdf)

(defsystem "my_package-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "MyMessage" :depends-on ("_package_MyMessage"))
    (:file "_package_MyMessage" :depends-on ("_package"))
  ))