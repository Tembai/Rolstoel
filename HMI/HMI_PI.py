# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'HMI_PI.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!
import roslib
import rospy
from std_msgs.msg import Int32, Float32, String
from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Form(object):
    def Bttn_mode(self):
        mode = rospy.Publisher('Mode', String, queue_size=10)
        if self.Button_Mode.isChecked():
            self.Button_Mode.setStyleSheet("Background-color : Green")
            self.Button_Mode.setText("Automatische Besturing")
            mode.publish("Automatisch")

        else:
            self.Button_Mode.setStyleSheet("Background-color : Red")
            self.Button_Mode.setText("Handmatige Besturing")
            mode.publish("Handmatig")
    
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(403, 482)
        self.Button_Mode = QtWidgets.QPushButton(Form)
        self.Button_Mode.setGeometry(QtCore.QRect(0, 240, 401, 231))
        self.Button_Mode.setStyleSheet("Background-color : Red")
        self.Button_Mode.setCheckable(True)
        self.Button_Mode.setObjectName("Button_Mode")
        self.Accu_Bar = QtWidgets.QProgressBar(Form)
        self.Accu_Bar.setGeometry(QtCore.QRect(20, 10, 361, 61))
        self.Accu_Bar.setProperty("value", 100)
        self.Accu_Bar.setObjectName("Accu_Bar")
        self.lcdNumber = QtWidgets.QLCDNumber(Form)
        self.lcdNumber.setGeometry(QtCore.QRect(20, 80, 361, 151))
        self.lcdNumber.setObjectName("lcdNumber")

        self.retranslateUi(Form)
        self.Button_Mode.clicked.connect(self.Bttn_mode)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.Button_Mode.setText(_translate("Form", "Handmatige Besturing"))

def Roscom():
    print "Init Ros Communicatie"
    rospy.init_node('Rolstoel_HMI_node', anonymous=True)


if __name__ == "__main__":
    import sys
    Roscom()

    try:
            app
    except:
        app = QtWidgets.QApplication(sys.argv)
        Form = QtWidgets.QWidget()
        ui = Ui_Form()
        ui.setupUi(Form)
        Form.setWindowTitle("Rolstoel")
        Form.show()
        sys.exit(app.exec_())