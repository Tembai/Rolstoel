# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'HMI_PI.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(403, 482)
        self.Button_Mode = QtWidgets.QPushButton(Form)
        self.Button_Mode.setGeometry(QtCore.QRect(0, 240, 401, 231))
        self.Button_Mode.setStyleSheet("Red")
        self.Button_Mode.setCheckable(True)
        self.Button_Mode.setObjectName("Button_Mode")
        self.Accu_Bar = QtWidgets.QProgressBar(Form)
        self.Accu_Bar.setGeometry(QtCore.QRect(20, 10, 361, 61))
        self.Accu_Bar.setProperty("value", 24)
        self.Accu_Bar.setObjectName("Accu_Bar")
        self.lcdNumber = QtWidgets.QLCDNumber(Form)
        self.lcdNumber.setGeometry(QtCore.QRect(20, 80, 361, 151))
        self.lcdNumber.setObjectName("lcdNumber")

        self.retranslateUi(Form)
        self.Button_Mode.clicked.connect(Form.Bttn_mode)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.Button_Mode.setText(_translate("Form", "Handmatig"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Form = QtWidgets.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec_())

