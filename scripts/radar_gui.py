#!/usr/bin/env python
import os
import sys
import rospy
from std_srvs.srv import Empty
from PyQt4 import QtGui, QtCore

def callEmptyService(svc_name):
    try:
	callable = rospy.ServiceProxy(svc_name, Empty)
	print type(callable())
    except rospy.ServiceException, e:
	print "Service call failed: %s"%e

class Example(QtGui.QWidget):

    def __init__(self):
	super(Example, self).__init__()

	self.initUI()

    def initUI(self):

	# title = QtGui.QLabel('Title')
	# author = QtGui.QLabel('Author')
	# review = QtGui.QLabel('Review')

	# titleEdit = QtGui.QLineEdit()
	# authorEdit = QtGui.QLineEdit()
	# reviewEdit = QtGui.QTextEdit()

	grid = QtGui.QGridLayout()
	grid.setSpacing(10)

	# grid.addWidget(title, 1, 0)
	# grid.addWidget(titleEdit, 1, 1)

	# grid.addWidget(author, 2, 0)
	# grid.addWidget(authorEdit, 2, 1)

	# grid.addWidget(review, 3, 0)
	# grid.addWidget(reviewEdit, 3, 1, 5, 1)

	self.onoff_button = QtGui.QPushButton('On Radar')
	grid.addWidget(self.onoff_button, 0, 0)

	self.reread_button = QtGui.QPushButton('Reread Vehicle Data')
	grid.addWidget(self.reread_button, 1, 0)

	self.setLayout(grid) 

	QtCore.QObject.connect(self.reread_button, QtCore.SIGNAL('clicked()'), self.onClickedRereadButton)
	QtCore.QObject.connect(self.onoff_button , QtCore.SIGNAL('clicked()'), self.onClickedOnoffButton)

	self.setGeometry(300, 300, 350, 300)
	self.setWindowTitle('Radar Control')    
	self.show()

    def onClickedRereadButton(self):
	# print 'hi trying to reread?'
	os.system('rosrun foobar radar_read_param.py')

    def onClickedOnoffButton(self):
	# print self.onoff_button.text()
	if self.onoff_button.text() == 'On Radar':
	    self.onoff_button.setText('Off Radar')
	    callEmptyService('radar_packet/radar_on')
	else:
	    self.onoff_button.setText('On Radar')
	    callEmptyService('radar_packet/radar_off')

def main():

    app = QtGui.QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
