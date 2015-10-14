#!/usr/bin/env python
import os, subprocess
import sys
import rospy
from std_srvs.srv import Empty
from PyQt4 import QtGui, QtCore

def callEmptyService(svc_name):
    try:
	callable = rospy.ServiceProxy(svc_name, Empty)
	print type(callable())
	return False
    except rospy.ServiceException, e:
	print "Service call failed: %s"%e
	return None

class Example(QtGui.QWidget):
# class Example(QtGui.QMainWindow):

    def __init__(self):
	super(Example, self).__init__()

	# self.status_bar = status_bar
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

	self.rawdata_button = QtGui.QPushButton('Raw Data')
	grid.addWidget(self.rawdata_button, 2, 0)

	self.clear_fault_button = QtGui.QPushButton('Clear Faults On')
	grid.addWidget(self.clear_fault_button, 3, 0)

	self.status_bar = QtGui.QStatusBar()
	grid.addWidget(self.status_bar, 5, 0)

	self.setLayout(grid) 

	QtCore.QObject.connect(self.reread_button, QtCore.SIGNAL('clicked()'), self.onClickedRereadButton)
	QtCore.QObject.connect(self.onoff_button , QtCore.SIGNAL('clicked()'), self.onClickedOnoffButton)
	QtCore.QObject.connect(self.rawdata_button , QtCore.SIGNAL('clicked()'), self.onClickedRawdataButton)
	QtCore.QObject.connect(self.clear_fault_button , QtCore.SIGNAL('clicked()'), self.onClickedClearFaultButton)

	self.setGeometry(300, 300, 350, 300)
	self.setWindowTitle('Radar Control')    
	self.show()

    def onClickedRereadButton(self):
	# print 'hi trying to reread?'
	# os.system('rosrun foobar radar_read_param.py')
	return_code = subprocess.call('rosrun foobar radar_read_param.py', shell=True)
	if return_code:
	    self.status_bar.showMessage('reread vehicle data, but could not reset process, not started ya?')
	else:
	    self.status_bar.showMessage('reread vehicle data, and success reset process sync accordingly')

    def onClickedOnoffButton(self):
	# print self.onoff_button.text()
	if self.onoff_button.text() == 'On Radar':
	    if callEmptyService('radar_packet/radar_on') is None:
		self.status_bar.showMessage('no service. radar driver/process started?')
	    else:
		self.onoff_button.setText('Off Radar')
		self.status_bar.showMessage('sent radar on')
	else:
	    if callEmptyService('radar_packet/radar_off') is None:
		self.status_bar.showMessage('no service. radar driver/process started?')
	    else:
		self.onoff_button.setText('On Radar')
		self.status_bar.showMessage('sent radar off')

    def onClickedRawdataButton(self):
	# print self.onoff_button.text()
	if self.rawdata_button.text() == 'Raw Data':
	    if callEmptyService('radar_packet/rawdata_on') is None:
		self.status_bar.showMessage('no service. radar driver/process started?')
	    else:
		self.rawdata_button.setText('Filtered Data')
		self.status_bar.showMessage('sent rawdata mode')
	else:
	    if callEmptyService('radar_packet/rawdata_off') is None:
		self.status_bar.showMessage('no service. radar driver/process started?')
	    else:
		self.rawdata_button.setText('Raw Data')
		self.status_bar.showMessage('sent filtered data mode')

    def onClickedClearFaultButton(self):
	# print self.onoff_button.text()
	if self.clear_fault_button.text() == 'Clear Faults On':
	    if callEmptyService('radar_packet/clear_fault_on') is None:
		self.status_bar.showMessage('no service. radar driver/process started?')
	    else:
		self.clear_fault_button.setText('Clear Faults Off')
		self.status_bar.showMessage('sent clearing fault on')
	else:
	    if callEmptyService('radar_packet/clear_fault_off') is None:
		self.status_bar.showMessage('no service. radar driver/process started?')
	    else:
		self.clear_fault_button.setText('Clear Faults On')
		self.status_bar.showMessage('sent clearing fault off')

def main():

    app = QtGui.QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
