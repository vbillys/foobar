#!/usr/bin/env python
import os, subprocess
import sys,signal
import rospy
from foobar.msg import GenericObjCanData
import ServerSolution
import re
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


class RadarStatusGui(QtGui.QWidget):
    def __init__(self, name_id, pos_offset):
	super(RadarStatusGui, self).__init__()

	# self.status_bar = status_bar
	# self.initUI()

	grid = QtGui.QGridLayout()
	grid.setSpacing(10)

	self.label_timestamp = QtGui.QLabel('Stamp:')
	self.label_timestamp_value = QtGui.QLabel('0')
	grid.addWidget(self.label_timestamp, 0 , 0)
	grid.addWidget(self.label_timestamp_value, 0 , 1)

	self.label_scanindex = QtGui.QLabel('ScanIndex:')
	self.label_scanindex_value = QtGui.QLabel('0')
	grid.addWidget(self.label_scanindex, 1 , 0)
	grid.addWidget(self.label_scanindex_value, 1 , 1)

	self.setLayout(grid) 

	self.setGeometry(800+pos_offset*40, 300, 350, 300)
	self.setWindowTitle('Radar Status ' + name_id)    
	self.show()
	
    def onIncomingData(self, msg):
	# print 'received data ', msg.data[782]
	scanindex = msg.data[782]
	self.label_scanindex_value.setText(str(scanindex))
	timestamp = msg.data[778] * 2
	self.label_timestamp_value.setText(str(timestamp))


class RadarControlGui(QtGui.QWidget):
# class Example(QtGui.QMainWindow):

    # def __init__(self, parent):
    def __init__(self, app):
	super(RadarControlGui, self).__init__()
	self.app = app

	# self.status_bar = status_bar
	self.initUI()
	
    def closeEvent(self, event):
	self.app.quit()
	event.accept()

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

def getNameResolution(name_resolution, name_id, interface):
    name_res_dict = {
	    'name_id': name_id,
	    'interface': interface
	    }
    return name_res_dict.get(name_resolution, None)

def static_vars(**kwargs):
    def decorate(func):
	for k in kwargs:
	    setattr(func, k, kwargs[k])
	return func
    return decorate

@static_vars(pos_offset=0)
def createRadarGui(radar_gui_list, name_id, interface, radar_type, name_resolution):
    solved_name_res = getNameResolution(name_resolution, name_id, interface)
    if solved_name_res is None: 
	print 'WARNING: internal error, name resolution faulty, couldni\'t create radar handler'
	return
    if radar_type == 'esr':
	radar_gui_list[name_id] = {}
	new_radar_gui = RadarStatusGui(name_id, createRadarGui.pos_offset)
	sub_data = rospy.Subscriber ('radar_packet/'+solved_name_res+'/processed', GenericObjCanData, new_radar_gui.onIncomingData)
	radar_gui_list[name_id]['gui'] = new_radar_gui
	createRadarGui.pos_offset = createRadarGui.pos_offset + 1


def main():

    if not ServerSolution.resolveRosmaster(): return
    devices_conf = ServerSolution.resolveParameters('radar_packet/devices',os.path.dirname(__file__)+'/radar_read_param.py') 
    if devices_conf is None: return 
    options_conf = ServerSolution.resolveParameters('radar_packet/options',os.path.dirname(__file__)+'/radar_read_param.py') 
    if options_conf is None: return 
    if ServerSolution.checkNodeStarted('radar_gui'): return
    rospy.init_node('radar_gui', anonymous=False)

    app = QtGui.QApplication(sys.argv)

    radar_gui_list = {}
    for device in devices_conf:
	_radar_type = 'esr' if re.search('esr',device['name_id']) else None
	if _radar_type:
	    print device['interface'], device['name_id'], ' interface: ', _radar_type
	    createRadarGui(radar_gui_list, device['name_id'], device['interface'], _radar_type, options_conf['name_resolution'])
	else:
	    print device['interface'], device['name_id'], ' interface: WARNING not detected!'

    ex = RadarControlGui(app)
    # ex.setDisabled(True)
    # ex2 = RadarStatusGui()
    sys.exit(app.exec_())
    
    return app

def sigint_handler(*args):
    """Handler for the SIGINT signal."""
    g_app.quit()

if __name__ == '__main__':
    g_app = main()
    signal.signal(signal.SIGINT, sigint_handler)
    # sys.exit(g_app.exec_())
