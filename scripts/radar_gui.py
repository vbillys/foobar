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
	
	self.timer_till_NA = QtCore.QTimer()
	self.recv_counter = 0
	QtCore.QObject.connect(self.timer_till_NA,QtCore.SIGNAL("timeout()"), self.checkNAandResetCounter)
	self.timer_till_NA.start(2500)

	grid = QtGui.QGridLayout()
	grid.setSpacing(3)

	# splitter = QtGui.QSplitter(QtCore.Qt.Vertical)
	horizLine	=  QtGui.QFrame()
	horizLine.setFrameStyle(QtGui.QFrame.HLine)
	# horizLine.setSizePolicy(QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
	horizLine.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)

	horizLine2	=  QtGui.QFrame()
	horizLine2.setFrameStyle(QtGui.QFrame.HLine)
	horizLine2.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)

	horizLine3	=  QtGui.QFrame()
	horizLine3.setFrameStyle(QtGui.QFrame.HLine)
	horizLine3.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)

	vertiLine2	=  QtGui.QFrame()
	vertiLine2.setFrameStyle(QtGui.QFrame.VLine)
	vertiLine2.setSizePolicy(QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)

	vertiLine3	=  QtGui.QFrame()
	vertiLine3.setFrameStyle(QtGui.QFrame.VLine)
	vertiLine3.setSizePolicy(QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)

	self.label_NA = QtGui.QLabel('CHECKING...')
	grid.addWidget(self.label_NA, 0 , 0)

	grid.addWidget(horizLine, 1, 0, 1 ,8)

	self.label_timestamp = QtGui.QLabel('Stamp:')
	self.label_timestamp_value = QtGui.QLabel('NA')
	grid.addWidget(self.label_timestamp, 2 , 0)
	grid.addWidget(self.label_timestamp_value, 2 , 1)

	self.label_scanindex = QtGui.QLabel('ScanIndex:')
	self.label_scanindex_value = QtGui.QLabel('NA')
	grid.addWidget(self.label_scanindex, 3 , 0)
	grid.addWidget(self.label_scanindex_value, 3 , 1)

	self.label_temp = QtGui.QLabel('Temperature:')
	self.label_temp_value = QtGui.QLabel('NA')
	grid.addWidget(self.label_temp, 4 , 0)
	grid.addWidget(self.label_temp_value, 4 , 1)

	self.label_raw_mode = QtGui.QLabel('Raw Mode:')
	self.label_raw_mode_value = QtGui.QLabel('NA')
	grid.addWidget(self.label_raw_mode, 5 , 0)
	grid.addWidget(self.label_raw_mode_value, 5 , 1)

	self.label_max_track_ack = QtGui.QLabel('No of Track ACK:')
	self.label_max_track_ack_value = QtGui.QLabel('NA')
	grid.addWidget(self.label_max_track_ack, 6 , 0)
	grid.addWidget(self.label_max_track_ack_value, 6 , 1)

	self.label_grouping_mode = QtGui.QLabel('Grouping Mode:')
	self.label_grouping_mode_value = QtGui.QLabel('NA')
	grid.addWidget(self.label_grouping_mode , 7 , 0)
	grid.addWidget(self.label_grouping_mode_value , 7 , 1)

	self.label_xcvr_operational= QtGui.QLabel('XVCR Operational:')
	self.label_xcvr_operational_value= QtGui.QLabel('NA')
	grid.addWidget(self.label_xcvr_operational, 8 , 0)
	grid.addWidget(self.label_xcvr_operational_value , 8 , 1)

	self.label_autoalign_angle= QtGui.QLabel('AngleAutoAlign:')
	self.label_autoalign_angle_value= QtGui.QLabel('NA')
	grid.addWidget(self.label_autoalign_angle, 9 , 0)
	grid.addWidget(self.label_autoalign_angle_value, 9 , 1)

	self.label_recc_unconv= QtGui.QLabel('UnConverge Recommend:')
	self.label_recc_unconv_value= QtGui.QLabel('NA')
	grid.addWidget(self.label_recc_unconv, 10 , 0)
	grid.addWidget(self.label_recc_unconv_value, 10 , 1)

	self.label_syspower_mode= QtGui.QLabel('Sys Power Mode:')
	self.label_syspower_mode_value= QtGui.QLabel('NA')
	grid.addWidget(self.label_syspower_mode, 11 , 0)
	grid.addWidget(self.label_syspower_mode_value, 11 , 1)

	grid.addWidget(vertiLine2, 2, 3, 10 ,1)

	self.label_comm_err = QtGui.QLabel('Comm Error:')
	self.label_comm_err_value  = QtGui.QLabel('NA')
	grid.addWidget(self.label_comm_err , 2 , 4)
	grid.addWidget(self.label_comm_err_value  , 2 , 5)

	grid.addWidget(vertiLine3, 2, 6, 10 ,1)

	self.label_sw_ver_dsp = QtGui.QLabel('Ver. SW Dsp:')
	self.label_sw_ver_dsp_value = QtGui.QLabel('NA')
	grid.addWidget(self.label_sw_ver_dsp, 2 , 7)
	grid.addWidget(self.label_sw_ver_dsp_value , 2 , 8)


	self.setLayout(grid) 

	self.setGeometry(800+pos_offset*40, 200, 750, 400)
	self.setWindowTitle('Radar Status ' + name_id)    
	self.show()

    def resetDisplayNA(self, nodata):
	# self.label_timestamp_value.setText('NA')
	# self.label_scanindex_value.setText('NA')
	# self.label_sw_ver_dsp_value.setText('NA')
	# self.label_comm_err_value.setText('NA')
	# self.label_temp_value.setText('NA')
	if nodata:
	    self.label_NA.setText('NO DATA!!')
	else:
	    self.label_NA.setText('INCOMING DATA OK')
	
    def checkNAandResetCounter(self):
	if self.recv_counter == 0:
	    self.resetDisplayNA(True)
	else:
	    self.resetDisplayNA(False)
	self.recv_counter = 0

    def onIncomingData(self, msg):

	self.recv_counter = self.recv_counter + 1

	# print 'received data ', msg.data[782]
	scanindex = msg.data[782]
	self.label_scanindex_value.setText(str(scanindex))
	timestamp = msg.data[778] * 2
	self.label_timestamp_value.setText(str(timestamp))
	sw_ver_dsp_value = msg.data[787]
	self.label_sw_ver_dsp_value.setText(str(sw_ver_dsp_value))

	comm_err_value = msg.data[779]
	self.label_comm_err_value.setText(str(comm_err_value))

	temperature_value = msg.data[788]
	self.label_temp_value.setText(str(temperature_value))
	raw_mode_value = msg.data[789]
	self.label_raw_mode_value.setText(str(raw_mode_value))
	max_track_ack_value = msg.data[792]
	self.label_max_track_ack_value.setText(str(max_track_ack_value))
	grouping_mode_value = msg.data[794]
	self.label_grouping_mode_value.setText(str(grouping_mode_value))
	xcvr_operational_value = msg.data[795]
	self.label_xcvr_operational_value.setText(str(xcvr_operational_value))
	autoalign_angle_value = msg.data[809]
	self.label_autoalign_angle_value.setText(str(autoalign_angle_value))
	recc_unconv_value = msg.data[832]
	self.label_recc_unconv_value.setText(str(recc_unconv_value))
	syspower_mode_value = msg.data[834]
	self.label_syspower_mode_value.setText(str(syspower_mode_value))


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
