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
    def __init__(self, name_id, pos_offset, radar_data):
	super(RadarStatusGui, self).__init__()

	self.radar_data = radar_data

	# self.status_bar = status_bar
	# self.initUI()

	self.defineConstant()
	
	self.timer_till_NA = QtCore.QTimer()
	self.recv_counter = 0
	QtCore.QObject.connect(self.timer_till_NA,QtCore.SIGNAL("timeout()"), self.checkNAandResetCounter)
	self.timer_till_NA.start(1000)

	self.timer_poll = QtCore.QTimer()
	QtCore.QObject.connect(self.timer_poll,QtCore.SIGNAL("timeout()"), self.pollData)
	self.timer_poll.start(50)

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

	self.label_mrlr_mode= QtGui.QLabel('MR_LR Mode:')
	self.label_mrlr_mode_value= QtGui.QLabel('NA')
	grid.addWidget(self.label_mrlr_mode, 12 , 0)
	grid.addWidget(self.label_mrlr_mode_value, 12 , 1)

	self.label_sidelobe_blockage= QtGui.QLabel('Sidelobe Blockage:')
	self.label_sidelobe_blockage_value= QtGui.QLabel('NA')
	grid.addWidget(self.label_sidelobe_blockage, 13 , 0)
	grid.addWidget(self.label_sidelobe_blockage_value, 13 , 1)

	self.label_partial_blockage= QtGui.QLabel('Partial Blockage:')
	self.label_partial_blockage_value= QtGui.QLabel('NA')
	grid.addWidget(self.label_partial_blockage, 14 , 0)
	grid.addWidget(self.label_partial_blockage_value, 14 , 1)

	self.label_found_target= QtGui.QLabel('Found Target:')
	self.label_found_target_value= QtGui.QLabel('NA')
	grid.addWidget(self.label_found_target, 15 , 0)
	grid.addWidget(self.label_found_target_value, 15 , 1)

	# self.label_= QtGui.QLabel('MR_LR Mode:')
	# self.label__value= QtGui.QLabel('NA')
	# grid.addWidget(self.label_, 12 , 0)
	# grid.addWidget(self.label__value, 12 , 1)

	self.label_vert_alignupdt= QtGui.QLabel('Vert Align Updated:')
	self.label_vert_alignupdt_value= QtGui.QLabel('NA')
	grid.addWidget(self.label_vert_alignupdt, 16 , 0)
	grid.addWidget(self.label_vert_alignupdt_value, 16 , 1)

	self.label_vert_misalign= QtGui.QLabel('Vert Misalign:')
	self.label_vert_misalign_value= QtGui.QLabel('NA')
	grid.addWidget(self.label_vert_misalign, 17 , 0)
	grid.addWidget(self.label_vert_misalign_value, 17 , 1)

	self.label_serv_updatesdone= QtGui.QLabel('Serv UpdatesDone:')
	self.label_serv_updatesdone_value= QtGui.QLabel('NA')
	grid.addWidget(self.label_serv_updatesdone, 18 , 0)
	grid.addWidget(self.label_serv_updatesdone_value, 18 , 1)

	grid.addWidget(vertiLine2, 2, 3, 10 ,1)

	self.label_comm_err = QtGui.QLabel('Comm Error:')
	self.label_comm_err_value  = QtGui.QLabel('NA')
	grid.addWidget(self.label_comm_err , 2 , 4)
	grid.addWidget(self.label_comm_err_value  , 2 , 5)

	self.label_range_perf_err= QtGui.QLabel('Radar Blocked:')
	self.label_range_perf_err_value= QtGui.QLabel('NA')
	grid.addWidget(self.label_range_perf_err, 3 , 4)
	grid.addWidget(self.label_range_perf_err_value, 3 , 5)

	self.label_overheat_err= QtGui.QLabel('Overheat:')
	self.label_overheat_err_value= QtGui.QLabel('NA')
	grid.addWidget(self.label_overheat_err, 4 , 4)
	grid.addWidget(self.label_overheat_err_value, 4 , 5)

	self.label_internal_err= QtGui.QLabel('Internal Err:')
	self.label_internal_err_value= QtGui.QLabel('NA')
	grid.addWidget(self.label_internal_err, 5 , 4)
	grid.addWidget(self.label_internal_err_value, 5 , 5)

	self.label_active_fault_err= QtGui.QLabel('Active:')
	self.label_active_fault_err_value= QtGui.QLabel('NA')
	grid.addWidget(self.label_active_fault_err, 6 , 4)
	grid.addWidget(self.label_active_fault_err_value, 6 , 5)

	self.label_history_fault_err= QtGui.QLabel('Historic:')
	self.label_history_fault_err_value= QtGui.QLabel('NA')
	grid.addWidget(self.label_history_fault_err, 7 , 4)
	grid.addWidget(self.label_history_fault_err_value, 7 , 5)

	base_row_for_center_horiz = 8

	grid.addWidget(horizLine2, base_row_for_center_horiz, 4, 1 ,2)

	self.label_valid = QtGui.QLabel('VALID')
	self.label_lr = QtGui.QLabel('Long')
	self.label_sr = QtGui.QLabel('Short')
	grid.addWidget(self.label_valid, base_row_for_center_horiz+1,4)
	grid.addWidget(self.label_lr, base_row_for_center_horiz+1 ,5)
	grid.addWidget(self.label_sr, base_row_for_center_horiz+1 ,6)

	self.label_valid_rangerate= QtGui.QLabel('RangeRate:')
	grid.addWidget(self.label_valid_rangerate, base_row_for_center_horiz+2, 4)
	self.label_valid_range= QtGui.QLabel('Range:')
	grid.addWidget(self.label_valid_range,  base_row_for_center_horiz+3 , 4)
	self.label_valid_power= QtGui.QLabel('Power:')
	grid.addWidget(self.label_valid_power,  base_row_for_center_horiz+4, 4)
	self.label_valid_angle= QtGui.QLabel('Angle:')
	grid.addWidget(self.label_valid_angle,  base_row_for_center_horiz+5, 4)

	self.label_valid_lr_rangerate_value= QtGui.QLabel('NA')
	grid.addWidget(self.label_valid_lr_rangerate_value,  base_row_for_center_horiz+2, 5)
	self.label_valid_sr_rangerate_value= QtGui.QLabel('NA')
	grid.addWidget(self.label_valid_sr_rangerate_value,  base_row_for_center_horiz+2, 6)

	self.label_valid_lr_range_value= QtGui.QLabel('NA')
	grid.addWidget(self.label_valid_lr_range_value,  base_row_for_center_horiz+3, 5)
	self.label_valid_sr_range_value= QtGui.QLabel('NA')
	grid.addWidget(self.label_valid_sr_range_value,  base_row_for_center_horiz+3, 6)

	self.label_valid_lr_power_value= QtGui.QLabel('NA')
	grid.addWidget(self.label_valid_lr_power_value,  base_row_for_center_horiz+4, 5)
	self.label_valid_sr_power_value= QtGui.QLabel('NA')
	grid.addWidget(self.label_valid_sr_power_value,  base_row_for_center_horiz+4, 6)

	self.label_valid_lr_angle_value= QtGui.QLabel('NA')
	grid.addWidget(self.label_valid_lr_angle_value,  base_row_for_center_horiz+5, 5)
	self.label_valid_sr_angle_value= QtGui.QLabel('NA')
	grid.addWidget(self.label_valid_sr_angle_value,  base_row_for_center_horiz+5, 6)


	grid.addWidget(vertiLine3, 2, 7, 10 ,1)

	self.label_sw_ver_dsp = QtGui.QLabel('Ver. SW Dsp:')
	self.label_sw_ver_dsp_value = QtGui.QLabel('NA')
	grid.addWidget(self.label_sw_ver_dsp, 2 , 8)
	grid.addWidget(self.label_sw_ver_dsp_value , 2 , 9)

	self.label_sw_ver_pld= QtGui.QLabel('Ver. SW Pld:')
	self.label_sw_ver_pld_value= QtGui.QLabel('NA')
	grid.addWidget(self.label_sw_ver_pld, 3 , 8)
	grid.addWidget(self.label_sw_ver_pld_value , 3 , 9)

	self.label_sw_ver_host= QtGui.QLabel('Ver. SW Host:')
	self.label_sw_ver_host_value = QtGui.QLabel('NA')
	grid.addWidget(self.label_sw_ver_host, 4 , 8)
	grid.addWidget(self.label_sw_ver_host_value , 4 , 9)

	self.label_hw_ver= QtGui.QLabel('Ver. HW:')
	self.label_hw_ver_value = QtGui.QLabel('NA')
	grid.addWidget(self.label_hw_ver, 5 , 8)
	grid.addWidget(self.label_hw_ver_value , 5 , 9)

	self.label_interface_ver= QtGui.QLabel('Ver. Interface:')
	self.label_interface_ver_value= QtGui.QLabel('NA')
	grid.addWidget(self.label_interface_ver, 6 , 8)
	grid.addWidget(self.label_interface_ver_value, 6 , 9)

	self.label_serial_ver= QtGui.QLabel('Ver. Serial:')
	self.label_serial_ver_value= QtGui.QLabel('NA')
	grid.addWidget(self.label_serial_ver, 7 , 8)
	grid.addWidget(self.label_serial_ver_value , 7 , 9)

	self.label_serial_full_ver= QtGui.QLabel('Ver. Serial Full:')
	self.label_serial_full_ver_value= QtGui.QLabel('NA')
	grid.addWidget(self.label_serial_full_ver, 8 , 8)
	grid.addWidget(self.label_serial_full_ver_value, 8 , 9)


	self.setLayout(grid) 

	self.setGeometry(800+pos_offset*40, 200, 750, 450)
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
	# if self.recv_counter == 0:
	if self.radar_data.getCounter() == 0:
	    self.resetDisplayNA(True)
	else:
	    self.resetDisplayNA(False)
	self.recv_counter = 0
	self.radar_data.resetCounter()

    def defineConstant(self):
	self.strGroupingModes = ['no','mov. only','st. only','both']
	self.strSysPowerModes = ['init','rad_off','rad_on','dsp_shut','dsp_off','host_shut','(invalid)']
	self.strMrlrModes = ['resv', 'MR-only', 'LR-only', 'MR-and-LR']

    def pollData(self):
	_msg = self.radar_data.getData()
	if _msg is not None:
	    self.onIncomingData(_msg)

    def onIncomingData(self, msg):

	self.recv_counter = self.recv_counter + 1

	# print 'received data ', msg.data[782]
	scanindex = msg.data[782]
	self.label_scanindex_value.setText(str(scanindex))
	timestamp = msg.data[778] * 2
	self.label_timestamp_value.setText(str(timestamp))
	temperature_value = msg.data[788]
	self.label_temp_value.setText(str(temperature_value))
	raw_mode_value = msg.data[789]
	self.label_raw_mode_value.setText(str(raw_mode_value))
	max_track_ack_value = msg.data[792]
	self.label_max_track_ack_value.setText(str(max_track_ack_value))
	grouping_mode_value = msg.data[794]
	self.label_grouping_mode_value.setText(self.strGroupingModes[grouping_mode_value])
	xcvr_operational_value = msg.data[795]
	self.label_xcvr_operational_value.setText(str(xcvr_operational_value))
	autoalign_angle_value = msg.data[809]
	self.label_autoalign_angle_value.setText(str(autoalign_angle_value))
	recc_unconv_value = msg.data[832]
	self.label_recc_unconv_value.setText(str(recc_unconv_value))
	syspower_mode_value = msg.data[834]
	self.label_syspower_mode_value.setText(self.strSysPowerModes[syspower_mode_value])
	sidelobe_blockage_value = msg.data[805]
	self.label_sidelobe_blockage_value.setText(str(sidelobe_blockage_value))
	partial_blockage_value = msg.data[806]
	self.label_partial_blockage_value.setText(str(partial_blockage_value))
	mrlr_mode_value = msg.data[808]
	self.label_mrlr_mode_value.setText(self.strMrlrModes[mrlr_mode_value])
	found_target_value = msg.data[828]
	self.label_found_target_value.setText(str(found_target_value))
	vert_alignupdt_value = msg.data[825]
	self.label_vert_alignupdt_value.setText(str(vert_alignupdt_value))
	vert_misalign_value = msg.data[826] * 0.0625
	self.label_vert_misalign_value.setText(str(vert_misalign_value))
	serv_updatesdone_value = msg.data[827]
	self.label_serv_updatesdone_value.setText(str(serv_updatesdone_value))

	comm_err_value = msg.data[779]
	self.label_comm_err_value.setText(str(comm_err_value))
	range_perf_err_value = msg.data[790]
	self.label_range_perf_err_value.setText(str(range_perf_err_value))
	overheat_err_value = msg.data[791]
	self.label_overheat_err_value.setText(str(overheat_err_value))
	internal_err_value = msg.data[793]
	self.label_internal_err_value.setText(str(internal_err_value))
	active_fault_err_value = msg.data[837:845]
	self.label_active_fault_err_value.setText(' '.join([format(x,'02x') for x in active_fault_err_value]))
	history_fault_err_value = msg.data[845:853]
	self.label_history_fault_err_value.setText(' '.join([format(x,'02x') for x in history_fault_err_value]))

	sw_ver_dsp_value = msg.data[787]
	self.label_sw_ver_dsp_value.setText('x'+format(sw_ver_dsp_value,'04x'))
	sw_ver_pld_value = msg.data[798]
	self.label_sw_ver_pld_value.setText(str(sw_ver_pld_value))
	sw_ver_host_value = msg.data[799]
	self.label_sw_ver_host_value.setText('x'+format(sw_ver_host_value ,'08x'))
	hw_ver_value = msg.data[800]
	self.label_hw_ver_value.setText(str(hw_ver_value))
	interface_ver_value = msg.data[801]
	self.label_interface_ver_value.setText(str(interface_ver_value))
	serial_ver_value = msg.data[857]
	self.label_serial_ver_value.setText(str(serial_ver_value))
	serial_full_ver_value = msg.data[802]
	self.label_serial_full_ver_value.setText('x'+format(sw_ver_dsp_value,'06x'))

	valid_lr_rangerate_value = msg.data[769] * 0.0078125
	self.label_valid_lr_rangerate_value.setText(str(valid_lr_rangerate_value))
	valid_sr_rangerate_value = msg.data[774] * 0.0078125
	self.label_valid_sr_rangerate_value.setText(str(valid_sr_rangerate_value))

	valid_lr_range_value = msg.data[770] * 0.0078125
	self.label_valid_lr_range_value.setText(str(valid_lr_range_value))
	valid_sr_range_value = msg.data[775] * 0.0078125
	self.label_valid_sr_range_value.setText(str(valid_sr_range_value))

	valid_lr_power_value = msg.data[771]
	self.label_valid_lr_power_value.setText(str(valid_lr_power_value))
	valid_sr_power_value = msg.data[776]
	self.label_valid_sr_power_value.setText(str(valid_sr_power_value))

	valid_lr_angle_value = msg.data[772] * 0.0625
	self.label_valid_lr_angle_value.setText(str(valid_lr_angle_value))
	valid_sr_angle_value = msg.data[777] * 0.0625
	self.label_valid_sr_angle_value.setText(str(valid_sr_angle_value))


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

class radarData:
    def __init__(self):
	self.recv_counter = 0
    def onIncomingData(self, msg):

	self.recv_counter = self.recv_counter + 1
	self.data = msg

    def getData(self):
	if self.recv_counter:
	    return self.data
	else:
	    return None

    def resetCounter(self):
	self.recv_counter = 0

    def getCounter(self):
	return self.recv_counter


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
	# new_radar_gui = RadarStatusGui(name_id, createRadarGui.pos_offset)
	radar_data = radarData()
	new_radar_gui = RadarStatusGui(name_id, createRadarGui.pos_offset, radar_data)
	# sub_data = rospy.Subscriber ('radar_packet/'+solved_name_res+'/processed', GenericObjCanData, new_radar_gui.onIncomingData)
	sub_data = rospy.Subscriber ('radar_packet/'+solved_name_res+'/processed', GenericObjCanData, radar_data.onIncomingData)
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
