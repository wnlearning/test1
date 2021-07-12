from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from ground2 import *
import serial.tools.list_ports
import sys
import struct
from binascii import unhexlify
# from yaw import *


class MainWindow(QMainWindow):
	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)
		
		# 设置窗口标题
		self.setWindowTitle('Swarm')
		self.setWindowIcon(QIcon('title.jpg'))
		
		# 添加布局, 子窗口1
		layout1 = QVBoxLayout()
		
		# 创建标签及下拉列表
		lab1 = QLabel()
		lab1.setText('可用串口：')
		layout1.addWidget(lab1)
		self.combobox = QComboBox()
		data = serial.tools.list_ports.comports()
		for i in data:
			self.combobox.addItem(str(i))
		# (*list(serial.tools.list_ports.comports()))
		layout1.addWidget(self.combobox)
		
		# 创建输入标签及文本框
		lab2 = QLabel()
		lab2.setText('无人机编号：')
		layout1.addWidget(lab2)
		self.line_edit = QLineEdit()
		self.line_edit.setMaxLength(1)
		layout1.addWidget(self.line_edit)
		
		# 设置左1窗按钮组子窗口
		layout1_1 = QHBoxLayout()
		
		# 设置定点飞行参数
		lab1_1_1 = QLabel()
		lab1_1_1.setText('经度：')
		layout1_1.addWidget(lab1_1_1)
		self.line_edit1_1_1 = QLineEdit()
		layout1_1.addWidget(self.line_edit1_1_1)
		lab1_1_2 = QLabel()
		lab1_1_2.setText('纬度：')
		layout1_1.addWidget(lab1_1_2)
		self.line_edit1_1_2 = QLineEdit()
		layout1_1.addWidget(self.line_edit1_1_2)
		lab1_1_3 = QLabel()
		lab1_1_3.setText('高度：')
		layout1_1.addWidget(lab1_1_3)
		self.line_edit1_1_3 = QLineEdit()
		layout1_1.addWidget(self.line_edit1_1_3)
		layout1.addLayout(layout1_1)
		layout1_2 = QHBoxLayout()
		lab1_2_1 = QLabel()
		lab1_2_1.setText('目标航向：')
		layout1_2.addWidget(lab1_2_1)
		self.line_edit1_2_1 = QLineEdit()
		layout1_2.addWidget(self.line_edit1_2_1)
		lab1_2_2 = QLabel()
		lab1_2_2.setText('目标速度：')
		layout1_2.addWidget(lab1_2_2)
		self.line_edit1_2_2 = QLineEdit()
		layout1_2.addWidget(self.line_edit1_2_2)
		layout1.addLayout(layout1_2)
		
		# 创建基本命令列表与任务命令列表
		layout1_3 = QGridLayout()
		# 基本命令列表占一列
		layout1_3_1 = QVBoxLayout()
		lab3 = QLabel()
		lab3.setText('基本命令列表:')
		layout1_3_1.addWidget(lab3)
		uav1_lab = ['对时', '自检', '起飞', '定点飞行', '返航']
		for i in range(len(uav1_lab)):
			button = QPushButton(uav1_lab[i])
			# 将按钮按压信号与自定义函数关联
			button.pressed.connect(lambda x=i: self._my_func(x))
			# 将按钮添加到布局中
			layout1_3_1.addWidget(button)
		layout1_3.addLayout(layout1_3_1,0,0,1,1)
		
		# 基本命令列表占一列
		layout1_3_2 = QVBoxLayout()
		lab4 = QLabel()
		lab4.setText('任务命令列表:')
		layout1_3_2.addWidget(lab4)
		uav1_lab = ['接转发', '停止接转发', '预留2', '预留3', '预留4']
		for i in range(len(uav1_lab)):
			button = QPushButton(uav1_lab[i])
			# 将按钮按压信号与自定义函数关联
			self.work2 = WorkThread2()
			button.pressed.connect(lambda x=i: self._my_func2(x))
			self.work2.trigger.connect(self.display2)
			# 将按钮添加到布局中
			layout1_3_2.addWidget(button)
		layout1_3.addLayout(layout1_3_2, 0,1,1,1)
		layout1.addLayout(layout1_3)
		
		
		# 创建输出文本框
		lab4 = QLabel()
		lab4.setText('状态列表:')
		layout1.addWidget(lab4)
		self.text_browser = QTextBrowser()
		# self.text_browser.setText('Hello World')
		layout1.addWidget(self.text_browser)
		
		# 将前窗口作为子窗口1，建立子窗口2
		layout2 = QVBoxLayout()
		
		self.button5 = QPushButton('接收跟随机数据')
		self.work1 = WorkThread1()
		self.button5.pressed.connect(self.excute1)
		self.work1.trigger.connect(self.display1)
		layout2.addWidget(self.button5)
		self.button6 = QPushButton('停止接收跟随机数据，释放串口')
		self.button6.pressed.connect(self.stop1)
		layout2.addWidget(self.button6)
		
		self.text_browser2 = QTextBrowser()
		layout2.addWidget(self.text_browser2)
		
		# 建立主窗口
		layout3 = QHBoxLayout()
		# layout3 = QGridLayout()
		layout3.addLayout(layout1)
		layout3.addLayout(layout2)
		
		# 创建部件
		widget = QWidget()
		# 将布局添加到部件
		widget.setLayout(layout3)
		# 将部件添加到主窗口上
		self.setCentralWidget(widget)
	
	# 自定义的信号处理函数-基本命令函数
	def _my_func(self, n):
		try:
			ser1 = serial.Serial(self.combobox.currentText().split()[0], 57600, timeout=0.5)
			uav1 = UavOrder(id=self.line_edit.text())
		except:
			self.text_browser.append('串口打开失败！\n')
		try:
			if n == 0:
				ser1.write(uav1.time())
				self.text_browser.append('对时发送成功！\n')
			elif n == 1:
				ser1.write(uav1.test())
				self.text_browser.append('自检发送成功！\n')
			elif n == 2:
				ser1.write(uav1.takeoff())
				self.text_browser.append('起飞发送成功！\n')
			elif n == 3:
				ser1.write(uav1.goto(self.line_edit1_1_1.text(), self.line_edit1_1_2.text(), \
									 self.line_edit1_1_3.text(), self.line_edit1_2_1.text(),
									 self.line_edit1_2_2.text()))
				self.text_browser.append('定点飞行指令发送成功！')
				self.text_browser.append(uav1.goto(self.line_edit1_1_1.text(), self.line_edit1_1_2.text(), \
												   self.line_edit1_1_3.text(), self.line_edit1_2_1.text(),
												   self.line_edit1_2_2.text()).decode())
			elif n == 4:
				ser1.write(uav1.goback())
				self.text_browser.append('返航发送成功！\n')
			# ser1.close()
		except:
			self.text_browser.append('请确认串口... \n')
		return
	
	# 任务命令函数
	def _my_func2(self, n):

		try:
			if n == 0:
				self.excute2()
				# self.work2.start()
				# rec2send(send_id=1)
				self.text_browser.append('接转发线程启动！\n')
			if n == 1:
				global SIGNAL2
				SIGNAL2 = 0
		except:
			self.text_browser.append('接转发线程错误，检查串口冲突！\n')
		return
	# 启动实时跟随机接收数据线程
	def excute1(self):
		global SIGNAL1
		SIGNAL1 = 1
		self.work1.com = self.combobox.currentText().split()[0]
		self.work1.start()
	
	# 实时更新文本显示数据
	def display1(self, data):
		self.text_browser2.append(data)
	
	# 释放线程1
	def stop1(self):
		global SIGNAL1
		SIGNAL1 = 0
	
	# 启动接收领航机数据线程
	def excute2(self):
		global SIGNAL2
		SIGNAL2 = 1
		self.work2.start()
		
	# 显示接转发数据
	def display2(self, data2):
		self.text_browser.append(data2)
		
	# 停止接转发

# 线程1：接收跟随机数据
class WorkThread1(QThread):
	# 自定义信号对象。参数str就代表这个信号可以传一个字符串
	trigger = pyqtSignal(str)
	
	def __init__(self, com=None):
		super(WorkThread1, self).__init__()
		self.com = None
	
	def run(self):
		global SIGNAL1
		try:
			ser1 = serial.Serial(self.com, 57600, timeout=0.5)
			# print('串口号：', self.com)
			path = log_maker()
			self.trigger.emit('*****飞行日志已建立*****\n')
			uav1_data = UavData()
			while True:
				if SIGNAL1 == 0:
					self.trigger.emit('***通信串口已释放***')
					break
				data1 = ser1.readline()
				data = data1.decode()
				test1 = uav1_data.aly(data)
				uav1_data.write(path)
				uav1_data.track.append({'lon': uav1_data.pos_state['lon'], 'lat': uav1_data.pos_state['lat'],
										'time': uav1_data.time})
				if data is '':
					data = '+++未收到飞控回传数据+++\n'
				self.trigger.emit(data)
			# print('接收数据为：', test1 )
		except:
			self.trigger.emit('请确认串口正常...')


# 线程2：动态航点心跳包
class WorkThread2(QThread):
	trigger = pyqtSignal(str)
	
	def __init__(self, com=None, id=None, data=None):
		super(WorkThread2, self).__init__()
		self.id = id
		self.com = None
		self.data = data
	
	def run(self):
		global SIGNAL1
		SIGNAL1 = 0 # 使线程1释发串口
		time.sleep(0.2)
		try:
			ser1 = serial.Serial('COM7', 115200, timeout=0.05)  # 此串口当为读取领航机之单独串口。
			uav_leader_data = UavData()  # 此为从领航机所接收之数据对象
			ser2 = serial.Serial('COM10', 57600, timeout=0.3)  # 此为发指令之串口。
			uav_follower_data = UavData()
			uav_follow = UavOrder(id=1)
			path_leader = log_maker('***领航者数据***\n')
			time.sleep(1)
			path_follower = log_maker('***跟随者数据***\n')
			fig = plt.figure()
			ax1 = fig.add_subplot(1, 2, 1)
			ax2 = fig.add_subplot(1, 2, 2)
			while True:
				if SIGNAL2 == 0:
					self.trigger.emit('***接转发线程已停止***')
					break
				test_time2 = time.time()
				data_rec = ser1.readline()
				uav_leader_data.aly(data_rec.decode())
				uav_leader_data.write(path_leader)
				data_rec2 = ser2.readline()
				uav_follower_data.aly(data_rec2.decode())
				uav_follower_data.write(path_follower)
				# 北向初航向往东向初航向修正
				v_ang = None
				if uav_leader_data.v_state['v_angle'] is not None:
					v_ang = 90 - float(uav_leader_data.v_state['v_angle'])
					if abs(v_ang) < 180:
						pass
					else:
						v_ang = v_ang - 360 * np.sign(v_ang)
				data = uav_follow.goto(lon=uav_leader_data.pos_state['lon'], lat=uav_leader_data.pos_state['lat'],
				                       alt=uav_leader_data.pos_state['alt'], v_a=v_ang,
				                       v_v=uav_leader_data.v_state['v_val'])
				# print('\n转发领航机数据为：{}'.format(data))
				# print('跟随机数据为：', data_rec2.decode())
				ser2.write(data)
				self.trigger.emit('转发领航机数据：\n{}\n接收跟随机数据：\n{}\n'.format(data, data_rec2.decode()))
				
				# try:
				# ax1.scatter(float(uav_leader_data.pos_state['lon']), float(uav_leader_data.pos_state['lat']),
				#             marker='o')
				# ax1.scatter(float(uav_leader_data.pos_state['lon']), float(uav_leader_data.pos_state['lat']),
				#             marker='p')
				# ax1.xlim(109.02, 109.04)
				# ax1.ylim(34.28, 34.29)
				# plt.pause(0.1)
				# except:
				# 	print('！！！未执行画图命令！！！')
				# 	pass
				
				test_time3 = time.time()
				# 控制转发频率
				# print('应等待时间：', 0.5-(test_time3-test_time2))
				delta_time = 0.1 - (test_time3 - test_time2)
				if delta_time > 0:
					time.sleep(delta_time)
		except:
			self.trigger.emit('!!!动态心跳包未正常发送，请确认串口...！！！')


if __name__ == "__main__":
	# 创建应用实例，通过 sys.argv 传入命令行参数
	app = QApplication(sys.argv)
	# 创建窗口实例
	window = MainWindow()
	# 显示窗口
	window.show()
	# window.rec()
	# # 执行应用，进入事件循环
	app.exec_()
