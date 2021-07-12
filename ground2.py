'''
项目：无人争锋地面站，采用面向对象方法
负责人：王宁
时间：0629
'''
import serial
import os
import matplotlib.pyplot as plt
import time
import threading
import numpy as np
import struct
import math
from binascii import unhexlify


class UavData:
	'''
	接收到的UAV数据对象，包含8个属性，3个方法
	'''
	def __init__(self):
		self.name = 'uavdata'
		self.id = None
		self.pos_state = {'lat':None, 'lon':None, 'alt':None}
		self.v_state = {'v_angle':None, 'v_val':None}
		self.expec_v = {'v_N':None, 'v_E':None}
		self.dis = None
		self.time = None
		self.track = []

	def aly(self, rec_da): # 根据接收到的数据更新跟随机参数
		data = rec_da.split(';')
		if ('$sv' in data[0]) and ('vs$' in data[-1]): # 数据校验通过后解析
			for i in data:
				data_tem = i.split(':')
				if len(data_tem) == 2:
					if data_tem[0] == '$svid':
						self.id = data_tem[1]
					elif data_tem[0] == 'w' or data_tem[0] == 'lat':
						self.pos_state['lat'] = data_tem[1]
					elif data_tem[0] == 'j' or data_tem[0] == 'lon':
						self.pos_state['lon'] = data_tem[1]
					elif data_tem[0] == 'h' or data_tem[0] == 'alt':
						self.pos_state['alt'] = data_tem[1]
					elif data_tem[0] == 'y' or data_tem[0] == 'v_ang':
						self.v_state['v_angle'] = data_tem[1]
					elif data_tem[0] == 'v' or data_tem[0] == 'v_v':
						self.v_state['v_val'] = data_tem[1]
					elif data_tem[0] == 'v_N':
						self.expec_v['v_N'] = data_tem[1]
					elif data_tem[0] == 'v_E':
						self.expec_v['v_E'] = data_tem[1]
					elif data_tem[0] == 'dis':
						self.dis = data_tem[1]
					elif data_tem[0] == 'time':
						self.time = data_tem[1]
					else:
						pass
				else:
					pass
		return
	
	def write(self, path_write):
			try:
				with open(path_write,'a') as f:
					f.write('{}\t{}\t{}\t{}\t{}\t{}\n'.format(self.id, self.pos_state, self.v_state,
															  self.expec_v, self.dis, self.time))
			except:
				print('!!!数据未写入飞行日志!!!')

	def paint_master(self):
		if self.track is not None:
			x = [float(i['lon']) for i in self.track]
			y = [float(i['lat']) for i in self.track]
		return x, y

class UavOrder:
	'''
	用于给无人机发送指令的对象，包含1个属性，5个方法
	'''
	def __init__(self, id):
		self.id = id

	def takeoff(self, alt=10):
		self.alt = alt
		return '$svid:{};cmd:takeoff;alt:{};vs$'.format(self.id, self.alt).encode()

	def test(self):
		return '$svid:{};cmd:test;vs$'.format(self.id).encode()

	def goto(self,id=1, lon=None,lat=None,alt=None,v_a=0.000,v_v=None,time=None):
		return '$svid:{};cmd:goto;lon:{};lat:{};alt:{};v_a:{};v_v:{};time:{:.3};vs$'.format(id, lon, lat, alt,
		                                                                             v_a if v_a is None else round(v_a,3),
																				     v_v, time).encode()
	
	def time(self):
		return '$svid:{};cmd:time;vs$'.format(self.id).encode()
	
	def goback(self):
		return '$svid:{};cmd:return;vs$'.format(self.id).encode()

class LeaderData(UavData):
	def __init__(self):
		super().__init__() # 继承了位置数据self.pos_state，速度数据self.v_state
		self.name = 'leader_data'
		self.id = 'leader'
		self.head = None # 数据帧头
		self.num = None # 设备编号
		self.sal_num = None # 收星号
		self.state = None # 工作状态：单点、RTK、浮点
		self.value = None # 椭球分离值
		self.date = {'year':None, 'month':None, 'day':None, 'hour':None, 'min':None, 'sec':None, 'ms':None}
		self.crc = None
		self.end1 = None
		self.end2 = None
		
	def aly(self, rec_data):
		'''
		拿到串口解析后的数据更新对象属性
		'''
		# data_tem = unhexlify(rec_data) # 此处看情况，如果对方发数据没有按16进制，我就自己做转化
		data_tem = rec_data
		data = struct.unpack('<BBBBfffddHBBBBBBHBB', data_tem)
		self.head, self.num, self.sal_num, self.state, self.value, self.v_state['v_val'], self.pos_state['alt'],\
			self.pos_state['lat'], self.pos_state['lon'], self.date['year'], self.date['month'], self.date['day'],\
			self.date['hour'], self.date['min'], self.date['sec'], self.date['ms'], self.crc, self.end1, self.end2\
			= data
		
# 用于创建飞行路径，返回1个txt文件地址。
def log_maker(start_word = ''):
	t = time.gmtime()
	t = time.strftime("%Y-%m-%d_%H-%M-%S", t)
	try:
		data_log = open(os.getcwd()+'/'+'飞行日志/'+'{}.txt'.format(t), 'w')
	except:
		os.mkdir(os.getcwd()+'/'+'飞行日志')
		data_log = open(os.getcwd() + '/' + '飞行日志/' + '{}.txt'.format(t), 'w')
	data_log.write('***飞行日志： {}***\n'.format(time.ctime()))
	data_log.write(start_word)
	data_log.close()
	return os.getcwd()+'/'+'飞行日志/'+'{}.txt'.format(t)

# 用于接收跟随机数据
def rec():
	ser1 = serial.Serial('COM8', 115200, timeout=0.5)
	path = log_maker()
	uav1_data = UavData()
	while True:
		data1 = ser1.readline()
		data = data1.decode()
		uav1_data.aly(data)
		uav1_data.write(path)
		uav1_data.track.append({'lon':uav1_data.pos_state['lon'], 'lat':uav1_data.pos_state['lat'],
							   'time':uav1_data.time})
		print('rec leader data:', data)
		time.sleep(0.5)

# 用于接转发，不可与接收进程并行。
def rec2send(send_id=1,com_rec='COM7',com_send='COM10'):
	ser1 = serial.Serial(com_rec, 115200, timeout=0.05)  # 此串口当为读取领航机之单独串口。
	uav_leader_data = UavData() # 此为从领航机所接收之数据对象
	# uav_leader_data = L**eaderData()
	ser2 = serial.Serial(com_send, 57600, timeout=0.3) # 此为发指令之串口。
	uav_follower_data = UavData()
	uav_follow = UavOrder(id = send_id)
	path_leader = log_maker('***领航者数据***\n')
	time.sleep(1)
	path_follower = log_maker('***跟随者数据***\n')
	fig = plt.figure()
	ax1 = fig.add_subplot(1, 2, 1)
	ax2 = fig.add_subplot(1, 2, 2)
	time1 = time.time()
	while True:
		test_time2 = time.time()
		data_rec = ser1.readline()
		uav_leader_data.aly(data_rec.decode())
		# uav_leader_data.aly(data_rec) 此句用于实赛
		uav_leader_data.track.append([uav_leader_data.pos_state['lon'], uav_leader_data.pos_state['lat']])
		uav_leader_data.write(path_leader)
		data_rec2 = ser2.readline()
		uav_follower_data.aly(data_rec2.decode())
		uav_follower_data.track.append([uav_follower_data.pos_state['lon'], uav_follower_data.pos_state['lat']])
		uav_follower_data.write(path_follower)
		# 根据领航者航迹解算其航向
		if len(uav_leader_data.track) > 10:
			v_ang = yaw_solve(uav_leader_data.track[-1][1], uav_leader_data.track[-1][1],
			                  uav_leader_data.track[-10][1], uav_leader_data.track[-10][0])
			print('领航机航向为：', v_ang) # 测试用
		else:
			pass
		# 北向初航向往东向初航向修正 自测用
		# v_ang = None
		# if uav_leader_data.v_state['v_angle'] is not None:
		# 	v_ang = 90 - float(uav_leader_data.v_state['v_angle'])
		# 	if abs(v_ang) < 180:
		# 		pass
		# 	else:
		# 		v_ang = v_ang - 360 * np.sign(v_ang)
		data = uav_follow.goto(id=task_dispatch(v_ang),lon=uav_leader_data.pos_state['lon'], lat=uav_leader_data.pos_state['lat'],
							   alt=uav_leader_data.pos_state['alt'], v_a=v_ang, v_v=uav_leader_data.v_state['v_val'],
		                       time=time.time()-time1)
		print('\n转发领航机数据为：{}'.format(data))
		print('跟随机数据为：', data_rec2.decode())
		ser2.write(data)
		print('testdata',uav_leader_data.pos_state['lon'])
		ax1.cla()
		# print(uav_follower_data.track)
		ax1.plot(uav_leader_data.track, marker='o')
		ax1.plot(uav_follower_data.track, marker='p')
		# plt.show()
		plt.pause(0.1)
		
		test_time3 = time.time()
		# 控制转发频率
		# print('应等待时间：', 0.5-(test_time3-test_time2))
		delta_time = 0.1 - (test_time3-test_time2)
		if delta_time > 0:
			time.sleep(delta_time)

# 根据领航机航迹解算航向
def yaw_solve(lat1, lon1, lat2, lon2):
	'''
	输入：点1经纬度，点2经纬度
	输出：点1指向点2的矢量与正北方向的夹角,顺时针为正，输出范围【0 - 360】
	'''
	Pos1Lat = lat1 * 10000000.0
	Pos1Lon = lon1 * 10000000.0
	Pos2Lat = lat2 * 10000000.0
	Pos2Lon = lon2 * 10000000.0
	dLat = (Pos2Lat - Pos1Lat) / 10000000.0
	scale = math.cos((Pos2Lat) / 10000000.0 * 3.1415926 / 180.0)
	dLon = ((Pos2Lon - Pos1Lon)) / 10000000.0 * scale
	dLat = dLat * 1.113195 * 100000.0
	dLon = dLon * 1.113195 * 100000.0
	degree = math.atan2(dLon, dLat) * 180 / math.pi
	return float((degree + 360.0) % 360.0)

# def task_dispatch1(leader_yaw):
# 	first_flight = [-20, 20] # 引导第1趟飞行的正航向
# 	times1 = 0 # 发出第1次正引导后置1
# 	second_flight = [160, 180]
# 	times2 = 0 # 完成第1次负引导后置1
# 	if times1 == 0 & leader_yaw>first_flight[0] & leader_yaw<first_flight[1]:
# 		id = 1
# 	elif times1 == 0 & times2 == 0 & leader_yaw>second_flight[0] & leader_yaw<second_flight[1]:
# 		id = 2
# 		times1 = 1
# 	elif times1 == 1 & leader_yaw>first_flight[0] & leader_yaw<first_flight[1]:
# 		id = 3
# 		times2 = 1
# 	elif times1 == 1 & times2 == 1 & leader_yaw>second_flight[0] &leader_yaw<second_flight[1]:
# 		id = 4
# 	# if leader_yaw == None:
# 	# 	id = None
# 	# elif leader_yaw == 100:
# 	# 	id = 1
# 	# else:
# 	# 	id = 2
#
# 	return id
# 任务调度
def task_dispatch(times1=0, times2=0): # 这里使用闭包来记忆飞行的圈数
	def task_dispatch2(leader_yaw):
		nonlocal times1, times2
		first_flight = [-20, 20]  # 引导第1趟飞行的正航向
		# times1 = 0  # 发出第1次正引导后置1
		second_flight = [160, 180]
		# times2 = 0  # 完成第1次负引导后置1
		id = 0
		if times1 == 0 & leader_yaw > first_flight[0] & leader_yaw < first_flight[1]:
			id = 1
		elif times1 == 0 & times2 == 0 & leader_yaw > second_flight[0] & leader_yaw < second_flight[1]:
			id = 2
			times1 = 1
		elif times1 == 1 & leader_yaw > first_flight[0] & leader_yaw < first_flight[1]:
			id = 3
			times2 = 1
		elif times1 == 1 & times2 == 1 & leader_yaw > second_flight[0] & leader_yaw < second_flight[1]:
			id = 4
		return id
	return task_dispatch2
		
def main():
	# thread1 = threading.Thread(target=rec(), args=())
	thread2 = threading.Thread(target=rec2send(send_id=1), args=())

	# thread1.start()
	# thread1.join()
	thread2.start()

if __name__ =='__main__':
	main()
	# ser1 = serial.Serial('COM10', 87600, timeout=0.5)
	# uav1 = UavOrder(id=1)
	# ser1.write(uav1.goto())