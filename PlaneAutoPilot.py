import time
import random
import math
import msvcrt
import tkinter as tk
import importlib
from importlib.machinery import SourceFileLoader


xpc = SourceFileLoader('xpc', '../../Python3/src/xpc.py').load_module()

f_run 		= True
f_control 	= True
f_override	= True

def negOne(val):
	if val < -1.0:
		return -1.0
	elif val > 1.0:
		return 1.0
	else:
		return val

def zeroOne(val):
	if val < 0.0:
		return 0.0
	elif val > 1.0:
		return 1.0
	else:
		return val

def loop(cl, l):
	global f_control
	global f_override

	# sim vars
	drefs = [
	'sim/operation/override/override_joystick',\
	'sim/operation/override/override_joystick_pitch',\
	'sim/operation/override/override_joystick_roll',\
	'sim/operation/override/override_joystick_heading',\
	'sim/operation/override/override_throttles',\
	'sim/joystick/yoke_pitch_ratio',\
	'sim/joystick/yoke_roll_ratio',\
	'sim/joystick/yoke_heading_ratio',\
	'sim/flightmodel/controls/elv_trim',\
	'sim/flightmodel/controls/ail_trim',\
	'sim/flightmodel/controls/rud_trim',\
	'sim/flightmodel/engine/ENGN_thro_use',\
	'sim/flightmodel/controls/flaprqst',\
	'sim/flightmodel/controls/sbrkrqst',\
	'sim/aircraft/parts/acf_gear_deploy',\
	'sim/flightmodel/controls/parkbrake',\
	'sim/flightmodel/position/latitude',\
	'sim/flightmodel/position/longitude',\
	'sim/flightmodel/position/elevation',\
	'sim/flightmodel/position/theta',\
	'sim/flightmodel/position/phi',\
	'sim/flightmodel/position/psi',\
	'sim/flightmodel/position/alpha',\
	'sim/flightmodel/position/indicated_airspeed',\
	'sim/flightmodel/position/true_airspeed',\
	'sim/flightmodel/position/groundspeed',\
	'sim/flightmodel/controls/nosewheel_steer',\
	'sim/flightmodel/weight/m_total',\
	'sim/flightmodel/weight/m_fuel_total',\
	'sim/flightmodel/position/magnetic_variation',\
	'sim/flightmodel/position/P',\
	'sim/flightmodel/position/Q',\
	'sim/flightmodel/position/R',\
	'sim/flightmodel/position/vh_ind'
	]
	
	# "header"
	dref_override_joystick			= 0
	dref_override_joystick_pitch	= 1
	dref_override_joystick_roll		= 2
	dref_override_joystick_yaw		= 3
	dref_override_throttle			= 4
	dref_pitch						= 5
	dref_roll						= 6
	dref_yaw						= 7
	dref_pitch_trim					= 8
	dref_roll_trim					= 9
	dref_yaw_trim					= 10
	dref_throttle					= 11
	dref_flap						= 12
	dref_speedbrake					= 13
	dref_gear						= 14
	dref_brake						= 15
	dref_latitude					= 16
	dref_longitude					= 17
	dref_elevation					= 18
	dref_theta						= 19
	dref_phi						= 20
	dref_psi						= 21
	dref_alpha						= 22
	dref_indicated_airspeed			= 23
	dref_true_airspeed				= 24
	dref_groundspeed				= 25
	dref_nosewheel_steer			= 26
	dref_m_total					= 27
	dref_m_fuel_total				= 28
	dref_magnetic_variation			= 29
	dref_P							= 30
	dref_Q							= 31
	dref_R							= 32
	dref_vertical_speed				= 33


	'''
	beta
	vpath
	hpath
	
	(rotation, deg/s)
	P (roll)
	Q (pitch)
	R (yaw)
	
	(accel, deg/s^2)
	P_dot
	Q_dot
	R_dot
	
	(rad/s)
	Prad
	Qrad
	Rrad
	
	true_theta
	true_phi
	true_psi
	'''
	
	# get data from sim
	result = cl.getDREFs(drefs)
	
	# system
	override_joystick		= result[dref_override_joystick][0]
	override_joystick_pitch	= result[dref_override_joystick_pitch][0]
	override_joystick_roll	= result[dref_override_joystick_roll][0]
	override_joystick_yaw	= result[dref_override_joystick_yaw][0]
	override_throttle		= result[dref_override_throttle][0]
	
	# additional
	pitch_trim				= result[dref_pitch_trim][0]
	roll_trim				= result[dref_roll_trim][0]
	yaw_trim				= result[dref_yaw_trim][0]
	
	###########################################################################
	# main control
	pitch					= result[dref_pitch][0]
	roll					= result[dref_roll][0]
	yaw						= result[dref_yaw][0]
	throttle				= result[dref_throttle][0]
	
	# secondary control
	flap					= result[dref_flap][0]
	speedbrake				= result[dref_speedbrake][0]
	gear					= result[dref_gear][0]
	brake					= result[dref_brake][0]
	
	# aircraft "sensors" (INDependent)
	# gyro
	Q						= result[dref_Q][0]			# pitch rotation
	P						= result[dref_P][0]			# roll rotation
	R						= result[dref_R][0]			# yaw rotation
	# gyro + accel (IMU)
	theta					= result[dref_theta][0]		# pitch (angle to horizon)
	phi						= result[dref_phi][0]		# roll (angle to horizon)
	psi						= result[dref_psi][0]		# yaw (angle to ?..)
	# pito tube
	indicated_airspeed		= result[dref_indicated_airspeed][0]
	# baro
	elevation				= result[dref_elevation][0]
	vertical_speed			= result[dref_vertical_speed][0]
	
	# aircraft "sensors" (Dependent)
	# GPS
	latitude				= result[dref_latitude][0]
	longitude				= result[dref_longitude][0]
	groundspeed				= result[dref_groundspeed][0]
	# other
	magnetic_variation		= result[dref_magnetic_variation][0]
	###########################################################################
	
	# ?
	true_airspeed			= result[dref_true_airspeed][0]
	m_total					= result[dref_m_total][0]
	m_fuel_total			= result[dref_m_fuel_total][0]
	#nosewheel_steer			= result[dref_nosewheel_steer][0]
	
	alpha					= result[dref_alpha][0]
	#beta
	
	
	# show in window
	l[dref_override_joystick		].config(text=f'override_joystick: {override_joystick:5.1f}')
	l[dref_override_joystick_pitch	].config(text=f'override_joystick_pitch: {override_joystick_pitch:5.1f}')
	l[dref_override_joystick_roll	].config(text=f'override_joystick_roll: {override_joystick_roll:5.1f}')
	l[dref_override_joystick_yaw	].config(text=f'override_joystick_yaw: {override_joystick_yaw:5.1f}')
	l[dref_override_throttle		].config(text=f'override_throttle: {override_throttle:5.1f}')
	l[dref_pitch					].config(text=f'pitch: {pitch:8.4f}')
	l[dref_roll						].config(text=f'roll: {roll:8.4f}')
	l[dref_yaw						].config(text=f'yaw: {yaw:8.4f}')
	l[dref_pitch_trim				].config(text=f'pitch_trim: {pitch_trim:8.4f}')
	l[dref_roll_trim				].config(text=f'roll_trim: {roll_trim:8.4f}')
	l[dref_yaw_trim					].config(text=f'yaw_trim: {yaw_trim:8.4f}')
	l[dref_throttle					].config(text=f'throttle: {throttle:8.4f}')
	l[dref_flap						].config(text=f'flap: {flap:6.2f}')
	l[dref_speedbrake				].config(text=f'speedbrake: {speedbrake:6.2f}')
	l[dref_gear						].config(text=f'gear: {gear:6.2f}')
	l[dref_brake					].config(text=f'brake: {brake:6.2f}')
	l[dref_latitude					].config(text=f'latitude: {latitude:8.4f}')
	l[dref_longitude				].config(text=f'longitude: {longitude:8.4f}')
	l[dref_elevation				].config(text=f'elevation: {elevation:6.2f}')
	l[dref_theta					].config(text=f'theta: {theta:6.2f}')
	l[dref_phi						].config(text=f'phi: {phi:6.2f}')
	l[dref_psi						].config(text=f'psi: {psi:6.2f}')
	l[dref_alpha					].config(text=f'alpha: {alpha:6.2f}')
	l[dref_indicated_airspeed		].config(text=f'indicated_airspeed: {indicated_airspeed:6.2f}')
	l[dref_true_airspeed			].config(text=f'true_airspeed: {true_airspeed:6.2f}')
	l[dref_groundspeed				].config(text=f'groundspeed: {groundspeed:6.2f}')
	#l[dref_nosewheel_steer			].config(text=f'nosewheel_steer: {nosewheel_steer:6.2f}')
	l[dref_m_total					].config(text=f'm_total: {m_total:6.2f}')
	l[dref_m_fuel_total				].config(text=f'm_fuel_total: {m_fuel_total:6.2f}')
	l[dref_magnetic_variation		].config(text=f'magnetic_variation: {magnetic_variation:6.2f}')
	l[dref_P						].config(text=f'P(roll): {P:8.4f}')
	l[dref_Q						].config(text=f'Q(pitch): {Q:8.4f}')
	l[dref_R						].config(text=f'R(yaw): {R:8.4f}')
	l[dref_vertical_speed			].config(text=f'vertical_speed: {vertical_speed:6.2f}')
	
	#val = (-1 * theta * 0.3) + (-1 * (indicated_airspeed-30) * 0.3) + 1
	#val = -1 * theta * 0.01 + 0.5
	#val = 0.6 + ((vertical_speed-1) * -1 * 0.05)
	#cl.sendDREF(drefs[dref_throttle], zeroOne(val))
	
	#val = -1 * theta * 0.4
	
	if f_override:
		cl.sendDREF(drefs[dref_override_joystick_pitch], f_control)
		cl.sendDREF(drefs[dref_override_joystick_roll], f_control)
		cl.sendDREF(drefs[dref_override_joystick_yaw], f_control)
		cl.sendDREF(drefs[dref_override_throttle], f_control)
		
		f_override = False
	
	if f_control:
		# порядок контроля:
		# заданная_скорость -> скорость(+проверка ограничений, макс,мин) -> тангаж(+проверка ограничений, угол атаки и тп.)
		# заданная_высота -> высота(+проверка ограничений, макс, мин(земля,горы)) -> вертикальная_скорость(+проверка ограничений) -> мощность_двигателя(+ограничения 0..1, минимальные обороты, опрокидывание и тп)
		# заданное_направление:
		# (проверка ограничений(максимальный_угол_крена, радиус_поворота)
		#  крен + тангаж для входа в круг.
		#  руль направления для компенсации скольжения
		#  влияет на скорость, компенсировать двигателями)
		# 
		# оси нужно пересчитывать учитывая текущее положение в пространстве (не допускать управление одной осью пока она находится в другой плоскости (чем обычно) и управление не будет выполнять нужные действия, а будет влиять на другую ось... * cos())
		# 
		# ограничивать скорости вращений сигналом с гироскопов.
		
		# Направление полета
		hspeed 		= 80
		elev		= 1500
		heading 	= 244.73 + 90 #- 180
		
		vspeed_lim	= 10
		pitch_lim	= 25
		roll_lim	= 35
		
		# нужно для диапазона "-180"..нужное_направление.."+180"
		# что бы расчитанное управляющее воздействие
		# было в диапазоне от -180 до +180 с 0 по середине.
		# изначальный диапазон направления (0..360 градусов)
		#
		# начинает делать круг на крайних значениях нужного_направления
		# потому что при уходе за 0 или 360 управляющее воздействие меняет направление...
		# что бы это избежать - делаем нужное_направление серединой значений
		# а отклонения как в минус так и в плюс, одинаковыми и = 180(+ и -) град.
		#
		# короче смещаем середину диапазона psi, делая серединой нужное_направление (heading).
		#
		# FIXME найти минимальное значение для выбора направления поворота. (поворот вправо на 45 град, делается через левый разворот...)
		hmin = heading - 180
		if psi < hmin:
			fix_psi = psi + 360
		else:
			fix_psi = psi
	
		# PID - P
		pitch_P	= 0.2
		roll_P	= 0.2
		yaw_P	= 0.1
		
		# PID - I
		pitch_I	= 0
		roll_I	= 0
		yaw_I	= 0
		
		# отрицательная обратная связь - отрицательный коэффициент
		neg_feedback = -1
		
		# hspeed = needspeed с ограничением по углу атаки и максимального допустимого значения.
		# yaw от скольжения а не heading
		# roll и питч от круга для виража -> скольжение -> компенсация_yaw'ом.
		
		# нормализация() * максимальное_отклонение
		pitch_angle	= negOne((indicated_airspeed - hspeed) * 0.02) * pitch_lim
		roll_angle	= negOne((fix_psi - heading) * neg_feedback	* 0.02) * roll_lim
		vspeed		= negOne((elevation - elev) * neg_feedback * 0.02) * vspeed_lim
		
		# значение  = нормализация((ось обратной связи - нужное положение) * отрицательная связть * PID) * уменьшить воздействие при "неправильных" положениях(ось зависимости)
		pitch_val	= negOne((theta - pitch_angle)	* neg_feedback	* pitch_P)	* math.cos(math.radians(phi))
		roll_val	= negOne((phi - roll_angle)		* neg_feedback	* roll_P)	* math.cos(math.radians(theta))
		yaw_val		= negOne((fix_psi - heading)	* neg_feedback	* yaw_P)	* math.cos(math.radians(phi))
		
		throttle_val = zeroOne(0.5 + (vertical_speed - vspeed) * neg_feedback * 0.1)
		
		cl.sendDREF(drefs[dref_pitch], pitch_val)
		cl.sendDREF(drefs[dref_roll], roll_val)
		cl.sendDREF(drefs[dref_yaw], yaw_val)
		
		cl.sendDREF(drefs[dref_throttle], throttle_val)
	pass

def quit(event):
	global f_run
	
	f_run = False

def control(event):
	global f_control
	global f_override
	
	f_override = True
	if f_control:
		f_control = False
		print('[CONTROL] - Control is OFF')
	else:
		f_control = True
		print('[CONTROL] - Control is ON')

def main():
	global f_run
	global f_control
	global f_override

	client = xpc.XPlaneConnect()
	
	root = tk.Tk()
	root.title('XPC')
	root.minsize(300, 200)
	root.geometry('+1640+60')
	root.bind('q', quit)
	root.bind('c', control)
	labels = list()
	for i in range(0,40):
		l = tk.Label(root, text='')
		l.pack()
		labels.append(l)
	
	while f_run:
		try:
			loop(client, labels)
		except:
			print('[LOOP] - EXCEPTION')
			print('[LOOP] - Wait 2 second')
			time.sleep(2)
			print('[LOOP] - Restart...')
		
		root.update()
		
		# exit by press 'q' key in console
		if msvcrt.kbhit():
			key = msvcrt.getch()
			if ord(key) == ord('q'):
				f_run = False
			elif ord(key) == ord('c'):
				control(None)
	
	client.close()


if __name__ == '__main__':
	main()
