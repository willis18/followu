#!/usr/bin/python3
 
import time, serial, threading

com_timeout = 0
battery_level = 80
get_steer_level = 476
steer_minimum = 200
steer_center = 500
steer_maximum = 800
ai_enabled = 0

print("Power Steer Driver Init. (Powered by IOELECTRON)")

serial_port = serial.Serial(
	port="/dev/ttyTHS2",
	baudrate=38400,
	bytesize=serial.EIGHTBITS,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,)
rx_frames = []
thread_run = True

def serial_command_receive(rx_frame):
	global com_timeout
	global battery_level
	global get_steer_level
	global steer_minimum
	global steer_center
	global steer_maximum
	global ai_enabled

	frame_debug = False
	if frame_debug: print(rx_frame)

	try:
		parse_data = rx_frame.split(",")
		#10,211,84,488,820,1,checksum (마지막 파싱 데이터는 체크섬)
		if len(parse_data) == 7:
			rx_checksum = int(parse_data[len(parse_data)-1])
			rx_checksum_cal = -1 #체크섬은 1을 더해서 오기 때문에 미리 -1로 설정
			#수신된 체크섬 수를 합산 함.
			for c in parse_data[len(parse_data)-1]:
				rx_checksum_cal += ord(c)
			
			#전체 데이터의 체크섬을 합산함.
			sum_all = 0
			for c in rx_frame:
				sum_all += ord(c)

			#체크섬 데이터 결과를 확인함.
			if (sum_all - rx_checksum_cal) == rx_checksum:
				com_timeout = 10
				battery_level = int(parse_data[0])*10
				get_steer_level = int(parse_data[1])
				steer_minimum = int(parse_data[2])
				steer_center = int(parse_data[3])
				steer_maximum = int(parse_data[4])
				ai_enabled = int(parse_data[5])
			else:
				print("checksum error.")
		else:
			print("Rx frame error.")
	except Exception as e:
		print("Rx frame error." + str(e))
	
def serial_threading():
	global thread_run
	global rx_frames

	print("serial receive threading start.")
	while thread_run:
		try:
			for c in serial_port.read():
				rx_frames.append(chr(c))
				if c == 13:
					serial_command_receive(''.join(rx_frames))
					del rx_frames[:]
				elif len(rx_frames) > 100:
					del rx_frames[:]
		except Exception as e:
			print("Error occurred. Exiting Program: " + str(e))
			serial_port.close()
			thread_run = False

	print("serial receive threading terminated.")

def car_control_cmd(steerMode=0, setSteer=511, driveMode=0, speedLevel=0, headLamp=0, signalLamp=0, horn=0):
	'''
	[steerMode 0=>steerNormal, 1=>steerActive, 2=>steerBreak]
	[setSteer 0~1023]
	[driveMode 0=>driveNormal, 1=>driveForward, 2=>driveReverse, 3=>driveBreak]
	[speedLevel 0~99]
	[headLamp 0=>off, 1=>active]
	[signalLamp 0=>off, 1=>left, 2=>right, 3=>left,right]
	[horn 0=>off, 1=>active]
	'''
	frame_debug = False

	sendframes = ""
	if steerMode == 0:
		sendframes += "X000"
	elif steerMode == 1:
		sendframes += ("%04d" % setSteer)
	else:
		sendframes += "O000"

	if driveMode == 0:
		sendframes += "N"
	elif driveMode == 1:
		sendframes += "F"
	elif driveMode == 2:
		sendframes += "R"
	else:
		sendframes += "B"
	
	sendframes += ("%02d" % speedLevel)

	if headLamp == 1:
		sendframes += "O"
	else:
		sendframes += "X"
	
	if signalLamp == 3:
		sendframes += "E"
	elif signalLamp == 2:
		sendframes += "R"
	elif signalLamp == 1:
		sendframes += "L"
	else:
		sendframes += "N"

	if horn == 1:
		sendframes += "H"
	else:
		sendframes += "N"
	
	checksum = 1
	for c in sendframes:
		checksum += ord(c)
	
	sendframes += ("%04d" % checksum) + "\r"

	if frame_debug:
		for c in sendframes:
			print(ord(c), end=" ")
		print(sendframes)
	else:
		serial_command_send(sendframes)

def serial_command_send(tx_frame):
	'''
	주기적으로 송신시킬 것. 무한 루프에 입력시킬 경우 프레임 에러를 유발 시킴.
	권장 주기 => 10Hz(100ms) ~ 100Hz(10ms)
	?0001223456666[13] (15byte)
	?==> 'X' Steer Normal
	?==> 'O' Steer Break
	?000==> Steer Active AD Level 4byte
	1==> Car 'F' forward/ 'R' reverse/ 'B' emergency break/ other normal break
	22=> Car speed level 00~99
	3==> Car headlamp 'O' active/ other unactive
	4==> Car signallamp 'L' Left signal/ 'R' Right signal/ 'E' Hazard signal/ other off signal
	5==> Car horn 'H' active/ other unactive
	6666=> Communication checksum data sum ascii decimal and add 1
	'''
	global thread_run
	frame_debug = False

	if not thread_run: return
	try:
		if frame_debug: print(tx_frame + "=>") #Send frame check
		serial_port.write(tx_frame.encode())
	except Exception as e:
		print("send fail.\n" + str(e))

def timeout_checker():
	global com_timeout
	print_checker = False
	one_second_loop = 0
	print("driver connection checker start.")
	while(thread_run):
		if com_timeout > 0: com_timeout-=1
		one_second_loop += 1
		if one_second_loop >= 10:
			one_second_loop = 0
			if print_checker:
				print("Status: [BAT:" + str(battery_level) + "] [Steer:"+str(get_steer_level), end="")
				print(" MIN:" + str(steer_minimum) + " Cen:" + str(steer_center) + " MAX:" + str(steer_maximum), end = "")
				print(" AI Access:" + str(ai_enabled) + "]")
		time.sleep(0.1)
	print("driver connection checker terminated.")

receive_thread = threading.Timer(1, serial_threading)
receive_thread.start()

timeout_checking_thread = threading.Timer(1, timeout_checker)
timeout_checking_thread.start()