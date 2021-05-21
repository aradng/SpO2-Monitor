from crccheck.crc import Crc16Ccitt
import serial
import serial.tools.list_ports
import os,sys
from io import StringIO
from time import sleep
import csv

MAX_SAMPLES = 1000

def parse_data(input):
	entry = []
	try:
		reader = csv.reader(StringIO(input), delimiter=',')
		for row in reader:
			for i in row:
				entry.append(int(i , 16))
		#print(input, end='')
	except:
		pass
	return entry

def crc16(input):
	crcinst = Crc16Ccitt()
	crcinst.reset()
	for entry in input:
		for i in entry:
			crcinst.process([i >> 8])
			crcinst.process([i & 0xff])
	return hex(crcinst.final())

while True:
	ports = list(serial.tools.list_ports.comports())
	for p in ports:
		if(p.device != 'COM1'):
			print(p.device)
			ser = serial.Serial(p.device , 115200 , timeout = 1)
			ans = ""
			data = []
			sample = []
			sampleno = 0
			successrate = 0
			lossrate = 0
			while sampleno <= MAX_SAMPLES :
				#try:
				if "crc" in ans :
					crc_input = hex(int(ans[ans.index('crc')+4:] , 16))
					crc_output = crc16(sample)
					#print(crc_input , crc_output)
					if crc_input == crc_output:
						print("█", end='')
						sys.stdout.flush()
						data.append(sample)
						successrate += 1
					else:
						print("▄", end='')
						sys.stdout.flush()
						lossrate += 1
					sample =[]
				else:
					if parse_data(ans):
						sample.append(parse_data(ans))
						sampleno += 1
				ans = ser.readline().decode('utf-8')
				#except:
				#	print("busy/not found")
				#	sleep(5)
			#print(data)
			file_index = 0
			while os.path.exists("sample_%s.csv" % file_index):
    				file_index += 1
			f = open("sample_%s.csv" % file_index , 'w')
			print("\nloss rate : %.2f%%" %(lossrate/(lossrate+successrate)))
			print("output : sample%s.csv" % file_index)
			for sample in data:
				for entry in sample:
					f.write(str(entry[0]) + ',' + str(entry[1]) + ',' + str(entry[2]) + '\n')
			f.close()
			data = []
			ser.close()