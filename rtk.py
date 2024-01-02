#
# Parseur de logs NMEA
#
# Références  :
#       https://gpsd.gitlab.io/gpsd/NMEA.html
#       https://anavs.com/knowledgebase/nmea-format
#
# Types de trames présents dans le log LABO :
#                   DAT TIM LAT LON ALT VIT
#       $GNTXT       -   -   -   -   -   -
#       $GNRMC       x   x   x   x   -   x
#       $GNVTG       -   -   -   -   -   x
#       $GNGGA       -   x   x   x   x   x
#       $GNGSA       -   -   -   -   -   -
#       $GPGSV       -   -   -   -   -   -
#       $GLGSV       -   -   -   -   -   -
#       $GNGLL       -   x   x   x   -   -
# /=====================================================================
# | HISTORIQUE - Script de décodage de trames NMEA
# |---------------------------------------------------------------------
# | Changelog : [*] Bug fixes
# |             [+] New function
# |             [-] Update
# |-------+------------+------------------------------------------------
# | VERS  | DATE       | EVOLUTIONS
# |-------+------------+------------------------------------------------
# |       |            |
# | v0.1x | 04/01/2024 | + Ouvre le port serie a la vollee si antenne branchee
# |       |            | + Infos systeme sur btnUp
# |       |            | + Nouveaux dossiers et renommages
# |       |            | + Pilotage led avant et integ boitier
# |       |            | * Correction des fichiers NMEA
# |       |            | + Decodage des logs en sortie de programme
# |       |            | - Ammelioration de l'affichage LCD
# |       |            | - Affichage IP wifi/eth
# |       |            |
# | v0.0x | 22/12/2023 | + Premiere version fonctionnelle
# |       |            |
# \---------------------------------------------------------------------
# git gui &
# git add rtk.py

import sys
sys.path.append('../')
import rgb1602
import time
from datetime import datetime
import RPi.GPIO as GPIO
import serial
import subprocess
from os.path import exists
import psutil
import gpx_lib

from gpiozero import CPUTemperature

_version = "0.12"

posFix=[ "NOFX","GPS ", "DGPS", "N/A ", "RTKx", "RTKf", "INS "]

lcd=rgb1602.RGB1602(16,2)   

btnRIGHT  = 0
btnUP     = 1
btnDOWN   = 2
btnLEFT   = 3
btnSELECT = 4

glbLat    = 0.0
glbLon    = 0.0
glbFix    = 1
glbNSat   = 0
glbHeight = 0.0
glbHDop   = 0.0
glbFreeze	= 0
glbSortir	= False
serOpen		= False
tgl			= False
logToggle	= True

glbWifiAd = "0.0.0.0"
glbEthAd  = "0.0.0.0"

MAX_TRAMES=10000  
TM_FREEZE=10
TramesNMEA=[]

Constellation = {	"GP": "GPS only",
						"GL": "GLONASS",
						"GA": "GALILEO",
						"GN": "multi GNSS"	}

#=======================================================================
# CustomChar
#-----------------------------------------------------------------------
#
# 
def CustomChar():
	cVide = [	0b00011,
					0b00011,
					0b00011,
					0b00011,
					0b00000,
					0b00000,
					0b00000,
					0b00000 ]
	cPlein = [  0b00000,
					0b00000,
					0b00000,
					0b00000,
					0b00011,
					0b00011,
					0b00011,
					0b00011 ]
	cCroix = [  0b11111,
					0b11001,
					0b11011,
					0b10101,
					0b10101,
					0b11011,
					0b10001,
					0b11111 ]  
	# create a new character
	lcd.customSymbol(0, cVide) 
	lcd.customSymbol(1, cPlein)   
	lcd.customSymbol(2, cCroix)  

#=======================================================================
# Init_LCD
#-----------------------------------------------------------------------
#
# 
def Init_LCD():
	print("> Init_LCD")
	GPIO.setmode(GPIO.BCM)
	GPIO.setwarnings(False)
	GPIO.setup(16, GPIO.IN)
	GPIO.setup(17, GPIO.IN)
	GPIO.setup(18, GPIO.IN)
	GPIO.setup(19, GPIO.IN)
	GPIO.setup(20, GPIO.IN)
	GPIO.setup(21, GPIO.OUT)
	GPIO.setup(13, GPIO.OUT)
	GPIO.output(13, GPIO.HIGH)
	
#=======================================================================
# read_buttons
#-----------------------------------------------------------------------
# Read the key value
# 
def read_LCD_buttons():
	key_in16 = GPIO.input(16)
	key_in17 = GPIO.input(17)
	key_in18 = GPIO.input(18)
	key_in19 = GPIO.input(19)
	key_in20 = GPIO.input(20)

	if (key_in16 == 1):
		print("\n> ReadBTN:btnSELECT\n")
		return btnSELECT
	if (key_in17 == 1):
		print("\n> ReadBTN:btnUP")
		return btnUP
	if (key_in18 == 1):
		print("\n> ReadBTN:btnDOWN")
		return btnDOWN
	if (key_in19 == 1):
		print("\n> ReadBTN:btnLEFT")
		return btnLEFT
	if (key_in20 == 1):
		print("\n> ReadBTN:btnRIGHT")
		return btnRIGHT

#=======================================================================
# DecodeGGA 
#-----------------------------------------------------------------------
# $GNGGA,122220.00,4544.6212848,N,00450.0667166,E,2,12,0.60,188.552,M,47.358,M,,0136*4A
# Field	Meaning
# 0	Message ID $GPGGA
# 1	UTC of position fix
# 2	Latitude
# 3	Direction of latitude:
#     N: North
#     S: South
# 4	Longitude
# 5	Direction of longitude:
#     E: East
#     W: West
# 6	GPS Quality indicator:
#     0: Fix not valid
#     1: GPS fix
#     2: Differential GPS fix (DGNSS), SBAS, OmniSTAR VBS, Beacon, RTX in GVBS mode
#     3: Not applicable
#     4: RTK Fixed, xFill
#     5: RTK Float, OmniSTAR XP/HP, Location RTK, RTX
#     6: INS Dead reckoning
# 7	Number of SVs in use, range from 00 through to 24+
# 8	HDOP
# 9	Orthometric height (MSL reference)
# 10	M: unit of measure for orthometric height is meters
# 11	Geoid separation
# 12	M: geoid separation measured in meters
# 13	Age of differential GPS data record, Type 1 or Type 9. Null field when DGPS is not used.
# 14	Reference station ID, range 0000 to 4095. A null field when any reference station ID is selected and no corrections are received. See table below for a description of the field values.
# 15	The checksum data, always begins with *

def DecodeGGA(trame):
	global glbLat,glbLon,glbFix,glbNSat,glbHeight,glbHDop

	print ("> DecodeGGA:" + Constellation[trame[1:3]])
	try:
		fields =trame.split(",")
		lcLat    = float(fields[2])/100
		lcLon    = float(fields[4])/100
		lcFix    = int(fields[6])
		lcNSat   = int(fields[7])
		lcHeight = int(float(fields[9]))
		lcHDop   = float(fields[8])
		if ((lcLat < 50) & (lcLat > 40) & (lcLon>3) & (lcLon<6)):
			glbLat   = lcLat   
			glbLon   = lcLon   
			glbFix   = lcFix   
			glbNSat  = lcNSat  
			glbHeight= lcHeight
			glbHDop  = lcHDop  
							 
	except:
		#glbLat    = 0.0
		#glbLon    = 0.0
		#glbFix    = 0
		#glbNSat   = 0
		#glbHeight = 0
		#glbHDop   = 0
		pass

#=======================================================================
# InfoSys 
#-----------------------------------------------------------------------
# 'lo': [
#    snicaddr(family=<AddressFamily.AF_INET: 2>, address='127.0.0.1', netmask='255.0.0.0', broadcast=None, ptp=None),
#    snicaddr(family=<AddressFamily.AF_INET6: 10>, address='::1', netmask='ffff:ffff:ffff:ffff:ffff:ffff:ffff:ffff', broadcast=None, ptp=None),
#    snicaddr(family=<AddressFamily.AF_PACKET: 17>, address='00:00:00:00:00:00', netmask=None, broadcast=None, ptp=None)],
# 'eth0': [
#     snicaddr(family=<AddressFamily.AF_INET: 2>, address='192.168.1.213', netmask='255.255.255.0', broadcast='192.168.1.255', ptp=None),
#     snicaddr(family=<AddressFamily.AF_INET6: 10>, address='2001:861:206:35b0:6fff:cd76:dc8e:4d32', netmask='ffff:ffff:ffff:ffff::', broadcast=None, ptp=None),
#     snicaddr(family=<AddressFamily.AF_INET6: 10>, address='fe80::d2ea:e237:97b0:b458%eth0', netmask='ffff:ffff:ffff:ffff::', broadcast=None, ptp=None),
#     snicaddr(family=<AddressFamily.AF_PACKET: 17>, address='b8:27:eb:75:c2:6f', netmask=None, broadcast='ff:ff:ff:ff:ff:ff', ptp=None)],
# 'wlan0': [
#     snicaddr(family=<AddressFamily.AF_INET: 2>, address='192.168.1.214', netmask='255.255.255.0', broadcast='192.168.1.255', ptp=None),
#     snicaddr(family=<AddressFamily.AF_INET6: 10>, address='2001:861:206:35b0:5e8:74b1:ccd9:c407', netmask='ffff:ffff:ffff:ffff::', broadcast=None, ptp=None),
#     snicaddr(family=<AddressFamily.AF_INET6: 10>, address='fe80::959f:e103:8fb7:9f33%wlan0', netmask='ffff:ffff:ffff:ffff::', broadcast=None, ptp=None),
#     snicaddr(family=<AddressFamily.AF_PACKET: 17>, address='b8:27:eb:20:97:3a', netmask=None, broadcast='ff:ff:ff:ff:ff:ff', ptp=None)]}
#
def GetIPaddress():
	global glbFreeze
	lcd.clear()	

	ad=psutil.net_if_addrs()
	glbWifiAd = ad["wlan0"][0].address
	glbEthAd  = ad["eth0"][0].address

	l1 = "E:"+glbEthAd
	l2 = "W:"+glbWifiAd
	lcd.setCursor(0, 0)
	lcd.printout(l1)      
	lcd.setCursor(0, 1)
	lcd.printout(l2)
	lcd.write(0)

	glbFreeze=TM_FREEZE
	print("> GetIPaddress: ["+l1+"] ["+l2+"]")
	
#=======================================================================
# InfoSys 
#-----------------------------------------------------------------------
#
def InfoSys():
	global glbFreeze
	lcd.clear()
	
	cpu = CPUTemperature()
	cpufreq = psutil.cpu_freq()
	svmem = psutil.virtual_memory()

	l1=format("CPU:%2.0f%% MEM:%2.0f%%" % (psutil.cpu_percent(),svmem.percent))
	l2=format("T:%2.0f°c F:%dHz"   % (cpu.temperature,cpufreq.current))
	
	#l1=format("T:%4.1f° M:%4.1f%%" % (cpu.temperature,svmem.percent))
	#l#2=format("C:%3.0f%% F:%dHz" % (psutil.cpu_percent(),cpufreq.current))
	lcd.setCursor(0, 0)
	lcd.printout(l1)      
	lcd.setCursor(0, 1)
	lcd.printout(l2)	
	lcd.write(0)

	glbFreeze=TM_FREEZE
	print("> InfoSys: ["+l1+"] ["+l2+"]")
	
#=======================================================================
# SearchUSBport 
#-----------------------------------------------------------------------
#
def SearchUSBport():
	usbPort=""
	for port in range (0,10):
		ttyPort="/dev/ttyUSB"+str(port)
		if exists(ttyPort):
			usbPort=ttyPort
			print("> SearchUSBport: Port série="+usbPort)
			break
	return(usbPort)

#=======================================================================
# MAIN 
#-----------------------------------------------------------------------
#
lcd_key   = 0

Init_LCD()
lcd.clear()

CustomChar()
l1="Rasperry - v"+_version
l2="Parsing logs..."
lcd.setRGB(0,0,250)
lcd.clear()
lcd.setCursor(0,0)
lcd.printout(l1)
lcd.setCursor(0,1)
lcd.printout(l2)
print("> Main: [%s] [%s]" % (l1,l2))

time.sleep(5)

dt=datetime.now().strftime("%Y%m%d_%H%M%S")
nmeaPath="/home/blueb/RTK/nmea/nmea-"+dt+"(init).nmea"
loopCtr =0
nmeaData = ""
lcd.setRGB(255,0,0)
while (loopCtr < (60*60*10*5)) & (not glbSortir):  #Tourne 10h
	loopCtr=loopCtr+1
	if ((loopCtr % 5) == 0):
		if tgl:
			tgl=False
		else:
			tgl=True

	#--------------------------------------------------------------------
	# Détection et ouverture du port serie si non fait
	#
	if not serOpen:
		try:
			serPath=SearchUSBport()
			if (len(serPath)>0):
				print("> Main: Essai ouverture ["+serPath+"]")
				serPort=serial.Serial(	port=serPath,
												baudrate=115200,
												parity=serial.PARITY_NONE,
												stopbits=serial.STOPBITS_ONE,
												bytesize=serial.EIGHTBITS,
												timeout=0.1)
				serOpen=True
				lcd.setRGB(0,255,0)
				serPort.flush()
				print(">  SUCCESS")
			else:
				l1="*** ERROR ***"
				l2="No RTK receiver"
				lcd.clear()
				lcd.setCursor(0, 0)
				lcd.printout(l1)
				lcd.setCursor(0, 1)
				lcd.printout(l2)
				print("> Main: ["+l1+"] ["+l2+"]")		
		except:
			print(">  FAILED")
			lcd.setRGB(255,0,0)
			serOpen=False

	#--------------------------------------------------------------------
	# Vidage du buffer port serie
	#
	while True:
		try:
			if serOpen:
				nmeaData = serPort.readline().strip()
				nmeaData = nmeaData.decode('utf-8')
				codes=nmeaData.split(",")
				if len(codes) > 1:
					code=nmeaData.split(",")[0]
					if (code[3:]=="GGA"):
						#DecodeGGA("$GNGGA,122220.00,4544.6212848,N,00450.0667166,E,2,12,0.60,188.552,M,47.358,M,,0136*4A")
						DecodeGGA(nmeaData)
		except:
				l1="** EXCEPTION **"
				l2="Serial read error"
				#lcd.clear()
				#lcd.setCursor(0, 0)
				#lcd.printout(l1)
				#lcd.setCursor(0, 1)
				#lcd.printout(l2)
				print("> Main: ["+l1+"] ["+l2+"]")	
				serOpen=False
				break
		if len(nmeaData)>1:
			print("      > " + nmeaData)
			dt=datetime.now().strftime("%H:%M:%S:%f")
			TramesNMEA.append(dt + ";" + nmeaData)
		else:
			break    
			
	#--------------------------------------------------------------------
	# Detection si bouton pressé
	#
	lcd_key = read_LCD_buttons()  #  Reading keys
	if (lcd_key == btnRIGHT):
		lcd.setRGB(40,40,40)
	elif (lcd_key == btnLEFT):
		#
		# GAUCHE=lancer/arreter la capture des trames
		#
		lcd.clear()
		lcd.setCursor(0, 0)
		if logToggle:
			print("> Main: Recording OFF")    
			logToggle=False
			lcd.printout("Recording OFF")      
		else:
			print("> Main: Recording ON")    
			logToggle=True
			lcd.printout("Recording ON")  
	elif (lcd_key == btnUP):
		#
		# UP=infos system
		#
		InfoSys()
		
	elif (lcd_key == btnDOWN):
		#
		# DOWN=affichage adresses IP wifi et eth
		#
		GetIPaddress()
	elif (lcd_key == btnSELECT):
		#
		# RIGHT=Quitter l'appli
		#
		glbSortir = True

	#--------------------------------------------------------------------
	# Symbole d'activité clignotant, different si enregisrement actif ou pas
	#  
	lcd.setCursor(15, 1)
	if logToggle:
		if ((loopCtr % 2) == 0):
			GPIO.output(21, GPIO.HIGH)				
			lcd.write(0)
		else:
			GPIO.output(21, GPIO.LOW)				
			lcd.write(1)
	else:
		if ((loopCtr % 2) == 0):
			GPIO.output(21, GPIO.HIGH)				
			lcd.write(0)
		else:
			GPIO.output(21, GPIO.LOW)				
			lcd.write(2)

	#--------------------------------------------------------------------
	# Gestion de l'affichage LCD avec les infos GNSS
	#  
	if (glbFreeze==0):
		if ((loopCtr % 5) == 0):
			lcd.clear()
			lcd.setCursor(0,0)
			l1=format("%08.5f %07.5f.." % (glbLat,glbLon))
			lcd.printout(l1)
			lcd.setCursor(0,1)
			if tgl:
				l2=format("Sat:%02d Fx:%s " % (glbNSat,posFix[glbFix]))
			else:
				l2=format("%0.3dm HDp:%06.3f " % (glbHeight,glbHDop))
			lcd.printout(l2)
			lcd.write(0)
	else:
		glbFreeze=glbFreeze-1
	#--------------------------------------------------------------------
	# Si tableau de trames plein, ecrire dans un fichier daté
	#    
	if len(TramesNMEA)>MAX_TRAMES:
		print("> Main: Purge liste")
		if logToggle:
			with open(nmeaPath, "w") as nmeaHdlr:
				for item in TramesNMEA:
					nmeaHdlr.write("\n"+item)
			nmeaHdlr.close()
			dt=datetime.now().strftime("%Y%m%d_%H%M%S")
			nmeaPath="/home/blueb/RTK/nmea/nmea-"+dt+".nmea"
		TramesNMEA=[]

	#--------------------------------------------------------------------
	# Infos periodiques
	#    
	if ((loopCtr % 61) ==0):
		if tgl:
			InfoSys()
		else:
			GetIPaddress()
		
	#--------------------------------------------------------------------
	# Fin de boucle
	#    
	if glbSortir:
		print("> Sortie demandée")

	print("> Main: ["+l1+"] ["+l2+"]")
	time.sleep(0.2)
			
  
#=======================================================================
#                                   Fin
#=======================================================================

lcd.setRGB(200,10,10)
print("> Main: Exit")

# Sauvegarde du reliquat de log si activé
#   
if logToggle:
	with open(nmeaPath, "w") as nmeaHdlr:
		for item in TramesNMEA:
			nmeaHdlr.write("\n"+item)
	nmeaHdlr.close()

l1="*** exit **"
l2="Parsing logs..."
lcd.setRGB(0,0,250)
lcd.clear()
lcd.setCursor(0,0)
lcd.printout(l1)
lcd.setCursor(0,1)
lcd.printout(l2)
print("> Main: ["+l1+"] ["+l2+"]")
time.sleep(0.2)

#gpx_lib.init()

lcd.clear()
lcd.setRGB(40,40,40)
			
