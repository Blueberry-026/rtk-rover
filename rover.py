# -*- coding: utf-8 -*-
#
#
# Parseur de logs NMEA
#
# References  :
#       https://gpsd.gitlab.io/gpsd/NMEA.html
#       https://anavs.com/knowledgebase/nmea-format
#
# Types de trames presents dans le log LABO :
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
# | HISTORIQUE - Script de decodage de trames NMEA
# |---------------------------------------------------------------------
# | Changelog : [*] Bug fixes
# |             [+] New function
# |             [-] Update
# |-------+------------+------------------------------------------------
# | VERS  | DATE       | EVOLUTIONS
# |-------+------------+------------------------------------------------
# |       |            |
# | v0.2x | 04/01/2024 | + Recuperation de la table centipede
# |       |            | + Choix auto de la base la plus proche
# |       |            | + Thread de lecture TCP de la base
# |       |            | + Fichier de dump du bianire decu de la base
# |       |            | + Ajout d'un fichier INI pour parametrage
# |       |            | * Modification du time-out (durée max de fonstionnement)
# |       |            |
# | v0.1x | 02/01/2024 | + Ouvre le port serie a la vollee si antenne branchee
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
# git add rover.py
# pip install haversine --break-system-packages

import sys
sys.path.append('../')
import rgb1602
import time
import RPi.GPIO as GPIO
import serial
import subprocess
import psutil
import gpx_lib
import socket
import binascii
import random
import configparser
import threading
import os
import haversine as hs

from math import sqrt
from datetime import datetime
from os.path  import exists
from gpiozero import CPUTemperature
#from time import time

_version = "0.21"

posFix=[ "NOFX","GPS ", "DGPS", "N/A ", "RTKx", "RTKf", "INS "]
trc_IN  =0
trc_MSG =1
trc_OUT =2
trc_HDL =0

lcd=rgb1602.RGB1602(16,2)   

glbLat    = 0.0
glbLon    = 0.0
glbFix    = 1
glbNSat   = 0
glbHeight = 0.0
glbHDop   = 0.0
glbFreeze	= 0
glbSortir	= False
glbTxGGA	= False
glbTramesNMEA = 0
serOpen		= False
tgl			= False
logToggle	= True
glbLstBases = ""

defLatitude		=0.0
defLongitude	=0.0

glbWifiAd = "0.0.0.0"
glbEthAd  = "0.0.0.0"

glbRootPath="/home/blueb/RoverRTK"
lcdFreeze=10
glbOffset=0
dmpSize=10

TramesNMEA=[]
glbLastGGA=""
glbDureeMax = 0
Constellation = {	"GP": "GPS only",
                        "GL": "GLONASS",
                        "GA": "GALILEO",
                        "GN": "multi GNSS"	}
imbric     = 1
glbTraceMode   = True

btnSELECT	= 16
btnUP		= 17
btnDOWN		= 18
btnLEFT		= 19
btnRIGHT	= 20
glbRcv		= ""

cstIP		= ''
cstPort		= 0
cstUser		= ''
cstPwd		= ''

basePoint	= ''
baseIndex	= 0
baseDist	= 0.0
baseUplGGA	= False

baseRecord =[	# STR;ASSAS;Ruan;RTCM3;1004,1005,1006;2;GLO+GAL+SBS+BDS+GPS;NONE;FRA;48.096;1.894;0;0;NTRIP RTKBase U-blox_ZED-F9P 2.4.2 1.13;none;N;N;15200;CentipedeRTK
    "typ",		# STR (the only acceptable string)
    "mpt",		# Caster mountpoint
    "iden",		# Source identifier, e.g. name of city next to source location
    "fmt",		# Data format RTCM, RAW, etc.
    "dts",		# format-details
    "car",		# Data stream contains carrier phase information
                #	0 = No (e.g. for DGPS)
                #	1 = Yes, L1 (e.g. for RTK)
                #	2 = Yes, L1&L2 (e.g. for RTK)
    "nav",		# Navigation system(s)
                #	GPS
                #	GPS+GLONASS
                #	GPS+EGNOS
    "netw",		# Network
                # 	EUREF
                # 	IGS
                # 	IGLOS
                # 	SAPOS
                # 	GREF
                # 	Misc
    "ctry",		# Country. Three character country code in ISO 3 Characters 3166
    "lat",		# Latitude. Floating point number, two digits after decimal point
    "lon",		# Longitude. Floating point number, two digits after decimal point
    "nmea",		# Necessity for Client to send NMEA message with approximate position to Caster
                #	0 = Client must not send NMEA message with approximate position to Caster
                #	1 = Client must send NMEA GGA message with approximate position to Caster
    "sol",		# Stream generated from single reference station or from networked reference stations
                #	0 = Single base
                #	1 = Network
    "gen",		# Generator. Hard or software generating data stream
    "comp",		# Compression/Encryption algorithm applied. Characters, undefined length
    "aut",		# Authentication. Access protection for this particular data stream
                #	N = None
                #	B = Basic
                #	D = Digest
    "fee",		# User fee for receiving this particular data stream
                #	N = No user fee
                #	Y = Usage is charged
    "bitr",		# Bit rate of data stream, bits per second
    "misc" 		# Miscellaneous information, last data field in record
]
     
ggaRecord = [ 	# $GNGGA,122220.00,4542.4812848,N,00451.2467166,E,2,12, 0.60,188.552, M,47.358,M,,0136*4A
    "id",		#  0 Message ID $GPGGA
    "time",		#  1 UTC of position fix
    "lat",		#  2 Latitude
    "dlat",		#  3 Direction of latitude:
                #		N: North
                #		S: South
    "lon",		#  4 Longitude
    "dlon",		#  5 Direction of longitude:
                #     E: East
                #     W: West
    "qlty",		#  6 GPS Quality indicator:
                #     0: Fix not valid
                #     1: GPS fix
                #     2: Differential GPS fix (DGNSS), SBAS, OmniSTAR VBS, Beacon, RTX in GVBS mode
                #     3: Not applicable
                #     4: RTK Fixed, xFill
                #     5: RTK Float, OmniSTAR XP/HP, Location RTK, RTX
                #     6: INS Dead reckoning
    "nsat",		#  7 Number of SVs in use, range from 00 through to 24+
    "hdop",		#  8 HDOP level precision factor
    "hgth",		#  9 Orthometric height (MSL reference)
    "hmea",		# 10 M: unit of measure for orthometric height is meters
    "geoid",	# 11 Geoid separation
    "gmea",		# 12 M: geoid separation measured in meters
    "diff",		# 13 Age of differential GPS data record, Type 1 or Type 9. 
                #		Null field when DGPS is not used.
    "ref", 		# 14 Reference station ID, range 0000 to 4095. A null field  
                #		when any reference station ID is selected and no 
                #		corrections are received. See table below for a 
                #		description of the field values.
    "cks"		# 15 The checksum data, always begins with *
]
#=======================================================================
# CustomChar
#-----------------------------------------------------------------------
#
# 
def CustomChar():
    Trace(trc_IN, sys._getframe().f_code.co_name)
    cHaut = [	0b00011,
                0b00011,
                0b00011,
                0b00011,
                0b00000,
                0b00000,
                0b00000,
                0b00000 ]
    cBas = [  	0b00000,
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
    lcd.customSymbol(0, cHaut) 
    lcd.customSymbol(1, cBas)   
    lcd.customSymbol(2, cCroix)
    Trace(trc_OUT, sys._getframe().f_code.co_name)

# ==================================================================================================================
# readINI
# ------------------------------------------------------------------------------------------------------------------
#
def readINI():
    global cstIP,cstPort,cstUser,cstPwd,cstPoint
    global dmpSize,lcdFreeze,defLatitude,defLongitude,glbOffset,glbDureeMax,glbTramesNMEA
    
    Trace(trc_IN, sys._getframe().f_code.co_name)

    cstIP    	= 'caster.centipede.fr'
    cstPort  	= 2101
    cstUser  	= 'centipede'
    cstPwd   	= 'centipede'
    cstPoint 	= 'COMBE'
    glbOffset	= 0
    defLongitude= 4.500
    defLatitude	= 45.446
    lcdFreeze	= 10
    dmpSize	= 100
    glbTramesNMEA = 5000
    glbDureeMax = 5
    glbRootPath = '/home/blue/RoverRTK'

    try:
        config = configparser.ConfigParser()
        config.read(glbRootPath+'/rover.ini')

        cstUser      = config['Centipede']['user']
        cstPwd       = config['Centipede']['password']
        cstPoint     = config['Centipede']['base']
        cstIP        = config['Centipede']['serveur']
        cstPort      = int(config['Centipede']['port'])
        lcdFreeze    = int(config['Ecran']['freeze'])
        glbDureeMax  = int(config['General']['dureemax'])
        dmpSize      = int(config['Traces']['dumpsize'])
        glbTramesNMEA= int(config['Traces']['tramesNMEA'])
        glbRootPath  = config['General']['rootpath']
        defLatitude  = float(config['Position']['latitude'])
        defLongitude = float(config['Position']['longitude'])

        if (config['General']['offset'] == "1"):
            glbOffset=random.random()	
        else:
            glbOffset=0
    except:
       Trace(trc_MSG,"***except****")
    Trace(trc_OUT, sys._getframe().f_code.co_name)

# ==================================================================================================================
# Distance
# ------------------------------------------------------------------------------------------------------------------
#
def Distance(pt1, pt2):
    Trace(trc_IN, sys._getframe().f_code.co_name)
    dst = hs.haversine(pt1,pt2,unit='km')
    Trace(trc_OUT, sys._getframe().f_code.co_name)
    return (dst)

# ==================================================================================================================
# Trace
# ------------------------------------------------------------------------------------------------------------------
#
def Trace(xsens, fct):
    global imbric, glbTraceMode,trc_HDL
    s=""

    if (trc_HDL == 0):
        trc_HDL = open (glbRootPath+'/debug.jrn', 'w')
        
    if xsens==trc_IN:
        for x in range(0, imbric):
            s = s + "   "
        log = str(imbric) + s + "> " + fct
        imbric=imbric+1
    elif xsens==trc_OUT:
        imbric=imbric-1
        for x in range(0, imbric):
            s = s + "   "
        log = str(imbric) + s + "< " + fct
    else:
        for x in range(0, imbric):
            s = s + "   "
        log=str(imbric) + s + "     * " + fct

    if glbTraceMode:
        print(log)
        trc_HDL.write("\n"+log)
                                      
#=======================================================================
# Init_LCD
#-----------------------------------------------------------------------
#
# 
def Init_LCD():
    Trace(trc_IN, sys._getframe().f_code.co_name)
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
    Trace(trc_OUT, sys._getframe().f_code.co_name)
    
#=======================================================================
# read_buttons
#-----------------------------------------------------------------------
# Read the key value
# 
def read_LCD_buttons():
    Trace(trc_IN, sys._getframe().f_code.co_name)

    ret=0
    if (GPIO.input(btnSELECT) == 1):
        Trace(trc_MSG,"ReadBTN:btnSELECT\n")
        ret = btnSELECT
    if (GPIO.input(btnUP) == 1):
        Trace(trc_MSG,"ReadBTN:btnUP")
        ret =  btnUP
    if (GPIO.input(btnDOWN) == 1):
        Trace(trc_MSG,"ReadBTN:btnDOWN")
        ret =  btnDOWN
    if (GPIO.input(btnLEFT) == 1):
        Trace(trc_MSG,"ReadBTN:btnLEFT")
        ret =  btnLEFT
    if (GPIO.input(btnRIGHT) == 1):
        Trace(trc_MSG,"ReadBTN:btnRIGHT")
        ret =  btnRIGHT

    Trace(trc_OUT, sys._getframe().f_code.co_name)   		
    return ret

#=======================================================================
# DecodeGGA 
#-----------------------------------------------------------------------
# $GNGGA,122220.00,4542.4812848,N,00451.2467166,E,2,12,0.60,188.552,M,47.358,M,,0136*4A
# Field	Meaning
# 0	Message ID $GPGGA
# 1	UTC of position fix
# 2	Latitude
# 3	Direction of latitude:
#     N: North
#     S: South
# 

def DecodeGGA(trame):
    global glbLat,glbLon,glbFix,glbNSat,glbHeight,glbHDop,glbLastGGA

    Trace(trc_IN, sys._getframe().f_code.co_name)
    Trace(trc_MSG, Constellation[trame[1:3]])
#   print (trame)
    glbLastGGA = trame
    try:
        fields =trame.split(",")
        fields = dict(zip(ggaRecord, fields))
        lcLat    = float(fields["lat"])/100
        lcLon    = float(fields["lon"])/100
        lcFix    = int(fields["qlty"])
        lcNSat   = int(fields["nsat"])
        lcHeight = int(float(fields["hgth"]))
        lcHDop   = float(fields["hdop"])
        if ((lcLat < 50) & (lcLat > 40) & (lcLon>0) & (lcLon<6)):
            glbLat   = lcLat + glbOffset   
            glbLon   = lcLon + glbOffset   
            glbFix   = lcFix   
            glbNSat  = lcNSat  
            glbHeight= lcHeight 
            glbHDop  = lcHDop  
            
        if (glbFix == 2):
            lcd.setRGB(0,0,150)
        elif (glbFix >= 4):
            lcd.setRGB(0,150,0)
            Trace(trc_MSG, "*** RTK **")
        else:
            lcd.setRGB(120,120,120)
        
    except:
        lcd.setRGB(120,120,120)
        #glbLat    = 0.0
        #glbLon    = 0.0
        #glbFix    = 0
        #glbNSat   = 0
        #glbHeight = 0
        #glbHDop   = 0
        pass
    Trace(trc_OUT, sys._getframe().f_code.co_name)
    
#=======================================================================
# GetIPaddress 
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
    
    Trace(trc_IN, sys._getframe().f_code.co_name)
    lcd.clear()	

    ad=psutil.net_if_addrs()
    glbWifiAd = ad["wlan0"][0].address
    glbEthAd  = ad["eth0"][0].address

    l1 = "W:"+glbWifiAd
    l2 = "E:"+glbEthAd
    lcd.setCursor(0, 0)
    lcd.printout(l1)      
    lcd.setCursor(0, 1)
    lcd.printout(l2)
    lcd.write(0)

    glbFreeze=lcdFreeze
    Trace(trc_MSG, "> GetIPaddress: ["+l1+"] ["+l2+"]")
    Trace(trc_OUT, sys._getframe().f_code.co_name)
    
#=======================================================================
# InfoSys 
#-----------------------------------------------------------------------
#
def InfoSys():
    global glbFreeze
    
    Trace(trc_IN, sys._getframe().f_code.co_name)
    lcd.clear()
    
    cpu = CPUTemperature()
    cpufreq = psutil.cpu_freq()
    svmem = psutil.virtual_memory()

    l1=format("CPU:%2.0f%% MEM:%2.0f%%" % (psutil.cpu_percent(),svmem.percent))
    l2=format("T:%2.0fc F:%dHz"   % (cpu.temperature,cpufreq.current))
    
    lcd.setCursor(0, 0)
    lcd.printout(l1)      
    lcd.setCursor(0, 1)
    lcd.printout(l2)	
    lcd.write(0)

    glbFreeze=lcdFreeze
    Trace(trc_MSG, "["+l1+"] ["+l2+"]")
    Trace(trc_OUT, sys._getframe().f_code.co_name)
    
#=======================================================================
# SearchUSBport 
#-----------------------------------------------------------------------
#
def SearchUSBport():
    Trace(trc_IN, sys._getframe().f_code.co_name)
    usbPort=""
    for port in range (0,10):
        ttyPort="/dev/ttyUSB"+str(port)
        if exists(ttyPort):
            usbPort=ttyPort
            Trace(trc_MSG, "> SearchUSBport: Port serie="+usbPort)
            break
    return(usbPort)
    Trace(trc_OUT, sys._getframe().f_code.co_name)

#=======================================================================
# rtkGetNearestBase
#-----------------------------------------------------------------------
#
def rtkGetNearestBase(name):
    global basePoint, baseDist, baseUplGGA,baseIndex, glbLstBases
    
    Trace(trc_IN, sys._getframe().f_code.co_name)
    nearest = 500
    
    i=0
    for i in range (1,len(glbLstBases)):
        fld=glbLstBases[i].split(';')
        fields = dict(zip(baseRecord, fld))
        #print(fields)
        if fld[0] == "STR":
            #print("%s----%s" % (fields["lat"],fields["lon"]))
            dst=Distance((defLatitude,defLongitude), (float(fields["lat"]),float(fields["lon"])))
            if dst<nearest:
                baseNdx=i
                basePoint=fields["mpt"]
                baseDist=dst
                jrn=format("I=%d Base=%s Lat=%f Lon=%f Dst=%f" % (i,fields["mpt"],float(fields["lat"]),float(fields["lon"]),dst))
                Trace(trc_MSG, jrn)
                nearest=dst
                if (fields["nmea"] == "1"):
                    baseUplGGA=True
                else:
                    baseUplGGA=True
    Trace(trc_OUT, sys._getframe().f_code.co_name)

#=======================================================================
# rtkBaseConnect [Thread]
#-----------------------------------------------------------------------
#
def rtkBaseConnect(name):
    Trace(trc_IN, sys._getframe().f_code.co_name)
    BaseConnect_HDR ="ICY 200 OK"
    BaseConnect_REQ = 'GET /' + basePoint + ' HTTP/1.0 \n' + \
        'Host: '+cstUser + ':' + cstPwd+'\n' + \
        'User-Agent: NTRIP Rover' + "\n" + \
        'Accept: ' + '\n' + \
        'Authorization: Basic '+cstUser + ':' + cstPwd+'\n\n'

    # Init connexion to base
    #
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)
    server_address = (cstIP, cstPort)
    sock.settimeout(None)
    sock.connect(server_address)

    # Send base connect request
    #    
    req = bytes(BaseConnect_REQ, encoding='utf-8')
    jrn=format('connect to [%s] port [%s] req [%s]' % (cstIP, cstPort,req) )
    Trace(trc_MSG, jrn)
    sock.sendall(req)
    
    # Continously read data sent by base
    #
    file = open (glbRootPath+'/base-dump.dmp', 'w')
    rcvCounter = 0
    
    ggaTrame  = "GNGGA,150412.00,4544.6231923,N,00450.0667313,E,2,12,0.62,188.042,M,47.358,M,,0136*43"
    ggaHeader = "Ntrip-GGA: {}\r\n".format(ggaTrame)
    data = sock.recv(len(BaseConnect_HDR))
    
    if (data.decode() == BaseConnect_HDR):
        Trace(trc_MSG, "Receiving base data  ok")
        while(not glbSortir):
            glbRcv=sock.recv(1024)
            if (len(glbRcv) >0):
                rcvCounter = rcvCounter + len(glbRcv)
                #print(binascii.hexlify(glbRcv))
                dmp = str(binascii.hexlify(glbRcv))

                if (baseUplGGA):
                    sock.send(ggaTrame.encode('ascii'))		# Utile ?

                # We limit dump file to 100Ko
                #
                if (rcvCounter < (dmpSize*1024)):
                    file.write(dmp)
                print(".",end='')
            else:
                print("*",end='')
    file.close()
    Trace(trc_OUT, sys._getframe().f_code.co_name)
                   
#=======================================================================
# rtkGetBaseList
#-----------------------------------------------------------------------
#
def rtkGetBaseList(name):
    global glbLstBases
    
    Trace(trc_IN, sys._getframe().f_code.co_name)
    BaseListe_HDR ="SOURCETABLE 200 OK"
    BaseListe_REQ = 'GET / HTTP/1.0 \n' + \
        'Host: ' + cstUser + ':' + cstPwd + '\n' + \
        'User-Agent: NTRIP Rover' + "\n" + \
        'Accept: /' + "\n" + \
        'Authorization: Basic ' + cstUser + ':' + cstPwd +  "\n\n"
    
    # Init connexion to base
    #
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)
    server_address = (cstIP, cstPort)
    sock.settimeout(None)
    sock.connect(server_address)
    
    # Send base list request
    #
    req = bytes(BaseListe_REQ, encoding='utf-8')
    jrn=format('connect to [%s] port [%s] req [%s]' % (cstIP, cstPort,req))
    Trace(trc_MSG,jrn)
    sock.sendall(req)
    
    # Read table (~100 000 bytes)
    #
    rcvCounter = 0
    data = sock.recv(len(BaseListe_HDR))
    if (data.decode() == BaseListe_HDR):
        Trace(trc_MSG, "Receiving caster table ok")
        while True:
            data = sock.recv(1024)
            rcvCounter = rcvCounter + len(data)
            #print ("< " + data.decode())
            glbLstBases=glbLstBases+data.decode()
            if len(data)==0:
                break
    else:
        Trace(trc_MSG, ">>ERROR receiving caster table")
    jrn=format('table received [%d] bytes / [%d] bases' % (rcvCounter,len(glbLstBases)))
    Trace(trc_MSG, jrn)
    # Dump table
    #
    file = open (glbRootPath+'/caster-table.txt', 'w')
    file.write(glbLstBases)
    file.close()
    glbLstBases=glbLstBases.split("\n")
    sock.close()
    Trace(trc_OUT, sys._getframe().f_code.co_name)

#=======================================================================
# MAIN
#-----------------------------------------------------------------------
#
Trace(trc_IN, sys._getframe().f_code.co_name)
jrn=datetime.now()
Trace(trc_MSG, str(jrn))
readINI()
Init_LCD()
lcd.clear()
CustomChar()

l1="Rasperry - v"+_version
l2="Parsing logs..."
lcd.setRGB(120,120,120)
lcd.clear()
lcd.setCursor(0,0)
lcd.printout(l1)
lcd.setCursor(0,1)
lcd.printout(l2)
jrn=format("> Main: [%s] [%s]" % (l1,l2))
Trace(trc_MSG, jrn)

Trace(trc_MSG, os.getcwd())
rtkGetBaseList("toto")

rtkGetNearestBase("toto")
jrn=format("Point [%s] Index [%d] Dist [%f]  GGA [%d]" % (basePoint,baseIndex,baseDist,1))
Trace(trc_MSG, jrn)
threadBase = threading.Thread(target=rtkBaseConnect, args=(1,))
threadBase.start()
    
time.sleep(5)

dt=datetime.now().strftime("%Y%m%d_%H%M%S")
nmeaPath=glbRootPath+"/nmea/nmea-"+dt+"(init).nmea"
loopCtr =0
nmeaData = ""

tmDeb=time.time()
while (not glbSortir):  #Tourne 10h
    loopCtr=loopCtr+1
    if ((loopCtr % 5) == 0):
        if tgl:
            tgl=False
        else:
            tgl=True

    #--------------------------------------------------------------------
    # Detection et ouverture du port serie si non fait
    #
    if not serOpen:
        try:
            serPath=SearchUSBport()
            if (len(serPath)>0):
                Trace(trc_MSG, "Essai ouverture ["+serPath+"]")
                serPort=serial.Serial(	port=serPath,
                                                baudrate=115200,
                                                parity=serial.PARITY_NONE,
                                                stopbits=serial.STOPBITS_ONE,
                                                bytesize=serial.EIGHTBITS,
                                                timeout=0.1)
                serOpen=True
                lcd.setRGB(0,255,0)
                serPort.flush()
                Trace(trc_MSG, "serOpenSUCCESS")
            else:
                l1="*** ERROR ***"
                l2="No RTK receiver"
                lcd.clear()
                lcd.setCursor(0, 0)
                lcd.printout(l1)
                lcd.setCursor(0, 1)
                lcd.printout(l2)
                Trace(trc_MSG, "> Main: ["+l1+"] ["+l2+"]")
        except:
            Trace(trc_MSG, "serOpen FAILED")
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
                Trace(trc_MSG, "> Main: ["+l1+"] ["+l2+"]")
                serOpen=False
                break
        if len(nmeaData)>1:
            #print("      > " + nmeaData)
            dt=datetime.now().strftime("%H:%M:%S:%f")
            TramesNMEA.append(dt + ";" + nmeaData)
        else:
            break
    if (len(glbRcv)>0):
        serPort.write(glbRcv)
        Trace(trc_MSG, "glbRcv")
        glbRcv=[]
        
    #--------------------------------------------------------------------
    # Detection si bouton presse
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
            Trace(trc_MSG, "Recording OFF")    
            logToggle=False
            lcd.printout("Recording OFF")      
        else:
            Trace(trc_MSG, "Recording ON")    
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
    # Symbole d'activite clignotant, different si enregisrement actif ou pas
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
            Trace(trc_MSG, "["+l1+"] ["+l2+"]")
            lcd.write(0)
    else:
        glbFreeze=glbFreeze-1
    #--------------------------------------------------------------------
    # Si tableau de trames plein, ecrire dans un fichier date
    #    
    if len(TramesNMEA)>glbTramesNMEA:
        Trace(trc_MSG, "Purge liste")
        if logToggle:
            with open(nmeaPath, "w") as nmeaHdlr:
                for item in TramesNMEA:
                    nmeaHdlr.write("\n"+item)
            nmeaHdlr.close()
            dt=datetime.now().strftime("%Y%m%d_%H%M%S")
            nmeaPath=glbRootPath+"/nmea/nmea-"+dt+".nmea"
        TramesNMEA=[]

    #--------------------------------------------------------------------
    # Infos periodiques
    #    
    if ((loopCtr % 61) ==0):
        tmFin=time.time()
        if(int((tmFin-tmDeb) / 60) > glbDureeMax):
            Trace(trc_MSG, "Duré de fonctionnement max atteinte ("+str(glbDureeMax)+" mn)")
            glbSortir=True        
        
        if tgl:
            InfoSys()
        else:
            GetIPaddress()
        
    #--------------------------------------------------------------------
    # Fin de boucle
    #    
    if glbSortir:
        Trace(trc_MSG, " Sortie demandee")

    #print("> Main: ["+l1+"] ["+l2+"]")
    time.sleep(0.1)
            
  
#=======================================================================
#                                   Fin
#=======================================================================

lcd.setRGB(200,10,10)
Trace(trc_MSG, "Exit")

# Sauvegarde du reliquat de log si active
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
            
Trace(trc_OUT, sys._getframe().f_code.co_name)
trc_HDL.close()
