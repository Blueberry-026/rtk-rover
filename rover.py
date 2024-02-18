# -*- coding: utf-8 -*-

_version    = "0.69a"
# /============================================================================
# | HISTORIQUE - Script de pilotage rover RTK
# |----------------------------------------------------------------------------
# | References  :
# |       https://gpsd.gitlab.io/gpsd/NMEA.html
# |       https://anavs.com/knowledgebase/nmea-format
# | 
# | Types de trames presents dans le log GNSS :
# |                   DAT TIM LAT LON ALT VIT
# |       $GxTXT       -   -   -   -   -   -
# |       $GxRMC       x   x   x   x   -   x
# |       $GxVTG       -   -   -   -   -   x
# |       $GxGGA       -   x   x   x   x   -
# |       $GxGSA       -   -   -   -   -   -
# |       $GxGSV       -   -   -   -   -   -
# |       $GxGLL       -   x   x   x   -   -
# |-------+------------+-------------------------------------------------------
# | Changelog : [*] Bug fixes
# |             [+] New function
# |             [-] Update
# |-------+------------+-------------------------------------------------------
# | VERS  | DATE       | EVOLUTIONS
# |-------+------------+-------------------------------------------------------
# |       |            |
# | v0.6x | 02/02/2024 | * Probleme de thread pour la RS
# |       |            | - Changement de la genration des logs (utilisation lib)
# |       |            | + Decodage RTCM en fin de script et generation fichier
# |       |            | + Stat NMEA sur les fix GPS
# |       |            | + Enregistrement NMEA que si trame recues non vides
# |       |            | + 
# |       |            |
# | v0.5x | 26/01/2024 | + Gestion du fichier de conf des reseaux wifi
# |       |            | + Cryptage des clés/password
# |       |            | * Bug de remplissage NMEA (globale)
# |       |            | - Trace in/out homogenisé
# |       |            | - Ajout heure ds journal et NMEA
# |       |            | - Plus de test de validité sur NMEA recu
# |       |            |
# | v0.4x | 19/01/2024 | + Export auto des données sur clé USB au deb/fin et touche + laneur
# |       |            | + Sauvegarde des 10 derniers journaux 
# |       |            | - Filtrage des ecritures GPX et CSV si lat/lon=0
# |       |            | - Suppression cyte non-ascii recues sur USB pr eviter exception
# |       |            | + Receptions USB deportées dans une thread spécifique
# |       |            | - Comptages des satellites amméliorés (lectures GSA)
# |       |            | - Ecran constellations ajouté
# |       |            | - Recherche un '$' à l'interieur d'une trame NMEA  corrompue
# |       |            |
# | v0.3x | 15/01/2024 | * Correction de la réémision du flux serie vers F9P
# |       |            | - Changement des affichages LCD
# |       |            | + Conversion auto des NMEA en GPX en fin de boucle
# |       |            | + Ammélioration du traitement des exceptions/journal de traces
# |       |            | * Lecture de 2 boutons simultanés (pour shutdown)
# |       |            | * Refonte de la conversion GPX
# |       |            | + Ajout de la verif checksum des trame
# |       |            | - Meilleure gestion du reseau - detection du Wifi
# |       |            | - Ajout d'infos LCD
# |       |            |
# | v0.2x | 04/01/2024 | + Recuperation de la table centipede
# |       |            | + Choix auto de la base la plus proche
# |       |            | + Thread de lecture TCP de la base
# |       |            | + Fichier de dump du bianire decu de la base
# |       |            | + Ajout d'un fichier INI pour parametrage
# |       |            | * Modification du time-out (durée max de fonstionnement)
# |       |            | - Ammélioration des traces (ecran + fichier)
# |       |            | * Correction du bug d'écriture port serie
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
# \----------------------------------------------------------------------------
# git gui &
# git add rover.py
# git diff main~5:rover.py rover.py
# git commit -am"add diff test file"
# git log --pretty=oneline
# git log -p -1 rover.py
# git log
# pip install haversine --break-system-packages
# sudo apt-get install realvnc-vnc-server
# sudo systemctl restart vncserver-x11-serviced
import sys

sys.path.append("../")
sys.path.append("lib")
import rgb1602
import time
import RPi.GPIO as GPIO
import serial
import psutil
import socket
import logging
import random
import configparser
import threading
import shutil
import os
import haversine as hs
import glob
import subprocess
import base64
from datetime import datetime
from os.path  import exists
from gpiozero import CPUTemperature
from nmea_format import *
from rtcm_format import *
from centipede_format import *

posFix          = ["NOFX", "GPS ", "DGPS", "N/A ", "RTKx", "RTKf", "INS "]
statFix         = [0,0,0,0,0,0,0]
trc_IN          = 0
trc_MSG         = 1
trc_OUT         = 2
trc_HDL         = 0

lcd = rgb1602.RGB1602(16, 2)

glbLat          = 0.0
glbLon          = 0.0
glbFix          = 1
glbNSat         = 0
glbHeight       = 0.0
glbHDop         = 0.0
glbFreeze       = 0
 
glbMainExit     = False
glbTcpExit      = False
                
serOpen         = False
tglScreen       = False
tglLive         = False
logToggle       = True
glbLstBases     = ""
ggaTrame        = ""
                
glbWifiAd       = "0.0.0.0"
glbEthAd        = "0.0.0.0"

glbRecord       = True                
iniRootPath     = "/home/blueb/RoverRTK"
iniLcdFreeze    = 10
iniDmpSize      = 10
glbSilent       = False                
TramesNMEA      = []
glbLastGGA      = ""
iniDureeMax     = 0
gnssType        = { "GP": "GPS only",
                    "GL": "GLONASS",
                    "GA": "GALILEO",
                    "GN": "multi GNSS" }
imbric          = 0
glbTraceMode    = True

#---------------------- Gestion des read/write USB et TCP
glbTcpRxCounter = 0
glbTcpTxCounter = 0
glbUsbRxCounter = 0
glbUsbTxCounter = 0

glbTcpBuffer    = []
glbTcpRtcm      = []
glbTxGGA        = False
iniSzMaxNMEA    = 0

#---------------------- Variable issues de l'INI
iniWifiSSID     = ""
iniWifiPwd      = ""
iniWifiIndex    = 0
iniLogging      = 00

iniCstIP        = ""
iniCstPort      = 0
iniCstUser      = ""
iniCstPwd       = ""
iniCstPoint     = ""

basePoint       = ""
baseIndex       = 0
baseDist        = 0.0
baseUplGGA      = False

iniDefLat       = 0.0
iniDefLon       = 0.0

LOG_NST         = 0
LOG_DBG         = 1
LOG_INF         = 2
LOG_WRN         = 3
LOG_ERR         = 4
LOG_CRT         = 5

#----------------------
iniOffset       = 0
glbData         = []
glbRtcmMsg      = []

#---------------------- Boutons du LCD
btnSELECT       = 0x01
btnUP           = 0x02
btnDOWN         = 0x04
btnLEFT         = 0x10
btnRIGHT        = 0x20
              
glbSatCtr       = [ 0, #                   0 = QZSS	
                    0, #                   1 = GPS
                    0, #                   2 = GLONASS
                    0, #                   3 = Galileo
                    0, #                   4 = BeiDou
                    0  #                   5 = ???
                   ]
lastDate="x.y.z"
lastSpeed=0.0

glbErrInfos = { "ntw" : False,
                "xxx" : False,
                "yyy" : False
            }

#==============================================================================
# DecimalDegrees
#------------------------------------------------------------------------------
# => Format NMEA recu :
#       dd.mmmmmm
#       dd           degres
#          mm.mmmm   minutes decimales
# => Format de sortie : degres decimaux
#       dd.dddddd
#
def DecimalDegrees(strPos):
    if (strPos != ""):
        pos = float(strPos) / 100
        dg = int(pos)
        dec = (pos % 1) * 100
        mn = (dec * 100) / 60
        dm = dg + (mn / 100)
        if (dm < 4):
            print("\n   -> Position [%f] incorrecte ***" % pos)
    else:
        dm = 0.0
    return dm

#==============================================================================
# Distance
#------------------------------------------------------------------------------
#
def Distance(pt1, pt2):
    Trace(trc_IN, LOG_DBG, sys._getframe().f_code.co_name)
    dst = hs.haversine(pt1, pt2, unit="km")
    Trace(trc_OUT, LOG_DBG, sys._getframe().f_code.co_name)
    return (dst)

#==============================================================================
# Trace
#------------------------------------------------------------------------------
#    xsens, argv[0]
# severity, argv[1]
#      fct, argv[2]
#      txt, argv[3] (optionnel)
# 
#  0  logging.NOTSET
# 10  logging.DEBUG
# 20  logging.INFO
# 30  logging.WARNING  
# 40  logging.ERROR
# 50  logging.CRITICAL
#
def Trace(*argv):
    global imbric
    s = ""

    xsens    = argv[0]
    severity = argv[1]
    fct      = argv[2]
    
    if (fct=="<module>"):
        fct=""
        
    if (len(argv) == 4):
        txt  = argv[3]
    else:
        txt  = ""
        
    if xsens == trc_IN:
        for x in range(0, imbric):
            s = s + chr(9)
        log = s + "> " + fct + " " + txt
        imbric = imbric + 1
    elif xsens == trc_OUT:
        imbric = imbric - 1
        for x in range(0, imbric):
            s = s + chr(9)
        log = s + "< " + fct + " " + txt
    else:
        for x in range(0, imbric):
            s = s + chr(9)
        log = s + " * " + fct + " " + txt
    
    if severity == LOG_DBG:
        logging.debug(   format("%s " % (log)))
    elif severity == LOG_INF:
        logging.info(    format("%s " % (log)))
    elif severity == LOG_WRN:
        logging.warning( format("%s " % (log)))
    elif severity == LOG_ERR:
        logging.error(   format("%s " % (log)))
    else:
        logging.critical(format("%s " % (log)))

    if (severity>=LOG_INF):
        print(log)
        
#==============================================================================
# crypt_Encode
#------------------------------------------------------------------------------
#
def crypt_Encode(key, clear):
    Trace(trc_IN, LOG_DBG, sys._getframe().f_code.co_name, f"key={key} clear={clear}")
    enc = []
    for i in range(len(clear)):
        key_c = key[i % len(key)]
        enc_c = chr((ord(clear[i]) + ord(key_c)) % 256)
        enc.append(enc_c)
    enc=base64.urlsafe_b64encode("".join(enc).encode()).decode()
    Trace(trc_OUT, LOG_DBG, sys._getframe().f_code.co_name, f"key={key} enc={enc}")
    return (enc)

#==============================================================================
# crypt_Decode
#------------------------------------------------------------------------------
#
def crypt_Decode(key, enc):
    Trace(trc_IN, LOG_DBG, sys._getframe().f_code.co_name, f"key={key} enc={enc}")
    dec="abc"
    if (key[:1] == "_"):
        dec=key[1:len(key)]
    else:
        dec = []
        enc = base64.urlsafe_b64decode(enc).decode()
        for i in range(len(enc)):
            key_c = key[i % len(key)]
            dec_c = chr((256 + ord(enc[i]) - ord(key_c)) % 256)
            dec.append(dec_c)
        dec="".join(dec)
    Trace(trc_OUT, LOG_DBG, sys._getframe().f_code.co_name, f"key={key} dec={dec}")
    return (dec)

#==============================================================================
# lcd_CustomChar
#------------------------------------------------------------------------------
#
def lcd_CustomChar():
    Trace(trc_IN, LOG_DBG, sys._getframe().f_code.co_name)
    cHaut =   [ 0b00011,
                0b00011,
                0b00011,
                0b00011,
                0b00000,
                0b00000,
                0b00000,
                0b00000]
    cBas =    [ 0b00000,
                0b00000,
                0b00000,
                0b00000,
                0b00011,
                0b00011,
                0b00011,
                0b00011]
    cRecOff=  [ 0b00111,
                0b00101,
                0b00101,
                0b00101,
                0b00101,
                0b00101,
                0b00101,
                0b00111]
    
    # create a new character
    #
    lcd.customSymbol(0, cHaut)
    lcd.customSymbol(1, cBas)
    lcd.customSymbol(2, cRecOff)
    Trace(trc_OUT, LOG_DBG, sys._getframe().f_code.co_name)

#==============================================================================
# lcd_SetColor
#------------------------------------------------------------------------------
#
def lcd_SetColor(status):
    global glbHDop
    try:
        if (glbErrInfos["ntw"]):
            lcd.setRGB(250,  0,  0)      # rouge
        else:
            #     Bon DOP: 0.6 -> CL=40     -> sombre
            #   Moyen DOP: 1.0 -> CL=152
            # Mauvais DOP: 2.0 -> CL=332    -> clair
            #
            cl = 40 + int((glbHDop-0.6)*600)
            if cl<40:
                cl=40
            if cl>250:
                cl=250
                
            cl=200
            if status == "INIT":
                lcd.setRGB(250, 120, 250)    # rose
            elif status == "GPS ":             
                lcd.setRGB(250,250,  0)      # jaune
            elif status == "DGPS":            
                lcd.setRGB( cl,  0, cl)      # violet
                
            elif status == "RTKx":            
                lcd.setRGB(  0, cl,  0)      # vert
            elif status == "RTKf":            
                lcd.setRGB(  0,  0, cl)      # bleu
                
            elif status == "NOFX":
                lcd.setRGB(150,150,150)      # gris clair
            elif status == "ERR":             
                lcd.setRGB(250,  0,  0)      # rouge
            else:                             
                lcd.setRGB( 40, 40, 40)      # gris
    except Exception as e:
        Trace(trc_MSG, LOG_CRT, sys._getframe().f_code.co_name, repr(e))
        
#==============================================================================
# lcd_Init
#------------------------------------------------------------------------------
#
def lcd_Init():
    Trace(trc_IN, LOG_DBG, sys._getframe().f_code.co_name)
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
    Trace(trc_OUT, LOG_DBG, sys._getframe().f_code.co_name)

#==============================================================================
# lcd_SetScreen
#------------------------------------------------------------------------------
#
def lcd_SetScreen(l1,l2):
    #Trace(trc_IN, format("[%s]" % (sys._getframe().f_code.co_name)))
    
    lcd.clear()
    lcd.setCursor(0, 0)
    lcd.printout(l1)
    lcd.setCursor(0, 1)
    lcd.printout(l2)

    #Trace(trc_OUT, format("[%s]" % (sys._getframe().f_code.co_name)))


#==============================================================================
# lcd_ReadButton
#------------------------------------------------------------------------------
# Read the key value
#
def lcd_ReadButton():
    #Trace(trc_IN, format("[%s]" % (sys._getframe().f_code.co_name)))

    ioSELECT = 16
    ioUP     = 17
    ioDOWN   = 18
    ioLEFT   = 19
    ioRIGHT  = 20
    
    ret = 0
    if (GPIO.input(ioSELECT) == 1):
        Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name, "ReadBTN:btnSELECT")
        ret += btnSELECT
    if (GPIO.input(ioUP) == 1):
        Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name, "ReadBTN:btnUP")
        ret += btnUP
    if (GPIO.input(ioDOWN) == 1):
        Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name, "ReadBTN:btnDOWN")
        ret += btnDOWN
    if (GPIO.input(ioLEFT) == 1):
        Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name, "ReadBTN:btnLEFT")
        ret += btnLEFT
    if (GPIO.input(ioRIGHT) == 1):
        Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name, "ReadBTN:btnRIGHT")
        ret += btnRIGHT

    #Trace(trc_OUT, sys._getframe().f_code.co_name+" ("+str(ret)+")")
    return ret

#==============================================================================
# gpx_ExtractFields
#------------------------------------------------------------------------------
#
def gpx_ExtractFields(lcFields, pt):
    global tmFirst, tmLast, dtFirst, dtLast

    ret=False
    try:
        _val = int(lcFields[6])
        if ((_val > 0) and (len(lcFields)>8)):
            pt["dat"]  = lcFields[0]
            pt["tim"]  = lcFields[1]
            pt["lat"]  = DecimalDegrees(lcFields[2])
            pt["lon"]  = DecimalDegrees(lcFields[3])
            pt["alt"]  = float(lcFields[4])
            pt["sat"]  = int(lcFields[5])
            pt["fix"]  = int(lcFields[6])
            pt["dop"]  = float(lcFields[7])
            pt["spd"]  = float(lcFields[8])

            if ((pt["lat"]+pt["lon"])<40):
                Trace(trc_MSG, LOG_ERR, sys._getframe().f_code.co_name,
                    f"Position hors range [{pt['lat']} / {pt['lon']}]  ")
            else:
                ret=True
        else:
            Trace(trc_MSG, LOG_WRN, sys._getframe().f_code.co_name,"NoFIX - Trame ignorée")
    except Exception as e:
        ret=False
        Trace(trc_MSG, LOG_CRT, sys._getframe().f_code.co_name, repr(e))
    return(ret)

#==============================================================================
# gpx_Convert
#------------------------------------------------------------------------------
#
def gpx_Convert():
    global glbSatCtr

    Trace(trc_IN, LOG_DBG, sys._getframe().f_code.co_name)
    fileMask = iniRootPath+"/nmea/*.000"

    ggaLines=[]
    ctr=0
    if (len(glob.glob(fileMask)) == 0):
        Trace(trc_MSG, LOG_ERR, sys._getframe().f_code.co_name, f"Aucun fichier trouve a l'emplacement designe [{fileMask}]")
    else:
        for fName in glob.glob(fileMask):

            # Pour chaque fichier *.000 trouvé, generer un fichier GGA contenant les infos 
            # utiles pour generer un GPX plus tard
            #
            lcd.clear()
            l1 = "GPX Convert("+str(ctr)+")"
            lcd.setCursor(0, 0)
            lcd.printout(l1)

            fragCtr = 0
            ggaLines=[]
            while (True):
                try:
                    baseName=format("%s.%03d" % (fName.split(".")[0],fragCtr))
                    Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name, baseName)
                    if (os.path.exists(baseName)):
                        ctr+=1
                        l1 = "GPX Convert("+str(ctr)+")"
                        l2 = baseName.split("-")[1][:14]
                        lcd_SetScreen(l1,l2)
                        Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name, f"[{l1}] / [{l2}]")
                        
                        fragCtr=fragCtr+1
                        with open(baseName) as f:
                            lines = f.read()
                        f.close()
                        
                        lines=lines.split("\n")
                        for line in lines:
                            if (line[9:15] == "$GNRMC") and (len(line)>30):
                            
                                # si ligne RMC non vide, remoriser la derniere vitesse recue
                                # et la date 
                                #
                                fields = line.split(",")
                                if ((len(fields)+1) == len(rmcRecord)):
                                    fields = dict(zip(rmcRecord, fields))
                                    if (len(fields["spd"])>0):
                                        lastSpeed = float(fields["spd"]) * 1.852
                                    else:
                                        lastSpeed = -1
                                    if (len(fields["dat"])>0):
                                        x=fields["dat"]
                                        lastDate  = f"20{x[4:6]}-{x[2:4]}-{x[0:2]}"
                                    else:
                                        lastDate  = "01/01/2000"

                            if (line[9:15] == "$GNGSA") and (len(line)>50):

                                # si ligne GSA non vide, remoriser le nombre de satellites total recus
                                #
                                fields = line.split(",")
                                if ((len(fields)+1) == len(gsaRecord)):
                                    satID=int(fields[18][:1])
                                    glbSatCtr[satID]=0
                                    for nf in range(3,14):
                                        if len(fields[nf])>0:
                                            glbSatCtr[satID]+=1
                            if (line[9:15] == "$GNGGA") and (len(line)>35):

                                # Lors de chaque reception GGA (1hz), ecrire une ligne contenant les  
                                # infos pour generer le GPX ensuite : les infos du GGA principalement  
                                # plus le nb satellites et la vitesse recus par d'autres trames
                                #
                                fields = line[9:].split(",")
                                fields = dict(zip(ggaRecord, fields))
                                
                                x=fields["time"]
                                lastTime  = f"{x[0:2]}:{x[2:4]}:{x[4:6]}"
                                
                                ggaLines.append(    lastDate       + "," +
                                                    lastTime       + "," +
                                                    fields["lat" ] + "," +
                                                    fields["lon" ] + "," +
                                                    fields["hgth"] + "," +
                                                    fields["nsat"] + "," +
                                                    fields["qlty"] + "," +
                                                    fields["hdop"] + "," +
                                                    str(lastSpeed) + "," +
                                                    str(sum(glbSatCtr))
                                                    )
                                #glbSatCtr       = [ 0, #                   0 = QZSS	
                                #                    0, #                   1 = GPS
                                #                    0, #                   2 = GLONASS
                                #                    0, #                   3 = Galileo
                                #                    0, #                   4 = BeiDou
                                #                    0  #                   5 = ???
                                #                   ]
                    else:
                        break
                except Exception as e:
                    Trace(trc_MSG, LOG_CRT, sys._getframe().f_code.co_name, repr(e))

            # Convertir en GPX le fichier compilé puis le sumprimer
            #
            head, tail = os.path.split(fName)
            nmea_AnalyzeFile(ggaLines,tail.split(".")[0] )

            if (len(ggaLines)>0):
                ggaFic=fName.split(".")[0] + ".gga"
                ggaHdl=open(ggaFic,"w")
                for xx in ggaLines:
                    ggaHdl.write(xx+"\n")
                ggaHdl.close()
    Trace(trc_OUT, LOG_DBG, sys._getframe().f_code.co_name)

#==============================================================================
# gpx_Fill
#------------------------------------------------------------------------------
# Selon le code recu, ecrit soit l'entete du fichier GPX, soit
# le point courant soit les lignes de fin de fichier
#
def gpx_Fill(handler, fGpx, code, pt):
    Trace(trc_IN, LOG_DBG, sys._getframe().f_code.co_name)
    if code == 'header':
        line =                                                   \
               "<gpx version=\"1.0\">\n"                         \
               "   <trk>\n"                                      \
               "      <name>" + fGpx + "</name>\n"               \
               "      <trkseg>\n"
        handler.write(line)
    elif code == 'point':
        if ((pt["lat"] != 0) and (pt["lon"] != 0) and (pt["spd"] > 1)):
            line = format(                                       \
                "<trkpt lat='%f' lon='%f'>"                      \
                "<time>%sT%sZ</time>"                            \
                "<ele>%d</ele>"                                  \
                "<speed>%06.2f</speed>"                          \
                "<hdop>%06.2f</hdop>"                            \
                "<fix>%s</fix>"                                  \
                "<sat>%d</sat>"                                  \
                "</trkpt>\n")                                    \
                % (pt["lat"],pt["lon"],pt["dat"],pt["tim"],      \
                   pt["alt"],pt["spd"],pt["dop"],pt["fix"],      \
                   pt["sat"])
            handler.write(line)
        else:
           Trace(trc_MSG, LOG_DBG, sys._getframe().f_code.co_name, "Position 0/0 non ecrite")
    elif code == 'footer':
        line =  "      </trkseg>\n"                              \
                "   </trk>\n"                                    \
                "</gpx>\n"              
        handler.write(line)
    else:
         Trace(trc_OUT, "code ["+code+"] erroné")
    Trace(trc_OUT, LOG_DBG, sys._getframe().f_code.co_name)

#==============================================================================
# csv_Fill
#------------------------------------------------------------------------------
#
def csv_Fill(hCsv, fCsv, pt):
    #Trace(trc_IN, format("[%s]" % (sys._getframe().f_code.co_name)))

    ln=format("\n%s;%s;%f;%f;%f;%f;%d;%d;%f" % \
               (pt['dat'],pt['tim'],\
                pt['lat'],pt['lon'],\
                pt['alt'],pt['dop'],\
                pt['sat'],pt['fix'],\
                pt['spd'])
               )
    hCsv.write(ln)
    #Trace(trc_OUT, format("[%s]" % (sys._getframe().f_code.co_name)))

#==============================================================================
# nmea_CalculChecksun
#------------------------------------------------------------------------------
# Calcul du checksum attendu de la ligne en cours
#
def nmea_CalculChecksun(ln):
    encoded_string = ln.encode()
    byte_array = bytearray(encoded_string)
    
    # Calcul du checksum attendu de la ligne en cours
    #   (XOR entre tous les caractères entre le $ et l'* non compris)
    #
    cks = 0
    for i in range(1, len(byte_array) - 3):
        cks ^= byte_array[i]
    
    if (cks==int(ln[-2:],16)):
        ret=True
    else:
        ret=False
    return ret
                       
#==============================================================================
# nmea_AnalyzeFile
#------------------------------------------------------------------------------
#
def nmea_AnalyzeFile(gga,fname):
    global tmFirst, tmLast, dtFirst, dtLast, statFix

    try:
        Trace(trc_IN, LOG_DBG, sys._getframe().f_code.co_name)
        gpxPt = { "tim" : "x.y.z",
                  "dat" : "x.y.z",
                  "lat" : 0.0,
                  "lon" : 0.0,
                  "alt" : 0.0,
                  "spd" : 0.0,
                  "dop" : 0,
                  "sat" : 0,
                  "fix" : 0
                }
    
        ficGpx=iniRootPath+"/gpx/"+fname[5:]+".gpx"
        #if (os.path.exists(ficGpx)):
        #    Trace(trc_MSG, LOG_DBG, sys._getframe().f_code.co_name, f"Fichier {ficGpx} déjà généré")
        #else:
        if True:
            ficCsv=iniRootPath+"/csv/"+fname[5:]+".csv"
            Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name,  f"Traitement de {ficGpx}")
            gpxHdl=open(ficGpx,"w")
            csvHdl=open(ficCsv,"w")
            
            csvHdl.write("dat;tim;lat;lon;alt;dop;sat;fix;vit")
            
            gpx_Fill(gpxHdl, ficGpx, 'header',gpxPt)
            ctrLine=0
            ctrValid=0
            DisplayFixStat(0)
            for line in gga:
                print("\rAnalyse ligne [%d] ..." % ctrLine, end='')
                line = line.rstrip('\n')
                ctrLine+=1
                
                # Separation de la ligne en champs CSV
                #
                fields = line.split(",")
    
                if (gpx_ExtractFields(fields,gpxPt)):
                    ctrValid+=1
                    gpx_Fill(gpxHdl, ficGpx, 'point', gpxPt)
                    csv_Fill(csvHdl, ficCsv, gpxPt)
                    if int(gpxPt["fix"]) <6:
                        statFix[int(gpxPt["fix"])] += 1
            gpx_Fill(gpxHdl, ficGpx, 'footer',gpxPt)
            gpxHdl.close()
            csvHdl.close()
            if (ctrValid==0):
                Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name, f"GPX and CSV empty - deleted {ficGpx}/{ficCsv}")
                os.remove(ficGpx)
                os.remove(ficCsv)
            DisplayFixStat(1)
            Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name,  f"=== Statistiques FIX de {ficGpx}")
    except Exception as e:
        Trace(trc_MSG, LOG_CRT, sys._getframe().f_code.co_name, repr(e))
    Trace(trc_OUT, LOG_DBG, sys._getframe().f_code.co_name)
    
#==============================================================================
# DisplayFixStat
#------------------------------------------------------------------------------
# 
#
def DisplayFixStat(code):
    global statFix
    
    if (code==0):
        for i in range(0,len(statFix)):
            statFix[i] = 0        
    else:
        if ((sum(statFix))>0):
            for i in range(0,len(statFix)):
                Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name, f"   - Fix {posFix[i]}  {statFix[i]} records ({int(statFix[i]/sum(statFix)*100)}%)")
        else:
            for i in range(0,len(statFix)):
                Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name, f"   - Fix {posFix[i]}  {statFix[i]} records")
    
    
#==============================================================================
# nmea_DecodeGGA
#------------------------------------------------------------------------------
# $GNGGA,122220.00,4542.4812848,N,00451.2467166,E,2,12,0.60,188.552,M,47.358,M,,0136*4A
#
def nmea_DecodeGGA(trame):
    global glbLat, glbLon, glbFix, glbNSat, glbHeight, glbHDop, glbLastGGA, statFix

    Trace(trc_IN, LOG_DBG, sys._getframe().f_code.co_name)
    Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name, gnssType[trame[1:3]])

    glbLastGGA = trame
    glbFix=0
    try:
        fields = trame.split(",")
        if ((len(fields)+1) == len(ggaRecord)):
            fields = dict(zip(ggaRecord, fields))
            lcLat = DecimalDegrees(fields["lat"])
            lcLon = DecimalDegrees(fields["lon"])
            lcFix = int(fields["qlty"])
            lcNSat = int(fields["nsat"])
            lcHDop = float(fields["hdop"])
            if (len(fields["hgth"])>0):
                lcHeight = int(float(fields["hgth"]))
            else:
                lcHeight = 0
            if ((lcLat < 50) & (lcLat > 40) & (lcLon > 0) & (lcLon < 6)):
                glbLat = lcLat + iniOffset
                glbLon = lcLon + iniOffset
                glbFix = lcFix
                glbNSat = lcNSat
                glbHeight = lcHeight
                glbHDop = lcHDop
    
            if ((lcFix >=0) and (lcFix <= 6)):
                statFix[lcFix] += 1

            lcd_SetColor(posFix[lcFix])
#         if (glbFix == 1):
#                lcd_SetColor("GPS")
#            elif (glbFix == 2):
#                lcd_SetColor("DGPS")
#            elif (glbFix == 4):
#                lcd_SetColor("RTKx")
#            elif (glbFix == 5):
#                lcd_SetColor("RTKf")
#            else:
#                lcd_SetColor("NOFIX")
        else:
            Trace(trc_MSG, LOG_ERR, sys._getframe().f_code.co_name, 
                format("GGA Longueur incorrecte [%d] recus, [%d] attendus" % (len(fields)+1,len(ggaRecord))))
    except Exception as e:
        Trace(trc_MSG, LOG_CRT, sys._getframe().f_code.co_name, repr(e))
    Trace(trc_OUT, LOG_DBG, sys._getframe().f_code.co_name)

#==============================================================================
# nmea_DecodeGSA
#------------------------------------------------------------------------------
# $GNGSA,A,3,20,30,22,11,07,05,14,09,13,,,,1.52,0.81,1.29,1*09
# $GNGSA,A,3,78,84,68,79,69,85,,,,,,,1.52,0.81,1.29,2*06
# $GNGSA,A,3,34,36,05,09,04,31,15,24,,,,,1.52,0.81,1.29,3*0C
# $GNGSA,A,3,46,36,19,22,,,,,,,,,1.52,0.81,1.29,4*0E
# $GNGSA,A,3,,,,,,,,,,,,,1.52,0.81,1.29,5*00 
#
def nmea_DecodeGSA(trame):
    global glbSatCtr

    Trace(trc_IN, LOG_DBG, sys._getframe().f_code.co_name)
    Trace(trc_MSG, LOG_DBG, sys._getframe().f_code.co_name, gnssType[trame[1:3]])

    try:
        fields = trame.split(",")
        if ((len(fields)+1) == len(gsaRecord)):
            if (len(fields) == 19):
                satID=int(fields[18][:1])
                glbSatCtr[satID]=0
                for nf in range(3,14):
                    if len(fields[nf])>0:
                        glbSatCtr[satID]+=1
            else:
                Trace(trc_MSG, LOG_ERR, sys._getframe().f_code.co_name, f"Trame GSA [{trame}] incorrecte")
        else:
            Trace(trc_MSG, LOG_ERR, sys._getframe().f_code.co_name, 
                  f"GSA Longueur incorrecte [{len(fields)+1}] recus, [{len(ggaRecord)}] attendus")
    except Exception as e:
        Trace(trc_MSG, LOG_CRT, sys._getframe().f_code.co_name, repr(e))
    if (satID==1):
        Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name, f"GPS:{glbSatCtr[1]} GAL:{glbSatCtr[3]} GLO:{glbSatCtr[2]} BEI:{glbSatCtr[4]}")
    Trace(trc_OUT, LOG_DBG, sys._getframe().f_code.co_name)
    
#==============================================================================
# nmea_DecodeRMC
#------------------------------------------------------------------------------
# $GNRMC,131058.00,A,4544.4967369,N,00449.6703838,E,10.912,210.46,040224,,,F,V*30
# $GNRMC,200349.00,V,,,,,,,030224,,,N,V*12
#
def nmea_DecodeRMC(trame):
    global lastSpeed,lastDate

    Trace(trc_IN, LOG_DBG, sys._getframe().f_code.co_name)
    Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name, gnssType[trame[1:3]])

    try:
        fields = trame.split(",")
        if ((len(fields)+1) == len(rmcRecord)):
            fields = dict(zip(rmcRecord, fields))
            if (len(fields["spd"])>0):
                lastSpeed = float(fields["spd"])
            else:
                lastSpeed = -1
            if (len(fields["dat"])>0):
                lastDate  = fields["dat"]
            else:
                lastDate  = "01/01/2000"
        else:
            Trace(trc_MSG, LOG_ERR, sys._getframe().f_code.co_name, 
                  format("Longueur incorrecte [%d] recus, [%d] attendus" % (len(fields)+1,len(rmcRecord))))
    except Exception as e:
        Trace(trc_MSG, LOG_CRT, sys._getframe().f_code.co_name, repr(e))
    Trace(trc_OUT, LOG_DBG, sys._getframe().f_code.co_name)

#==============================================================================
# ini_Write
#------------------------------------------------------------------------------
#
def ini_Write():
    Trace(trc_IN, LOG_DBG, sys._getframe().f_code.co_name)
    try:
        config = configparser.ConfigParser()
        config.read(iniRootPath+"/rover.ini")
        config["Position"]["latitude"]  = str(glbLat)
        config["Position"]["longitude"] = str(glbLon)
        with open(iniRootPath+"/rover.ini", 'w') as configfile:
            config.write(configfile)
    except Exception as e:
        Trace(trc_MSG, LOG_CRT, sys._getframe().f_code.co_name, repr(e))
    Trace(trc_OUT, LOG_DBG, sys._getframe().f_code.co_name)

#==============================================================================
# ini_Read
#------------------------------------------------------------------------------
#
def ini_Read():
    global iniCstIP, iniCstPort, iniCstUser, iniCstPwd, iniCstPoint
    global iniDmpSize, iniLcdFreeze, iniDefLat, iniDefLon, iniOffset, iniDureeMax, iniSzMaxNMEA,iniWifiSSID,iniWifiPwd,iniWifiIndex,iniLogging

    Trace(trc_IN, LOG_DBG, sys._getframe().f_code.co_name)

    # Valeur par défaut, ecrasée par le contenu du INI
    #
    iniCstIP       = "caster.centipede.fr"
    iniCstPort     = 2101
    iniCstUser     = "centipede"
    iniCstPwd      = "centipede"
    iniCstPoint    = "COMBE"
    iniOffset      = 0
    iniDefLon      = 4.500
    iniDefLat      = 45.446
    iniLcdFreeze   = 10
    iniDmpSize     = 100
    iniSzMaxNMEA   = 5000
    iniDureeMax    = 5
    iniRootPath    = "/home/blueb/RoverRTK"
    iniWifiIndex   = 0

    try:
        config = configparser.ConfigParser()
        config.read(iniRootPath+"/rover.ini")

        # Section GENERAL
        #
        iniRootPath    = config["General"]["rootpath"]
        iniDureeMax    = int(config["General"]["dureemax"])
        if (config["General"]["offset"] == "1"):
            iniOffset = random.random()
        
        # Section TRACES
        #
        iniDmpSize     = int(config["Traces"]["dumpsize"])
        iniSzMaxNMEA   = int(config["Traces"]["tramesNMEA"])
        iniLogging     = int(config["Traces"]["logging"])
        
        # Section ECRAN
        #
        iniLcdFreeze   = int(config["Ecran"]["freeze"])

        # Section POSITION
        #
        iniDefLat      = float(config["Position"]["latitude"])
        iniDefLon      = float(config["Position"]["longitude"])

        # Section CENTIPEDE
        #
        iniCstUser     = config["Centipede"]["user"]
        iniCstPwd      = config["Centipede"]["password"]
        iniCstPoint    = config["Centipede"]["base"]
        iniCstIP       = config["Centipede"]["serveur"]
        iniCstPort     = int(config["Centipede"]["port"])
        
        # Section WIFI
        #
        iniWifiSSID    = config["Wifi"]["reseau"]
        iniWifiPwd     = config["Wifi"]["password"]
        iniWifiIndex   = int(config["Wifi"]["index"])
    except Exception as e:
        Trace(trc_MSG, LOG_CRT, sys._getframe().f_code.co_name, repr(e))
    Trace(trc_OUT, LOG_DBG, sys._getframe().f_code.co_name)

#==============================================================================
# rtk_Ping
#------------------------------------------------------------------------------
#
def rtk_Ping(server):
    Trace(trc_IN, LOG_DBG, sys._getframe().f_code.co_name, f"Ping {server}")
    
    if (os.system("ping -c 1 "+server) == 0):
        isServerOk=True
    else:
        isServerOk=False
    
    Trace(trc_OUT, LOG_DBG, sys._getframe().f_code.co_name)
    return isServerOk

#==============================================================================
# rtk_GetBaseList
#------------------------------------------------------------------------------
#
def rtk_GetBaseList():
    global glbLstBases

    Trace(trc_IN, LOG_DBG, sys._getframe().f_code.co_name)

    # Requete pour recup de la table CASTER et reponse attendue en cas de succes
    #
    BaseListe_HDR = "SOURCETABLE 200 OK"
    BaseListe_REQ = 'GET / HTTP/1.0 \n' + \
                    'Host: ' + iniCstUser + ':' + iniCstPwd + '\n' + \
                    'User-Agent: NTRIP Rover' + "\n" + \
                    'Accept: /' + "\n" + \
                    'Authorization: Basic ' + iniCstUser + ':' + iniCstPwd + "\n\n"

    # Init connexion to base
    #
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)
    server_address = (iniCstIP, iniCstPort)
    sock.settimeout(2)
    sock.connect(server_address)

    # Send base list request
    #
    req = bytes(BaseListe_REQ, encoding='utf-8')
    Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name, f"Connect to [{iniCstIP}] port [{iniCstPort}] req [{req}]")
    sock.sendall(req)

    # Read table (~110 000 bytes)
    #
    glbTcpRxCounter = 0
    data = sock.recv(len(BaseListe_HDR))
    if (data.decode() == BaseListe_HDR):
        Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name, "Receiving caster table ok")
        while True:
            data = sock.recv(1024)
            glbTcpRxCounter = glbTcpRxCounter + len(data)
            glbLstBases = glbLstBases + data.decode()
            if len(data) == 0:
                break
    else:
        Trace(trc_MSG, LOG_ERR, sys._getframe().f_code.co_name, "Error receving caster table")
    sock.close()
    
    # Save caster table
    #
    file = open(iniRootPath + "/datas/caster-table.csv", "w")
    file.write(glbLstBases)
    file.close()
    glbLstBases = glbLstBases.split("\n")
    Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name, f"Table received [{glbTcpRxCounter}] bytes / [{len(glbLstBases)}] bases")
    Trace(trc_OUT, LOG_DBG, sys._getframe().f_code.co_name)

#==============================================================================
# rtk_GetNearestBase
#------------------------------------------------------------------------------
#
def rtk_GetNearestBase(name):
    global basePoint

    Trace(trc_IN, LOG_DBG, sys._getframe().f_code.co_name, f"base={name}")
    nearestDst  = 500
    nearestName = "none"

    i = 0
    for i in range(1, len(glbLstBases)):
        fld = glbLstBases[i].split(";")
        fields = dict(zip(baseRecord, fld))
        if fld[0] == "STR":
            dst = Distance((iniDefLat, iniDefLon), (float(fields["lat"]), float(fields["lon"])))
            if dst < nearestDst:
                basePoint = fields["mpt"]
                jrn = format("I=%d Base=%s Lat=%f Lon=%f Dst=%f" % (
                        i, fields["mpt"], float(fields["lat"]), float(fields["lon"]), dst))
                Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name, jrn)
                nearestDst  = dst
                nearestName = fields["mpt"]
    Trace(trc_OUT, LOG_DBG, sys._getframe().f_code.co_name)
    return(nearestName)

#==============================================================================
# rtk_IsKnownBase
#------------------------------------------------------------------------------
#
def rtk_IsKnownBase(name):
    Trace(trc_IN, LOG_DBG, sys._getframe().f_code.co_name, f"base={name}")

    ret=False
    name=";"+name+";"
    for i in range(0,len(glbLstBases)):
        if name in (glbLstBases[i]):
            Trace(trc_IN, LOG_DBG, sys._getframe().f_code.co_name, f"Base [{name}] trouvé index [{i}]")
            ret=True
            break
    Trace(trc_OUT, LOG_DBG, sys._getframe().f_code.co_name)
    return(ret)
    
#==============================================================================
# rtk_GetBaseInfos
#------------------------------------------------------------------------------
#
def rtk_GetBaseInfos(name):
    global basePoint, baseDist, baseUplGGA, baseIndex

    Trace(trc_IN, LOG_DBG, sys._getframe().f_code.co_name, f"base={name}")

    for i in range(1, len(glbLstBases)):
        fld = glbLstBases[i].split(";")
        if (len(fld) > 10):
            fields = dict(zip(baseRecord, fld))
            if fields["mpt"] == name:
                baseIndex  = i
                basePoint  = fields["mpt"]
                baseDist   = Distance((iniDefLat, iniDefLon), (float(fields["lat"]), float(fields["lon"])))
                baseUplGGA = fields["nmea"]
                Trace(trc_IN, LOG_DBG, sys._getframe().f_code.co_name, f"I={i} Base={fields['mpt']}")
                break
    Trace(trc_OUT, LOG_DBG, sys._getframe().f_code.co_name)

#==============================================================================
# rtk_BaseConnect [Thread]
#------------------------------------------------------------------------------
#
def rtk_BaseConnect(name):
    global glbTcpBuffer
    global glbTcpRtcm
    global glbMainExit
    global glbTcpExit
    global glbTcpRxCounter,glbTcpTxCounter
    global glbInitRTKok

    Trace(trc_IN, LOG_DBG, sys._getframe().f_code.co_name)
    BaseConnect_HDR = "ICY 200 OK"
    BaseConnect_REQ = 'GET /' + name + ' HTTP/1.0 \n' + \
                      'Host: ' + iniCstUser + ':' + iniCstPwd + '\n' + \
                      'User-Agent: NTRIP Rover' + "\n" + \
                      'Accept: ' + '\n' + \
                      'Authorization: Basic ' + iniCstUser + ':' + iniCstPwd + '\n\n'

    # Init connexion to base
    #
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)
    server_address = (iniCstIP, iniCstPort)
    sock.settimeout(None)
    sock.connect(server_address)
    Trace(trc_IN, LOG_DBG, sys._getframe().f_code.co_name, BaseConnect_REQ)
    # Send base connect request
    #
    req = bytes(BaseConnect_REQ, encoding='utf-8')
    jrn = format("connect to [%s] port [%s] req [%s]" % (iniCstIP, iniCstPort, req))
    Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name, jrn)
    sock.sendall(req)

    # Continously read data sent by base
    #
    file = open(iniRootPath + "/datas/base-dump.bin", "wb")
    glbTcpRxCounter = 0

    glbData = sock.recv(len(BaseConnect_HDR))
    try: 
        if (glbData.decode() == BaseConnect_HDR):
            Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name, "Receiving base data  ok")
        
            while ((not glbMainExit) and (not glbTcpExit)):
                rx = sock.recv(1024)
                if (len(rx) > 0):
                    glbTcpBuffer += rx
                    glbTcpRtcm   += rx
                    glbTcpRxCounter = glbTcpRxCounter + len(rx)

                    # Field "nmea" in caster table indicate if it's necessary or not
                    # to send back GGA data to base or if it's useless
                    #
                    if (baseUplGGA):
                        sock.send(glbLastGGA.encode('ascii'))
                        glbTcpTxCounter += len(glbLastGGA)
                        
                    # We limit dump file to 'iniDmpSize' Ko
                    #
                    if (glbTcpRxCounter < (iniDmpSize * 1024)):
                        file.write(rx)
                    Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name, format("TCP: Reception [%d] Total [%d ko]" % (len(rx), int(glbTcpRxCounter / 1024))))
                else:
                    Trace(trc_MSG, LOG_ERR, sys._getframe().f_code.co_name, "NoTCP data")
                #if len(glbTcpRtcm)>1024:
                #    break
        else:
            Trace(trc_MSG, LOG_ERR, sys._getframe().f_code.co_name, "TCP connexion ERR !!!")
    except Exception as e:
        Trace(trc_MSG, LOG_CRT, sys._getframe().f_code.co_name, repr(e))
        if str(socket.errno.ECONNRESET) in str(e):
            Trace(trc_MSG, LOG_ERR, sys._getframe().f_code.co_name, "Connection reset")                       
            glbInitRTKok = False
    file.close()
    Trace(trc_OUT, LOG_DBG, sys._getframe().f_code.co_name)

#==============================================================================
# sys_GetIPaddress
#------------------------------------------------------------------------------
# 'lo': [
#    snicaddr(family=<AddressFamily.AF_INET: 2>, address='127.0.0.1', netmask='255.0.0.0', broadcast=None, ptp=None),
#    snicaddr(family=<AddressFamily.AF_INET6: 10>, address='::1', netmask='ffff:ffff:ffff:ffff:ffff:ffff:ffff:ffff', broadcast=None, ptp=None),
#    snicaddr(family=<AddressFamily.AF_PACKET: 17>, address='00:00:00:00:00:00', netmask=None, broadcast=None, ptp=None)],
# 'eth0': [
#     snicaddr(family=<AddressFamily.AF_INET: 2>, address='192.168.1.213', netmask='255.255.255.0', broadcast='192.168.1.255', ptp=None),
#     snicaddr(family=<AddressFamily.AF_INET6: 10>, address='xx:xx:xx:xx:xx:xx', netmask='ffff:ffff:ffff:ffff::', broadcast=None, ptp=None),
#     snicaddr(family=<AddressFamily.AF_INET6: 10>, address='xx:xx:xx:xx:xx:xx', netmask='ffff:ffff:ffff:ffff::', broadcast=None, ptp=None),
#     snicaddr(family=<AddressFamily.AF_PACKET: 17>, address='xx:xx:xx:xx:xx:xx', netmask=None, broadcast='ff:ff:ff:ff:ff:ff', ptp=None)],
# 'wlan0': [
#     snicaddr(family=<AddressFamily.AF_INET: 2>, address='192.168.1.214', netmask='255.255.255.0', broadcast='192.168.1.255', ptp=None),
#     snicaddr(family=<AddressFamily.AF_INET6: 10>, address='xx:xx:xx:xx:xx:xx', netmask='ffff:ffff:ffff:ffff::', broadcast=None, ptp=None),
#     snicaddr(family=<AddressFamily.AF_INET6: 10>, address='xx:xx:xx:xx:xx:xx', netmask='ffff:ffff:ffff:ffff::', broadcast=None, ptp=None),
#     snicaddr(family=<AddressFamily.AF_PACKET: 17>, address='xx:xx:xx:xx:xx:xx', netmask=None, broadcast='ff:ff:ff:ff:ff:ff', ptp=None)]}
#
# Accès via mobile
# 1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN group default qlen 1000
#     link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
#     inet 127.0.0.1/8 scope host lo
#        valid_lft forever preferred_lft forever
#     inet6 ::1/128 scope host 
#        valid_lft forever preferred_lft forever
# 2: eth0: <NO-CARRIER,BROADCAST,MULTICAST,UP> mtu 1500 qdisc pfifo_fast state DOWN group default qlen 1000
#     link/ether xx:xx:xx:xx:xx:xx brd ff:ff:ff:ff:ff:ff
# 3: wlan0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc pfifo_fast state UP group default qlen 1000
#     link/ether xx:xx:xx:xx:xx:xx brd ff:ff:ff:ff:ff:ff
#     inet 192.168.192.187/24 brd 192.168.192.255 scope global dynamic noprefixroute wlan0
#        valid_lft 3076sec preferred_lft 2626sec
#     inet6 xx:xx:xx:xx:xx:xx:9ce/64 scope global dynamic mngtmpaddr noprefixroute 
#        valid_lft 6930sec preferred_lft 6930sec
#     inet6 xx:xx:xx:xx:xx:xx/64 scope link 
#        valid_lft forever preferred_lft forever

def sys_GetIPaddress():
    global glbFreeze

    Trace(trc_IN, LOG_DBG, sys._getframe().f_code.co_name)
    lcd.clear()

    ad = psutil.net_if_addrs()
    glbWifiAd = ad["wlan0"][0].address
    glbEthAd = ad["eth0"][0].address

    l1 = ">" + glbWifiAd
    l2 = ">" + glbEthAd
    lcd_SetScreen(l1,l2)

    glbFreeze = iniLcdFreeze
    Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name, f"sys_GetIPaddress: [{l1}] / [{l2}]")
    Trace(trc_OUT, LOG_DBG, sys._getframe().f_code.co_name)



#==============================================================================
# sys_GetMACaddress
#------------------------------------------------------------------------------
#
def sys_GetMACaddress(itf):
    global glbFreeze

    Trace(trc_IN, LOG_DBG, sys._getframe().f_code.co_name)
    ad = psutil.net_if_addrs()
    ret = ad[itf][0].address
    Trace(trc_OUT, LOG_DBG, sys._getframe().f_code.co_name)
    return (ret)

#==============================================================================
# sys_SatInfos
#------------------------------------------------------------------------------
#
def sys_SatInfos():
    global glbFreeze

    Trace(trc_IN, LOG_DBG, sys._getframe().f_code.co_name)
    lcd.clear()

    try:
        l1 = format("GPS:%02d GAL:%02d" % (glbSatCtr[1],glbSatCtr[3]))
        l2 = format("GLO:%02d BEI:%02d" % (glbSatCtr[2],glbSatCtr[4]))
    
        lcd_SetScreen(l1,l2)
        lcd.setCursor(15, 1)
        lcd.write(0)
    
        glbFreeze = iniLcdFreeze
        Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name, f"[{l1}] / [{l2}]")
    except Exception as e:
        Trace(trc_MSG, LOG_CRT, sys._getframe().f_code.co_name, repr(e))
    Trace(trc_OUT, LOG_DBG, sys._getframe().f_code.co_name)

#==============================================================================
# sys_GetOsInfos
#------------------------------------------------------------------------------
#
def sys_GetOsInfos():
    global glbFreeze

    Trace(trc_IN, LOG_DBG, sys._getframe().f_code.co_name)
    lcd.clear()

    try:
        cpu = CPUTemperature()
        cpufreq = psutil.cpu_freq()
        svmem = psutil.virtual_memory()
        
        l1 = format("CPU:%2.0f%% MEM:%2.0f%%" % (psutil.cpu_percent(), svmem.percent))
        l2 = format("T:%2.0f°c F:%dHz" % (cpu.temperature, cpufreq.current))
    
        lcd_SetScreen(l1,l2)
        lcd.setCursor(15, 1)
        lcd.write(0)
    
        glbFreeze = iniLcdFreeze
        Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name, f"[{l1}] / [{l2}]")
    except Exception as e:
        Trace(trc_MSG, LOG_CRT, sys._getframe().f_code.co_name, repr(e))
    Trace(trc_OUT, LOG_DBG, sys._getframe().f_code.co_name)

#==============================================================================
# sys_ListUsbPort
#------------------------------------------------------------------------------
#
def sys_ListUsbPort():
    Trace(trc_OUT, LOG_DBG, sys._getframe().f_code.co_name)
    usbPort = ""
    for port in range(0, 10):
        ttyPort = "/dev/ttyUSB" + str(port)
        if exists(ttyPort):
            usbPort = ttyPort
            Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name, "Port serie=" + usbPort)
            break
    Trace(trc_OUT, LOG_DBG, sys._getframe().f_code.co_name)
    return (usbPort)


#==============================================================================
# sys_WifiStatus
#------------------------------------------------------------------------------
#
def sys_WifiStatus():
    global glbFreeze

    Trace(trc_IN, LOG_DBG, sys._getframe().f_code.co_name)
    try:
        if glbNetworkOk:
            l1="Nw:"+str(glbNetworkOk) + " In:"+str(glbInitRTKok)
            ssid=subprocess.check_output(["iwgetid -r"], shell=True)
            ssid=ssid.decode("utf-8").strip()
         
            l2=ssid
            lcd_SetScreen(l1,l2)
            Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name, f"[{l1}] / [{l2}]")
        else:
            l1="** NoWIFI **"
            l2="** NoWIFI **"
            lcd_SetScreen(l1,l2)
            Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name, f"[{l1}] / [{l2}]")
        glbFreeze = iniLcdFreeze
    except Exception as e:
        Trace(trc_MSG, LOG_CRT, sys._getframe().f_code.co_name, repr(e))
    Trace(trc_OUT, LOG_DBG, sys._getframe().f_code.co_name)
    
#==============================================================================
# sys_BwUsed
#------------------------------------------------------------------------------
#
def sys_BwUsed():
    global glbFreeze

    Trace(trc_IN, LOG_DBG, sys._getframe().f_code.co_name)
    if (glbUsbRxCounter>(1024*1024)) or (glbTcpRxCounter>(1024*1024)):
        l1=format("USB:%04.1f-%04.1fMo" % ((glbUsbRxCounter/(1024*1024)),(glbUsbTxCounter/(1024*1024))))
        l2=format("TCP:%04.1f-%04.1fMo" % ((glbTcpRxCounter/(1024*1024)),(glbTcpTxCounter/(1024*1024))))
    else:
        l1=format("USB:%04d-%04dKo" % (int(glbUsbRxCounter/1024),int(glbUsbTxCounter/1024)))
        l2=format("TCP:%04d-%04dKo" % (int(glbTcpRxCounter/1024),int(glbTcpTxCounter/1024)))
    lcd_SetScreen(l1,l2)
    Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name, f"[{l1}] / [{l2}]")
    glbFreeze = iniLcdFreeze
    Trace(trc_OUT, LOG_DBG, sys._getframe().f_code.co_name)
    
#==============================================================================
# sysArchiveFile
#------------------------------------------------------------------------------
#
def sys_ArchiveFile(fname,nf):
    #Trace(trc_OUT, format("[%s]" % (sys._getframe().f_code.co_name)))
    try:
        for ndx in reversed(range(1,nf)):
            src=fname+"."+str(ndx-1)
            if os.path.exists(src):
                dst=fname+"."+str(ndx)
                print("cp "+src+" > "+dst)
       
                if os.path.exists(dst):
                    os.remove(dst) # file exits, delete it
                shutil.move(src, dst)
        shutil.copy(fname+".jrn", fname+".0")
    except Exception as e:
        #Trace(trc_MSG, format("!!EX [%s / %s]" % 
        #               (sys._getframe().f_code.co_name,repr(e))))
        print("!!EX [%s / %s]" % (sys._getframe().f_code.co_name,repr(e)))
    #Trace(trc_OUT, format("[%s]" % (sys._getframe().f_code.co_name)))

#==============================================================================
# sys_ExportOnUSB
#------------------------------------------------------------------------------
#
def sys_ExportOnUSB(src,dst):
    global glbFreeze
    
    Trace(trc_IN, LOG_DBG, sys._getframe().f_code.co_name, format("Src=%s Dst=%s" % (src,dst)))
    try:
        l1 = "** EXPORT USB **"
        lcd.clear()
        lcd.setCursor(0, 0)
        lcd.printout(l1)
        if os.path.isdir(dst):
            l2 = "Copy to USB..."
            jrn=format("> sys_ExportOnUSB: [%s] [%s]" % (l1, l2))
            Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name, jrn)
            lcd.setCursor(0, 1)
            lcd.printout(l2)
            #cmd="sudo rsync --exclude '.git' --exclude '.*' -av "+src+" " +dst
            #os.system(cmd)
            exit_code = subprocess.call("./bkp-python.sh")
            print(exit_code)
            lcd.setCursor(0, 1)
            l2 = "Export finished..."
            l2 = str(exit_code)
            lcd.printout(l2)
            Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name, "done")
        else:
            l2 = "USB not found"
            Trace(trc_MSG, LOG_ERR, sys._getframe().f_code.co_name, l2)
        glbFreeze = iniLcdFreeze
    except Exception as e:
        Trace(trc_MSG, LOG_CRT, sys._getframe().f_code.co_name, repr(e))
    Trace(trc_OUT, LOG_DBG, sys._getframe().f_code.co_name)
    
#==============================================================================
# RemoveNonAscii
#------------------------------------------------------------------------------
# Echo du F9P : b5 62 02 34 0c 00 01 00 
#               00 00 a1 00 00 0a 35 04   
#               00 00 27 fa
# [15:22:15:046] <0xb5>b<0x02>4<0x0c><break>
# [15:22:15:046] <0x01><break>
# [15:22:15:046] <break>
# [15:22:15:046] <break>
# [15:22:15:046] <0xa1><break>
# [15:22:15:046] <break>
# [15:22:15:046] 
# [15:22:15:046] 5<0x04><break>
# [15:22:15:046] <break>
# [15:22:15:046] '<0xfa>
#
# <PreSync1> <PreSync2> <msg class> <msg id> <len> <payload> <chks_A> <chks_B>
#
#  pre-sync1 : 0xb5
#  pre-sync1 : 0x62
#  msg class : 1-byte message class field follows. A class is a group of 
#              messages that are related to each other.
#     msg id : A 1-byte message ID field defines the message that is to follow
#     length : A 2-byte length field follows. The length is defined as being that 
#              of the payload only. It does not include the preamble, message class, 
#              message ID, length, or UBX checksum fields. 
#              The number format of the length field is an unsigned little-endian 
#              16-bit integer (a "U2" in UBX data types).
#    payload : The payload field contains a variable number (= length) of bytes
#     chks_A : The two 1-byte CK_A and CK_B fields hold a 16-bit checksum whose calculation 
#              is defined in UBX checksum section. This concludes the frame.
#     chks_B : The two 1-byte CK_A and CK_B fields hold a 16-bit checksum whose calculation 
#              is defined in UBX checksum section. This concludes the frame.
#

def RemoveNonAscii(text):
    #clean_text = remove_non_ascii(buffer)
    #print(clean_text)

    cl=""
    nb=0
    for i in text:
        if ((i > 30) and (i<128)):
            cl=cl+chr(i)
        else:
            nb+=1
    return cl, nb

#==============================================================================
# wifi_SetConfigFile
#------------------------------------------------------------------------------
#
def wifi_SetConfigFile(ssid,pwd,ndx):
    Trace(trc_IN, LOG_DBG, sys._getframe().f_code.co_name)

    config_lines = [
        "ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev",
        "update_config=1",
        "country=FR"
        ]
    _ssid = ssid.split(",")
    _pwd  = pwd.split(",")
    
    if (ndx==0):
        ndxMin=0
        ndxMax=len(_ssid)
    else:
        ndxMin=ndx
        ndxMax=ndx+1
        
    for i in range(ndxMin,ndxMax):
        Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name, format("SSID:%s PWD:%s" % (_ssid[i],_pwd[i])))
        mpass=crypt_Decode(sys_GetMACaddress("eth0"),_pwd[i])
        config_lines = config_lines + [
                            'network={',
                            '\tssid="{}"'.format(_ssid[i]),
                            '\tpsk="{}"'.format(mpass),
                            '}\n'
        ]
    config = "\n".join(config_lines)
        
    #give access and writing. may have to do this manually beforehand
    os.popen("sudo chmod a+w /etc/wpa_supplicant/wpa_supplicant.conf")
    
    #writing to file
    with open("/etc/wpa_supplicant/wpa_supplicant.conf", "w") as wifi:
        wifi.write(config)
    
    l1 = "Wifi network"
    l2 = "  reset connexions"
    lcd_SetScreen(l1,l2)
    Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name, "Reset WiFi config file")
    os.popen("sudo wpa_cli -i wlan0 reconfigure")
    Trace(trc_OUT, LOG_DBG, sys._getframe().f_code.co_name)

#==============================================================================
# sys_StatsFix
#------------------------------------------------------------------------------
#
def sys_StatsFix():
    global glbFreeze
    
    Trace(trc_IN, LOG_DBG, sys._getframe().f_code.co_name)
    if (sum(statFix)>0):
        fx=1
        l1 = format("%s:%2d-%s:%2d" % (	posFix [fx+0],
                                        statFix[fx+0]/sum(statFix)*100,
                                        posFix [fx+1],
                                        statFix[fx+1]/sum(statFix)*100))
        fx=4
        l2 = format("%s:%2d-%s:%2d" % (	posFix [fx+0],
                                        statFix[fx+0]/sum(statFix)*100,
                                        posFix [fx+1],
                                        statFix[fx+1]/sum(statFix)*100))
    else:
        fx=1
        l1 = format("%s:__-%s:__"  % (	posFix [fx+0],
                                        posFix [fx+1]
                                        ))
        fx=4
        l2 = format("%s:__-%s:__"  % (	posFix [fx+0],
                                        posFix [fx+1],
                                        ))
    lcd_SetScreen(l1,l2)
    glbFreeze = iniLcdFreeze
    Trace(trc_MSG, LOG_WRN, sys._getframe().f_code.co_name, f"[{l1}] / [{l2}]")
    Trace(trc_OUT, LOG_DBG, sys._getframe().f_code.co_name)
    
#==============================================================================
# sys_NtpUpdate
#------------------------------------------------------------------------------
#
def sys_NtpUpdate():
    Trace(trc_IN, LOG_DBG, sys._getframe().f_code.co_name)
    l1 = "Sync NTP started"
    l2 = datetime.now().strftime("%H:%M:%S")
    lcd_SetScreen(l1,l2)
    Trace(trc_MSG, LOG_WRN, sys._getframe().f_code.co_name, f"[{l1}] / [{l2}]")
    
    #os.system("ssudo systemctl restart ntp")
    os.system("sudo service ntp stop")
    os.system("sudo ntpd -gq")
    os.system("sudo service ntp start")

    
    time.sleep(2)
    l1 = "Synchro NTP done"
    l2 = datetime.now().strftime("%H:%M:%S")
    lcd_SetScreen(l1,l2)    
    Trace(trc_MSG, LOG_WRN, sys._getframe().f_code.co_name, f"[{l1}] / [{l2}]")
    Trace(trc_OUT, LOG_DBG, sys._getframe().f_code.co_name)
    
#==============================================================================
# nmea_CheckTrameValidity
#------------------------------------------------------------------------------
#
def nmea_CheckTrameValidity(buffer):
    global glbUsbTxCounter,TramesNMEA,glbRecord
    
    Trace(trc_IN, LOG_DBG, sys._getframe().f_code.co_name)
    nonAsc=0
    try:

        # Vidage du buffer port serie
        #
        trmOk=False
        
        nmeaData,nonAsc = RemoveNonAscii(buffer)
        if (nonAsc>0):
            Trace(trc_MSG, LOG_DBG, sys._getframe().f_code.co_name, format("FIX (%d bytes non ascii) [%s]" % (nonAsc,nmeaData)))
        
        glbUsbTxCounter += len(nmeaData)
        if (nmeaData.find("$")>0):

            # Si ca ne commence pas par $, mais qu'il y a un, se recaler
            #
            nmeaData=nmeaData[nmeaData.find("$"):]
            Trace(trc_MSG, LOG_DBG, sys._getframe().f_code.co_name, format("FIX (commence pas par $ mais $ trouvé, recalage) [%s] " % (nmeaData)))
        elif (nmeaData.find("$")==0):

            # On commence bien par $, verifier si il y en a plusieurs. Si oui,
            # se recaler sur le deuxieme
            #
            if (nmeaData.count("$")>1):
                nmeaData=nmeaData[nmeaData[1:].find("$"):]
                Trace(trc_MSG, LOG_DBG, sys._getframe().f_code.co_name, "FIX (multiples $) [%s]" % (nmeaData))
        else:

            # Pas de $, ne pas traiter
            #
            Trace(trc_MSG, LOG_DBG, sys._getframe().f_code.co_name, "Trame sans $")
        
        if (nmeaData.find("$")==0) and (nmeaData[len(nmeaData)-3:len(nmeaData)-2]=="*"):
            
            # Ne traiter que si CKS ok et "*" en bout de ligne
            #
            if (nmea_CalculChecksun(nmeaData)):
                trmOk=True
              
                code = nmeaData.split(",")[0]
                if (code[3:] == "GGA"):
                    if len(nmeaData) > 50:  #RMC vide=25c
                        glbRecord=True
                    else:
                        glbRecord=False
                    nmea_DecodeGGA(nmeaData)
                elif (code[3:] == "GSA"):
                    nmea_DecodeGSA(nmeaData)
                elif (code[3:] == "RMC"):
                    nmea_DecodeRMC(nmeaData)
                    
                if glbRecord:
                    dt = datetime.now().strftime("%H:%M:%S")
                    TramesNMEA.append(dt+";"+nmeaData)                      
            else:
                Trace(trc_MSG, LOG_ERR, sys._getframe().f_code.co_name, "Erreur de checksum - %s non parsée" % (code[3:]))
    except Exception as e:
        Trace(trc_MSG, LOG_CRT, sys._getframe().f_code.co_name, repr(e))
    Trace(trc_OUT, LOG_DBG, sys._getframe().f_code.co_name)
    return(trmOk)
    
#==============================================================================
# rtcm_ClairMessage
#------------------------------------------------------------------------------
#  
def rtcm_ClairMessage(codeMsg):
    clairMsg = "Message inconnu"
    for j in range(0,len(rtcmMsgList)):
        if not (rtcmMsgList[j].find(str(codeMsg)) >= 0):
            continue
        clairMsg=rtcmMsgList[j]
        break
    return clairMsg

#==============================================================================
# rtcm_Decode
#------------------------------------------------------------------------------
#  Ex D3 01 08 43 50 00 3B 71 87 82 00 00 21 A1 85 22 
#     80 00 00 00 20 00 40 00 5F F5 FE 9C A8 8C 8C A8
#       ->len = 0x108 (264)
#       ->Num = 0x435 (1077)
#            | preamble | rsv    |  length   |  Msg      |    data message    |  parity  |
#            +----------+--------+-----------+-----------+--------------------+----------+
#            |<- 0xD3 ->|<- 6b ->|<-- 10 --->|<-- 16 --->|<--- length x 8 --->|<-- 24 -->|
#     ofs=    b+0        b+1      b+2         b=+3        b=+4                 b=3+len
#
def rtcm_Decode(binFic):
    global glbTcpRtcm
    global glbRtcmMsg
    Done = False
    
    Trace(trc_IN, LOG_DBG, sys._getframe().f_code.co_name)
    LN_ENCAPSULATION=1+2+3
    nbMsg=0
    bNdx=0
    fIn  = open(iniRootPath + "/datas/base-dump.bin", "rb")
    fOut = open(binFic.split(".")[0]+".rtcm","w")
    glbTcpRtcm = fIn.read()
    msgTotal = [0] * 200
    while(not Done):
        try:
            if (bNdx> (len(glbTcpRtcm)-512)):
                Done=True
            if (len(glbTcpRtcm)<20):

                # Pas assez d'octets recus, attente de la suite
                #
                fOut.write (f"\nPas assez de données ({len(glbTcpRtcm)} - exit)")
                Done=True
            else:
                if (glbTcpRtcm[bNdx] == 0xD3):
                    msgLen = (((glbTcpRtcm[bNdx+1] * 0x0100) + glbTcpRtcm[bNdx+2]) & 0x3FFF)
                    msgNum=((glbTcpRtcm[bNdx+3]* 0x0100)+glbTcpRtcm[bNdx+4])
                    msgNum=msgNum >> 4
                    if ((msgLen<1024) and (msgNum>1000) and (msgNum<1200)):
                        if ((len(glbTcpRtcm)-bNdx) > msgLen+LN_ENCAPSULATION):
    
                            # On a recu la trame complete -> traiter. Sinon, attendre
                            #
                            clairMsg = rtcm_ClairMessage(msgNum)
                            if ((msgNum>=1000) and (msgNum<4500)):
                                ln=format("\n0x%4.4X> MSG [%d/%X) LEN [%d/%X] %s" % (bNdx,msgNum,msgNum,msgLen,msgLen,clairMsg))
                                fOut.write (ln)
                                ln="\n            "
                                for k in range(0, msgLen + LN_ENCAPSULATION):
                                    ln=ln+format("%2.2X." % glbTcpRtcm[bNdx+k])
                                fOut.write (ln)
                                bNdx+=msgLen+LN_ENCAPSULATION
                                nbMsg+=1
                                msgTotal[msgNum-1000]+=1
                                #Trace(trc_OUT, LOG_INF, sys._getframe().f_code.co_name,ln)
                        else:
                            fOut.write ( f"\nTrame non complete: Attendus={msgLen} Dispos={len(glbTcpRtcm)} - exit)")
                            Done=True
                    else:
                        fOut.write ( f"\nMSG non valide - code={msgNum} len={msgLen}")
                        bNdx+=1                       
                else:
                    fOut.write ( format("\n%.04X: byte %2.2X-%d ignoré" %(bNdx,glbTcpRtcm[bNdx],glbTcpRtcm[bNdx])))
                    bNdx+=1
                ln=format("%4.4X-%d trames" %(bNdx,nbMsg))
                lcd.setCursor(0, 1)
                lcd.printout(ln)
            #break
        except Exception as e:
            fOut.write (repr(e)) 
            Trace(trc_MSG, LOG_CRT, sys._getframe().f_code.co_name, repr(e))
    for i in range(0,len(msgTotal)):
        if (msgTotal[i]>0):
            Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name, format("[%d] %5d messages (%s)" % (i+1000,msgTotal[i],rtcm_ClairMessage(i+1000))))
    Trace(trc_OUT, LOG_INF, sys._getframe().f_code.co_name, format("[%d] messages vidés" % (nbMsg)))
    fOut.close()
    fIn.close()
    return nbMsg

#==============================================================================
# MAIN
#------------------------------------------------------------------------------
#

glbHDop = 0.0
sys_ArchiveFile("datas/debug",10)

logging.basicConfig(
     filename="datas/debug.jrn",
     level=logging.WARNING, 
     filemode="w", 
     format= "%(asctime)s.%(msecs)03d %(levelname)8s: %(message)s",
     datefmt="%H:%M:%S"
)

ini_Read()

logging.getLogger().setLevel(logging.WARNING)
logging.getLogger().setLevel(iniLogging)

lcd_Init()
lcd_SetColor("INIT")
Trace(trc_IN, LOG_DBG, sys._getframe().f_code.co_name)
jrn = datetime.now()
Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name, "START: " + str(jrn))

lcd.clear()
lcd_CustomChar()

wifi_SetConfigFile(iniWifiSSID,iniWifiPwd,iniWifiIndex)

glbInitRTKok=False

l1 = "Rover RTK v" + _version
l2 = "Initialisation..."
lcd_SetScreen(l1,l2)
jrn = format("> Main: [%s] [%s]" % (l1, l2))
Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name, jrn)

Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name,os.getcwd())
glbNetworkOk=False
time.sleep(2)

dt = datetime.now().strftime("%Y%m%d_%Hh%M")
nmeaFileCtr = 0
nmeaFileBase = dt
nmeaPath = format("%s/nmea/nmea-%s.%03d" % (iniRootPath, nmeaFileBase, nmeaFileCtr))
loopCtr = 0
nmeaData = ""
screenCtr=0
tmDeb = 0
causeSortie="??"

while (not glbMainExit):
    loopCtr = loopCtr + 1
    if ((loopCtr % 40) == 0):
        if tglScreen:
            tglScreen = False
        else:
            tglScreen = True

    #---------------------------------------------------------------------
    # Tous les 20 tours, essayer de d'etablir la connexion si internet est dispo
    #
    if (not glbInitRTKok):
        if ((loopCtr % 20) == 0):
            glbNetworkOk = rtk_Ping(iniCstIP)
            if (glbNetworkOk):
                glbErrInfos["ntw"]=False
                lcd_SetColor("INIT")
                sys_NtpUpdate()
                Trace(trc_MSG, LOG_ERR, sys._getframe().f_code.co_name,"Reseau disponible - Tentative de connexion")
                rtk_GetBaseList()
                
                # Si la base indiquée dans le INI est valide et presente dans la table
                # CASTER, se connecter à elle. Sinon, se connecter sur la plus proche
                #
                if (not (rtk_IsKnownBase(iniCstPoint))):
                    iniCstPoint = rtk_GetNearestBase(iniCstPoint)
   
                rtk_GetBaseInfos(iniCstPoint)
                jrn = format("Point [%s] Index [%d] Dist [%f] GGA [%d]" % (basePoint, baseIndex, baseDist, 1))
                Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name,jrn)

                threadTCP = threading.Thread(target=rtk_BaseConnect, args=(iniCstPoint,))
                threadTCP.start()
                glbInitRTKok = True

                sys_WifiStatus()

            else:
                glbErrInfos["ntw"]=True
                glbInitRTKok = False
                Trace(trc_MSG, LOG_ERR, sys._getframe().f_code.co_name,"Reseau non disponible")
                lcd_SetColor("ERR")
                
        # Toutes les minutes ennviron, forcer une reconfiguration réseau
        #
        if ((loopCtr % (60*10)) == 0):
            os.popen("sudo wpa_cli -i wlan0 reconfigure")
            Trace(trc_MSG, LOG_ERR, sys._getframe().f_code.co_name,"Forcage reconfig réseau (sudo wpa_cli -i wlan0 reconfigure)")
            
    #---------------------------------------------------------------------
    # Detection et ouverture du port serie si non fait
    #
    if not serOpen:
        try:
            serPath = sys_ListUsbPort()
            if (len(serPath) > 0):
                Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name,"Essai ouverture [" + serPath + "]")
                serPort = serial.Serial(port=serPath,
                                        baudrate=115200,
                                        parity=serial.PARITY_NONE,
                                        stopbits=serial.STOPBITS_ONE,
                                        bytesize=serial.EIGHTBITS,
                                        timeout=0.1
                                        )
                serPort.flushInput()
                serOpen = True
                Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name,"serOpen SUCCESS")
                
                #threadUSB = threading.Thread(target=nmea_ReadUsb, args=(serPort,))
                #threadUSB.start()
                #glbInitRTKok = True                
            else:
                l1 = "*** ERROR ***"
                l2 = "No RTK receiver"
                lcd_SetScreen(l1,l2)
                Trace(trc_MSG, LOG_ERR, sys._getframe().f_code.co_name, f"> Main: [{l1}] / [{l2}]")
        except Exception as e:
            Trace(trc_MSG, LOG_CRT, sys._getframe().f_code.co_name, repr(e))
            serOpen = False

    #---------------------------------------------------------------------
    # Si le port serie est bien ouvert, recuperer toutes les trames NMEA
    # jusqu'à ce que la lecture retourne 0 octets
    # (avant dans un thread, ramené en ligne car soucis dans un thread)
    #
    if serOpen:
        nln=1
        while True:
            buffer = serPort.readline().strip()
            Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name,format("RxUSB-%d: [%s]" % (nln,buffer)))
            nln+=1
            nmea_CheckTrameValidity(buffer)
            if len(buffer)==0:
                break
        
    #---------------------------------------------------------------------
    # Si on a recu des données TCP, les reemettre sans traitement vers le F9P
    #
    if (len(glbTcpBuffer) > 0):
        # glbUSBcounter=glbUSBcounter+len(glbTcpBuffer)
        glbUsbTxCounter = glbUsbTxCounter + serPort.write(glbTcpBuffer)
        Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name,format("Tx: TCP->USB (renvoi buffer TCP vers F9P : [%d] bytes,  Total [%d ko])" % (len(glbTcpBuffer), int(glbUsbTxCounter / 1024))))
        #rtcm_Decode()
        glbTcpBuffer = []

    #---------------------------------------------------------------------
    # Detection si bouton presse
    #
    lcd_key = lcd_ReadButton()
    if (lcd_key == btnRIGHT):

        # RIGHT=Infos WiFi
        #
        sys_WifiStatus()
    elif (lcd_key == btnLEFT):

        # LEFT=bande passante consommée
        #
        sys_BwUsed()
    elif (lcd_key == btnUP):

        # UP=infos system
        #
        sys_GetOsInfos()
    elif (lcd_key == btnDOWN):

        # DOWN=affichage adresses IP wifi et eth
        #
        sys_GetIPaddress()
    elif (lcd_key == btnUP+btnDOWN):

        # UP+DOWN=Shutdown
        #
        Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name, "up+down")
        glbMainExit = True
        glbTcpExit = True
        causeSortie="UD"
        os.system("shutdown -h +1")
    elif (lcd_key == btnRIGHT+btnLEFT):

        # LEFT+RIGHT=Export USB
        #
        sys_ExportOnUSB("/home/blueb/RoverRTK/","/media/blueb/usb-backup/bkp-python/")
    elif (lcd_key == btnSELECT):

        # RIGHT=Quitter l'appli
        #
        causeSortie="BT"
        glbTcpExit = True
        glbMainExit = True
        
    #---------------------------------------------------------------------
    # Symbole d'activite clignotant, different si enregisrement actif ou pas
    #
    if (loopCtr % 2) == 0:
        lcd.setCursor(15, 1)
        if tglLive:
            tglLive = False
            GPIO.output(21, GPIO.HIGH)
            lcd.write(0)
        else:
            tglLive = True
            GPIO.output(21, GPIO.LOW)
            if glbRecord:
                lcd.write(1)
            else:
                lcd.write(2)

    #---------------------------------------------------------------------
    # Gestion de l'affichage LCD avec les infos GNSS
    #
    if (glbFreeze == 0):
        if (loopCtr % 20) == 0:
            try:
                # Alternance ecran LCD
                #
                if (tglScreen==True):
                    tglScreen = False
                    l1 = format("%08.5f %07.5f.." % (glbLat, glbLon))
                    l2 = format("%0.3dm HDp:%06.3f " % (glbHeight, glbHDop))
                else:
                    tglScreen = True
                    l1 = format("Sat:%02d Fx:%s " % (sum(glbSatCtr), posFix[glbFix]))
                    l2 = format("Base:%s %dkm" % (basePoint, int(baseDist)))
                lcd_SetScreen(l1,l2)
                Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name, f"[{l1}] / [{l2}]")
            except Exception as e:
                Trace(trc_MSG, LOG_CRT, sys._getframe().f_code.co_name, repr(e))
    else:
        glbFreeze = glbFreeze - 1
        
    #---------------------------------------------------------------------
    # Si tableau de trames plein, ecrire dans un fichier date
    #
    if len(TramesNMEA) > iniSzMaxNMEA:
        Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name,"Taille NMEA atteinte - sauvegarde "+nmeaPath)
        if logToggle:
            if nmeaFileCtr==0:
                dt = datetime.now().strftime("%Y%m%d_%Hh%M")
                nmeaFileBase = dt
                nmeaPath = format("%s/nmea/nmea-%s.%03d" % (iniRootPath, nmeaFileBase, nmeaFileCtr))
            
            with open(nmeaPath, "w") as nmeaHdlr:
                for item in TramesNMEA:
                    nmeaHdlr.write("\n" + item)
            nmeaHdlr.close()

            nmeaFileCtr = nmeaFileCtr + 1
            nmeaPath = format("%s/nmea/nmea-%s.%03d" % (iniRootPath, nmeaFileBase, nmeaFileCtr))
        TramesNMEA = []
        ini_Write()
        
    #---------------------------------------------------------------------
    # Infos periodiques
    #
    if ((loopCtr % 81) == 0):
        # Ne pas utiliser l'heure pdt 10mn, le temps de syncrho de l'OS sur NTP
        if (glbNetworkOk) and (loopCtr>1*10*60):
            if (tmDeb==0):
                tmDeb = time.time()
                Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name,"TDeb initialisé - " + str(datetime.now()))

            tmFin = time.time()
            if (int((tmFin - tmDeb) / 60) > iniDureeMax):
                Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name,"Durée de fonctionnement max atteinte (" + str(iniDureeMax) + " mn)")
                Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name,format("Deb=%f - Act=%f - durée=%f" % (tmDeb,tmFin,tmFin-tmDeb)))
                glbMainExit = True
                causeSortie="TM"
    if ((loopCtr % 40) == 0):
        if screenCtr==0:
            sys_GetOsInfos()
            screenCtr += 1
        elif screenCtr==1:
            sys_GetIPaddress()
            screenCtr += 1
        elif screenCtr==2:
            sys_WifiStatus()
            screenCtr += 1
        elif screenCtr==3:
            sys_BwUsed()
            screenCtr += 1
        elif screenCtr==4:
            sys_SatInfos()
            screenCtr += 1
        elif screenCtr>=5:
            sys_StatsFix()
            screenCtr = 0
            
    #---------------------------------------------------------------------
    # Fin de boucle
    #
    if (os.path.exists("stop")):
        os.remove("stop")
        causeSortie="ST"
        glbMainExit=True
    if glbMainExit:
        Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name," Sortie demandee")

    # print("> Main: ["+l1+"] ["+l2+"]")
    time.sleep(0.1)

#==============================================================================
#                                   Fin
#==============================================================================

lcd_SetColor("INIT")

l1 = "Decodage RTCM.."
l2 = "   ...debut...."
lcd_SetScreen(l1,l2)

# Export en format text du buffer binaire RTCM
#
rtcm_Decode(iniRootPath + "/datas/base-dump.bin")
            
# Sauvegarde du reliquat de log si active
#
if logToggle:
    with open(nmeaPath, "w") as nmeaHdlr:
        for item in TramesNMEA:
            nmeaHdlr.write("\n" + item)
    nmeaHdlr.close()

# Conversion automatique des NMEA en GPX
#
gpx_Convert()

# Ajout des stats de FIX
#
DisplayFixStat(1)
    
tmFin = time.time()
Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name,"Durée=" + str(int((tmFin - tmDeb) / 60)) + "mn")
Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name,"END: " + str(datetime.now()))

l2 = "Finished        "
lcd_SetScreen(l1,l2)

sys_ExportOnUSB("/home/blueb/RoverRTK/","/media/blueb/usb-backup/bkp-python/")

l1 = "Rover RTK v" + _version
l2 = "Termine...["+causeSortie+"]"
lcd_SetScreen(l1,l2)
Trace(trc_MSG, LOG_INF, sys._getframe().f_code.co_name, f"[{l1}] / [{l2}]")

# Normalement inutile, sécurité..
#
logging.shutdown()