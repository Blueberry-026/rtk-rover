#
# Parseur de logs NMEA generes par NetBox
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
# /============================================================================
# | HISTORIQUE - Script de decodage de trames NMEA
# |----------------------------------------------------------------------------
# | Changelog : [*] Bug fixes
# |             [+] New function
# |             [-] Update
# |-------+------------+-------------------------------------------------------
# | VERS  | DATE       | EVOLUTIONS
# |-------+------------+-------------------------------------------------------
# |       |            |
# | v0.0x | 28/12/2023 | + Premiere version fonctionnelle
# |       |            |
# \----------------------------------------------------------------------------
#
_version = 0.01

import glob
import sys
import os
import argparse
import platform
from math import sin, cos, sqrt, atan2
from datetime import datetime

nmeaFicCurrent=0
nmeaFicTotal=0

ctrMLF, ctrREP , ctrVID, ctrLINE, ctrVTG, ctrGSV, ctrGSA, ctrINC, ctrTXT=0,0,0,0,0,0,0,0,0
ctrCKO, ctrCKA , ctrCKF=0,0,0
ctrRMC, ctrRMCv, ctrRMCnv=0,0,0
ctrGGA, ctrGGAf, ctrGGAnf=0,0,0
ctrGLL, ctrGLLv, ctrGLLnv=0,0,0
ctrMLF, ctrVID=0,0
ctrRMC, ctrRMCv, ctrRMCnv=0,0,0
ctrPOS, ctrPOSv, ctrPOSnv  =0,0,0
gpxLat, gpxLon, gpxAlt, gpxVit, gpxDop, gpxSat, gpxDat, gpxTim, gpxTimPrec=0,0,0,0,0,0,0,0,0
gpxFix=""
strTXT, strSAT, strGPS, strGAL, strINC="","","","",""
_lat = _lon = _alt = _vit = _dop = 0.0
_sat = 0
_val = ""
_tim = "..:..:.."
_dat = "....-..-.."
_valid = " ? "
FMT_T = "%H:%M:%S"
FMT_D = "%Y-%m-%d"
tmFirst=datetime.strptime("01:01:01", FMT_T)
tmLast=datetime.strptime("01:01:02", FMT_T)
dtFirst=datetime.strptime("2000-01-01", FMT_D)
dtLast=datetime.strptime("2000-01-01", FMT_D)
dDST, dDAT, dLAT, dLON, dVIT=0,0,0,0,0
totDST, totSEC=0,0
#ficCsv=0

# DecimalDegrees
# -----------------------------------------------------------------------------
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
        dg  = int(pos)
        dec = (pos % 1) * 100
        mn  = (dec * 100) / 60
        dm  = dg + (mn / 100)
        if (dm < 4):
            print("\n   -> Position [%f] incorrecte ***" % pos)
    else:
        dm=0.0
    return dm



# =============================================================================
# Main
# -----------------------------------------------------------------------------
# Si le programme est appele sans arguments, il traite arbitrairement le
# fichier "\default.nmea" dans le repertoire "logBaseRep"
# Si un argument est passe (par ex *.*, log*.nmea, test.nmea...) le ou les
# fichiers concernes seront traites
#
#if __name__ == '__main__':
def init():
	FMT_T = "%H:%M:%S"
	FMT_D = "%Y-%m-%d"

	defaultBaseRep = os.getcwd() + "/nmea"
	codesGPX = ['header', 'point', 'footer']

	nmeaDir = os.getcwd() + "/nmea"
	nmeaFiles = "*.nmea"

	lstDir = [nmeaDir]
	listdirs(nmeaDir, lstDir, 0)
	for workDir in lstDir:
		if (platform.system()=='Linux'):
			fileMask = workDir + "/" + nmeaFiles
		else:
			fileMask = workDir + "\\" + nmeaFiles
		if (len(glob.glob(fileMask)) == 0):
			print("===== [Parser NMEA v%3.2f] =====================================" % _version)
			print("   ERREUR: Aucun fichier trouve a l'emplacement designe [%s]" % workDir)
			continue
		else:
			# Initialisation du fichier SYNTHESE.CSV qui est commun a tous les fichiers
			# qui seront traites dans ce batch
			#
			try:
				if (platform.system() == 'Linux'):
					 outFileName = os.getcwd() + "/stats/synthese.csv"
				else:
					 outFileName = workDir + "\\synthese.csv"
				ficCsv = open(outFileName, 'w')
				FillCsv(ficCsv, 'header')
			except:
				print("\n   *** Impossible d'acceder au fichier [%s]" % outFileName)
				print("     *** Traitement annule")
				exit(0)
			nmeaFicTotal=len(glob.glob(fileMask))
			nmeaFicCurrent=0

		for fName in glob.glob(fileMask):
			# Initialialisation des compteurs (pour stats et verif des trames non conformes)
			#
			ctrLINE = ctrREP = ctrVTG = ctrGSV = ctrGSA = 0
			ctrINC = ctrCKO = ctrCKA = ctrCKF = ctrMLF = ctrVID = ctrTXT = 0
			ctrRMC = ctrRMCv = ctrRMCnv = 0
			ctrGGA = ctrGGAf = ctrGGAnf = 0
			ctrGLL = ctrGLLv = ctrGLLnv = 0
			ctrPOS = ctrPOSv = ctrPOSnv = 0
			dDST = dNBS = dLAT = dLON = dVIT = 0
			totDST = totSEC = 0
			tmFirst = tmLast = ""
			dtFirst = dtLast = ""

			# Initialisation des champs utilises pour remplir le fichier GPX
			#
			gpxLat = gpxLon = gpxVit = gpxDop = gpxSat = 0
			gpxDat = gpxTim = gpxTimPrec = ""
			gpxAlt = 1
			strTXT = strSAT = strGPS = strGAL = strINC = ""

			_cod = "..."
			_lat = _lon = _alt = _vit = _dop = 0.0
			_sat = 0
			_val = ""
			_tim = "..:..:.."
			_dat = "....-..-.."
			_valid = " ? "

			nmeaFicCurrent+=1
			AnalyzeNmeaFile(fName)

		ficCsv.close()
