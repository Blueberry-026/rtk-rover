# **Rover RTK - Localisation GNSS centimetrique**

## Constitution du rover:
Le rover est composé des éléments suivants:
* Recepteur GNSS (https://www.ardusimple.com/product/simplertk2blite-bt-case-kit/)<br>
![Recepteur GNSS](http://blueb.fr/RTK/docs/Photos/github/reduced/antenne.jpg)
* Rasbperry Pi3B+ (https://www.gotronic.fr/art-carte-raspberry-pi-3-b-27826.htm)<br>
![Rasbperry Pi3B+](http://blueb.fr/RTK/docs/Photos/github/reduced/Pi3.jpg)
* Ecran 2x16 LCD RGB (https://www.gotronic.fr/art-afficheur-lcd-i2c-2x16-hat-dfr0514-30630.htm)<br>
![Ecran 2x16 LCD RGB](http://blueb.fr/RTK/docs/Photos/github/reduced/lcd.jpg)
* Boitier recup (20x20x7)<br>
![Boitier recup (20x20x7)](http://blueb.fr/RTK/docs/Photos/github/reduced/boitier2-A.jpg)

+Powerbank 20k<br>

## Principe:
![principe](http://blueb.fr/RTK/docs/github/RTK-v1.png)

## Statistiques GNSS:
Types de trames NMEA recues (sur 1h):
```
+--------+---------+--------+ 
| NMEA   | records |    %   | 
+--------+---------+--------+ 
| -----  |       0 |  0.00% | 
| GNGST  |    3686 |  4.39% | 
| GNGGA  |    3687 |  4.39% | 
| GNGSA  |   18432 | 21.95% | 
| GAGSV  |   12474 | 14.85% | 
| GBGSV  |   11756 | 14.00% | 
| GLGSV  |   11986 | 14.27% | 
| GPGSV  |   14596 | 17.38% | 
| GQGSV  |    3686 |  4.39% | 
| GNRMC  |    3687 |  4.39% | 
| GNTXT  |       0 |  0.00% | 
| GNVTG  |       0 |  0.00% | 
| GNGLL  |       0 |  0.00% | 
```
Types de fix GNSS (sur 1h):
```
+------+---------+--------+
| FIX  | records |    %   |
+------+---------+--------+
| NOFX |       0 |  0.00% | 
| GPS  |       5 |  0.14% | 
| DGPS |      80 |  2.18% | 
| N/A  |       0 |  0.00% | 
| RTKx |     878 | 23.91% | 
| RTKf |    2709 | 73.77% | 
| INS  |       0 |  0.00% | 
```
Trames RTCM v3 recues de la base et à renvoyer vers la puce F9P (sur 1h):
```
+------+---------+--------+---------+--------------------- 
| RTCM | records |    %   | msg/min | clair message 
+------+---------+--------+---------+--------------------- 
| 1004 |    137  | 10.84% | 58.71 | Extended L1&L2 GPS RTK Observables for GPS RTK Use, the main msg  
| 1005 |     13  |  1.03% |  5.57 | Stationary RTK Reference Station ARP  
| 1006 |      5  |  0.40% |  2.14 | Stationary RTK Reference Station ARP plus the Antenna Height  
| 1008 |     13  |  1.03% |  5.57 | Antenna Descriptor and Serial Number 
| 1012 |    138  | 10.92% | 59.14 | Extended L1&L2 GLONASS RTK Observables, the other main msg  
| 1019 |     44  |  3.48% | 18.86 | GPS Broadcast Ephemeris (orbits)  
| 1020 |     81  |  6.41% | 34.71 | GLONASS Broadcast Ephemeris (orbits)  
| 1033 |     13  |  1.03% |  5.57 | Receiver and Antenna Descriptors 
| 1042 |     48  |  3.80% | 20.57 | BDS Satellite Ephemeris Data 
| 1046 |     78  |  6.17% | 33.43 | Galileo I/NAV Satellite Ephemeris Data 
| 1077 |    138  | 10.92% | 59.14 | GPS MSM7 
| 1087 |    138  | 10.92% | 59.14 | GLONASS MSM7 
| 1097 |    138  | 10.92% | 59.14 | Galileo MSM7 
| 1107 |    138  | 10.92% | 59.14 | SBAS MSM7 
| 1127 |    137  | 10.84% | 58.71 | BeiDou MSM7 
| 1230 |      5  |  0.40% |  2.14 | GLONASS L1 and L2 Code-Phase Biases 
totalRtcm [1264] rtcmDuration [140] 
```
## Interface utilisateur:
Sur le LCD, en alternance :
|         |            |
| ------------- |-------------| 
| Lancement |  |
| ![lcd-sat](http://blueb.fr/RTK/docs/github/LCDv2/LCD-INIT.jpg)     |  | 
| Synchro NTP  |  |
| ![lcd-sat](http://blueb.fr/RTK/docs/github/LCDv2/LCD-NTPIN.jpg)    | ![lcd-sys](http://blueb.fr/RTK/docs/github/LCDv2/LCD-NTPOUT.jpg)| 
| [Latitude / Longitude] - [Altitude / HDop]<br>                     | [NbSat / Fix] - [Base / Distance] |
| ![lcd-sat](http://blueb.fr/RTK/docs/github/LCDv2/LCD-LATLON.jpg)   | ![lcd-sat](http://blueb.fr/RTK/docs/github/LCDv2/LCD-BASE.jpg)  |
| [IP Wifi] - [IP filaire]                                           | [Flags réseau] - [SSID] |
| ![lcd-ip](http://blueb.fr/RTK/docs/github/LCDv2/LCD-IP.jpg)        | ![lcd-sat](http://blueb.fr/RTK/docs/github/LCDv2/LCD-SSID.jpg) |
| [Transfert USB] - [Transfert TCP/IP]                               | [%CPU / %MEM] - [Temperature / Frequence]|
| ![lcd-sat](http://blueb.fr/RTK/docs/github/LCDv2/LCD-TRAFIC.jpg)   | ![lcd-sys](http://blueb.fr/RTK/docs/github/LCDv2/LCD-CPU.jpg) |
| [NbSat GALILEO/GPS] - [NbSat BEIDOU/GLONASS]                       | |
| ![lcd-sat](http://blueb.fr/RTK/docs/github/LCDv2/LCD-CONST.jpg)    | |
| Dump RTCM                                                          | Génération GPX                                  |
| ![lcd-sat](http://blueb.fr/RTK/docs/github/LCDv2/LCD-RTCM.jpg)     | ![lcd-sys](http://blueb.fr/RTK/docs/github/LCDv2/LCD-GPX.jpg) | 
| Fin de programme                                                   |                                  |
| ![lcd-sat](http://blueb.fr/RTK/docs/github/LCDv2/LCD-END.jpg)      | | 

## Fichiers générés:
Après conversion des trames GGA en GPX:<br>
![Rendu OSM](http://blueb.fr/RTK/docs/Photos/github/reduced/osm.png)

## Comparaison avec GPS simple:
Exemple de rendu comparé d'un GPX du compteur vélo et celui du du rover: capture de 4 passages en roulant sur une ligne sol precise<br>
![Rendu OSM](http://blueb.fr/RTK/docs/github/CompareGPS-RTK.png)

## Autres setup RTK possibles:
![principe](http://blueb.fr/RTK/docs/github/RTK-v2.png)
![principe](http://blueb.fr/RTK/docs/github/RTK-v3.png)
