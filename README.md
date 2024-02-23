# **rtk-rover**

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
[Uploading debug.0…]()

## Statistiques GNSS:
Types de trames NMEA recues:
```
+--------+---------+---------+ 
| type   | records |     %   | 
+--------+---------+---------+ 
| -----  |       0 |   0.00% | 
| GNGST  |     207 |   5.23% | 
| GNGGA  |     208 |   5.26% | 
| GNGSA  |    1038 |  26.25% | 
| GAGSV  |     401 |  10.14% | 
| GBGSV  |     424 |  10.72% | 
| GLGSV  |     713 |  18.03% | 
| GPGSV  |     549 |  13.88% | 
| GQGSV  |     207 |   5.23% | 
| GNRMC  |     208 |   5.26% | 
| GNTXT  |       0 |   0.00% | 
| GNVTG  |       0 |   0.00% | 
| GNGLL  |       0 |   0.00% | 
```
Types de fix GNSS:
```
+------+---------+---------+ 
| fix  | records |    %    | 
+------+---------+---------+ 
| NOFX |       0 |  --.--% | 
| GPS  |       0 |  --.--% | 
| DGPS |       0 |  --.--% | 
| N/A  |       0 |  --.--% | 
| RTKx |       0 |  --.--% | 
| RTKf |       0 |  --.--% | 
| INS  |       0 |  --.--% | 
```
Trames RTCM v3 recues de la base et à renvoyer vers la puce F9P:
```
+------+---------+--------+--------+--------------------- 
| code | msg/min |    %   | nombre | clair message 
+------+---------+--------+--------+--------------------- 
| 1004 |   59.23 | 11.09% |    153 | Extended L1&L2 GPS RTK Observables for GPS RTK Use, the main msg 
| 1005 |   05.81 | 01.09% |     15 | Stationary RTK Reference Station ARP  
| 1006 |   01.94 | 00.36% |      5 | Stationary RTK Reference Station ARP plus the Antenna Height  
| 1008 |   05.81 | 01.09% |     15 | Antenna Descriptor and Serial Number 
| 1012 |   59.61 | 11.16% |    154 | Extended L1&L2 GLONASS RTK Observables, the other main msg  
| 1019 |   18.19 | 03.41% |     47 | GPS Broadcast Ephemeris (orbits)  
| 1020 |   27.87 | 05.22% |     72 | GLONASS Broadcast Ephemeris (orbits)  
| 1033 |   05.81 | 01.09% |     15 | Receiver and Antenna Descriptors 
| 1042 |   24.00 | 04.49% |     62 | BDS Satellite Ephemeris Data 
| 1046 |   25.94 | 04.86% |     67 | Galileo I/NAV Satellite Ephemeris Data 
| 1077 |   59.61 | 11.16% |    154 | GPS MSM7 
| 1087 |   59.61 | 11.16% |    154 | GLONASS MSM7 
| 1097 |   59.61 | 11.16% |    154 | Galileo MSM7 
| 1107 |   59.61 | 11.16% |    154 | SBAS MSM7 
| 1127 |   59.61 | 11.16% |    154 | BeiDou MSM7 
| 1230 |   01.94 | 00.36% |      5 | GLONASS L1 and L2 Code-Phase Biases 
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
