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
Types de trames NMEA recues (sur 1h):
```
+--------+---------+---------+
| type   | records |     %   |
+--------+---------+---------+
| -----  |       0 |   0.00% |
| GNGST  |    3693 |   4.33% |
| GNGGA  |    3695 |   4.34% |
| GNGSA  |   18467 |  21.67% |
| GAGSV  |    9797 |  11.50% |
| GBGSV  |   15409 |  18.08% |
| GLGSV  |   10785 |  12.66% |
| GPGSV  |   15974 |  18.75% |
| GQGSV  |    3693 |   4.33% |
| GNRMC  |    3695 |   4.34% |
| GNTXT  |       0 |   0.00% |
| GNVTG  |       0 |   0.00% |
| GNGLL  |       0 |   0.00% |
```
Types de fix GNSS (sur 1h):
```
+------+---------+---------+
| fix  | records |    %    |
+------+---------+---------+
| NOFX |       0 |   0.00% |
| GPS  |       0 |   0.00% |
| DGPS |      14 |   0.38% |
| N/A  |       0 |   0.00% |
| RTKx |       0 |   0.00% |
| RTKf |    3681 |  99.62% |
| INS  |       0 |   0.00% |
```
Trames RTCM v3 recues de la base et à renvoyer vers la puce F9P (sur 1h):
```
+------+---------+--------+--------+---------------------
| code | msg/min |    %   | nombre | clair message
+------+---------+--------+--------+---------------------
| 1004 |   58.36 | 11.08% |    142 | Extended L1&L2 GPS RTK Observables for GPS RTK Use, the main msg 
| 1005 |   05.75 | 01.09% |     14 | Stationary RTK Reference Station ARP 
| 1006 |   02.05 | 00.39% |      5 | Stationary RTK Reference Station ARP plus the Antenna Height 
| 1008 |   05.75 | 01.09% |     14 | Antenna Descriptor and Serial Number
| 1012 |   58.77 | 11.15% |    143 | Extended L1&L2 GLONASS RTK Observables, the other main msg 
| 1019 |   24.66 | 04.68% |     60 | GPS Broadcast Ephemeris (orbits) 
| 1020 |   25.48 | 04.84% |     62 | GLONASS Broadcast Ephemeris (orbits) 
| 1033 |   05.75 | 01.09% |     14 | Receiver and Antenna Descriptors
| 1042 |   22.60 | 04.29% |     55 | BDS Satellite Ephemeris Data
| 1046 |   22.19 | 04.21% |     54 | Galileo I/NAV Satellite Ephemeris Data
| 1077 |   58.77 | 11.15% |    143 | GPS MSM7
| 1087 |   58.77 | 11.15% |    143 | GLONASS MSM7
| 1097 |   58.77 | 11.15% |    143 | Galileo MSM7
| 1107 |   58.77 | 11.15% |    143 | SBAS MSM7
| 1127 |   58.36 | 11.08% |    142 | BeiDou MSM7
| 1230 |   02.05 | 00.39% |      5 | GLONASS L1 and L2 Code-Phase Biases
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
