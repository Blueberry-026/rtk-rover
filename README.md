# rtk-rover

* Recepteur GNSS (https://www.ardusimple.com/product/simplertk2blite-bt-case-kit/)<br>
![Recepteur GNSS](http://blueb.fr/RTK/docs/Photos/github/reduced/antenne.jpg)
* Rasbperry Pi3B+ (https://www.gotronic.fr/art-carte-raspberry-pi-3-b-27826.htm)<br>
![Rasbperry Pi3B+](http://blueb.fr/RTK/docs/Photos/github/reduced/Pi3.jpg)
* Ecran 2x16 LCD RGB (https://www.gotronic.fr/art-afficheur-lcd-i2c-2x16-hat-dfr0514-30630.htm)<br>
![Ecran 2x16 LCD RGB](http://blueb.fr/RTK/docs/Photos/github/reduced/lcd.jpg)
* Boitier recup (20x20x7)<br>
![Boitier recup (20x20x7)](http://blueb.fr/RTK/docs/Photos/github/reduced/boitier2-A.jpg)

+Powerbank 20k<br>

# Principe:
![principe](http://blueb.fr/RTK/docs/github/RTK-v1.png)

Trames RTCM v3 recues de la base et à renvoyer vers la puce F9P:
```
09:24:29.538 INFO: * rtcm_Decode [1004]   151 messages (1004 Extended L1&L2 GPS RTK Observables for GPS RTK Use, the main msg ) 
09:24:29.539 INFO: * rtcm_Decode [1005]    15 messages (1005 Stationary RTK Reference Station ARP ) 
09:24:29.539 INFO: * rtcm_Decode [1006]     5 messages (1006 Stationary RTK Reference Station ARP plus the Antenna Height ) 
09:24:29.540 INFO: * rtcm_Decode [1008]    15 messages (1007 Antenna Descriptor (msg 1008 (X) is also commonly used)) 
09:24:29.540 INFO: * rtcm_Decode [1012]   152 messages (1012 Extended L1&L2 GLONASS RTK Observables, the other main msg ) 
09:24:29.541 INFO: * rtcm_Decode [1019]    54 messages (1019 GPS Broadcast Ephemeris (orbits) ) 
09:24:29.542 INFO: * rtcm_Decode [1020]    78 messages (1020 GLONASS Broadcast Ephemeris (orbits) ) 
09:24:29.542 INFO: * rtcm_Decode [1033]    15 messages (1033 Receiver and Antenna Descriptors) 
09:24:29.543 INFO: * rtcm_Decode [1042]    50 messages (1042 BDS Satellite Ephemeris Data) 
09:24:29.544 INFO: * rtcm_Decode [1046]    60 messages (1046 Galileo I/NAV Satellite Ephemeris Data) 
09:24:29.545 INFO: * rtcm_Decode [1077]   152 messages (1077 GPS MSM7) 
09:24:29.546 INFO: * rtcm_Decode [1087]   152 messages (1087 GLONASS MSM7) 
09:24:29.547 INFO: * rtcm_Decode [1097]   152 messages (1097 Galileo MSM7) 
09:24:29.548 INFO: * rtcm_Decode [1107]   152 messages (1107 SBAS MSM7) 
09:24:29.548 INFO: * rtcm_Decode [1127]   151 messages (1127 BeiDou MSM7) 
```
Sur le CLD, en alternance :

* [Latitude / Longitude]  -  [Altitude / HDop]
* [NbSat / Fix]  - [Base / Distance]<br>
![lcd-sat](http://blueb.fr/RTK/docs/Photos/github/reduced/lcd2-lat.jpg)
![lcd-sat](http://blueb.fr/RTK/docs/Photos/github/reduced/lcd2-fix.jpg)

* [IP Wifi] - [IP filaire]
* [Flags reseau]  -  [SSID]<br>
![lcd-ip](http://blueb.fr/RTK/docs/Photos/github/reduced/lcd2-ip.jpg)
![lcd-ip](http://blueb.fr/RTK/docs/Photos/github/reduced/lcd2-ssid.jpg)

* [Transfert USB]  - [Transfert TCT]
* [%CPU / %MEM]  -  [Temperature / Frequence]<br>
![lcd-sat](http://blueb.fr/RTK/docs/Photos/github/reduced/lcd2-debit.jpg)
![lcd-sys](http://blueb.fr/RTK/docs/Photos/github/reduced/lcd2-sys.jpg)

Après conversion des trames GGA en GPX:<br>
![Rendu OSM](http://blueb.fr/RTK/docs/Photos/github/reduced/osm.png)

# Comparaison avec GPS simple:
Exemple de rendu comparé d'un GPX du compteur vélo et celui du du rover:<br>
![Rendu OSM](http://blueb.fr/RTK/docs/github/CompareGPS-RTK.png)

# Autres setup RTK:
![principe](http://blueb.fr/RTK/docs/github/RTK-v2.png)
![principe](http://blueb.fr/RTK/docs/github/RTK-v3.png)
