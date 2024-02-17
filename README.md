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

Principe:
![principe](http://blueb.fr/RTK/docs/github/RTK-v1.png)

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

Exemple de rendu comparé d'un GPX du compteur vélo et celui du du rover:<br>
![Rendu OSM](http://blueb.fr/RTK/docs/github/CompareGPS-RTK.png)

Principe:
![principe](http://blueb.fr/RTK/docs/github/RTK-v2.png)
![principe](http://blueb.fr/RTK/docs/github/RTK-v3.png)
