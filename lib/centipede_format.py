baseRecord = [  # STR;ASSAS;Ruan;RTCM3;1004,1005,1006;2;GLO+GAL+SBS+BDS+GPS;NONE;FRA;48.096;1.894;0;0;NTRIP RTKBase U-blox_ZED-F9P 2.4.2 1.13;none;N;N;15200;CentipedeRTK
    "typ",      # STR (the only acceptable string)
    "mpt",      # Caster mountpoint
    "iden",     # Source identifier, e.g. name of city next to source location
    "fmt",      # Data format RTCM, RAW, etc.
    "dts",      # format-details
    "car",      # Data stream contains carrier phase information
                #	0 = No (e.g. for DGPS)
                #	1 = Yes, L1 (e.g. for RTK)
                #	2 = Yes, L1&L2 (e.g. for RTK)
    "nav",      # Navigation system(s)
                #	GPS
                #	GPS+GLONASS
                #	GPS+EGNOS
    "netw",     # Network
                # 	EUREF
                # 	IGS
                # 	IGLOS
                # 	SAPOS
                # 	GREF
                # 	Misc
    "ctry",     # Country. Three character country code in ISO 3 Characters 3166
    "lat",      # Latitude. Floating point number, two digits after decimal point
    "lon",      # Longitude. Floating point number, two digits after decimal point
    "nmea",     # Necessity for Client to send NMEA message with approximate position to Caster
                #	0 = Client must not send NMEA message with approximate position to Caster
                #	1 = Client must send NMEA GGA message with approximate position to Caster
    "sol",      # Stream generated from single reference station or from networked reference stations
                #	0 = Single base
                #	1 = Network
    "gen",      # Generator. Hard or software generating data stream
    "comp",     # Compression/Encryption algorithm applied. Characters, undefined length
    "aut",      # Authentication. Access protection for this particular data stream
                #	N = None
                #	B = Basic
                #	D = Digest
    "fee",      # User fee for receiving this particular data stream
                #	N = No user fee
                #	Y = Usage is charged
    "bitr",     # Bit rate of data stream, bits per second
    "misc"      # Miscellaneous information, last data field in record
]

