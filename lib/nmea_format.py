ggaRecord = [   # $GNGGA,122220.00,4542.4812848,N,00451.2467166,E,2,12, 0.60,188.552, M,47.358,M,,0136*4A
    "id",       # 0 Message ID $GPGGA
    "time",     # 1 UTC of position fix
    "lat",      # 2 Latitude
    "dlat",     # 3 Direction of latitude:
                #		N: North
                #		S: South
    "lon",      # 4 Longitude
    "dlon",     # 5 Direction of longitude:
                #     E: East
                #     W: West
    "qlty",     # 6 GPS Quality indicator:
                #     0: Fix not valid
                #     1: GPS fix
                #     2: Differential GPS fix (DGNSS), SBAS, OmniSTAR VBS, Beacon, RTX in GVBS mode
                #     3: Not applicable
                #     4: RTK Fixed, xFill
                #     5: RTK Float, OmniSTAR XP/HP, Location RTK, RTX
                #     6: INS Dead reckoning
    "nsat",     # 7 Number of SVs in use, range from 00 through to 24+
    "hdop",     # 8 HDOP level precision factor
    "hgth",     # 9 Orthometric height (MSL reference)
    "hmea",     # 10 M: unit of measure for orthometric height is meters
    "geoid",    # 11 Geoid separation
    "gmea",     # 12 M: geoid separation measured in meters
    "diff",     # 13 Age of differential GPS data record, Type 1 or Type 9.
                #		Null field when DGPS is not used.
    "ref",      # 14 Reference station ID, range 0000 to 4095. A null field
                #		when any reference station ID is selected and no
                #		corrections are received. See table below for a
                #		description of the field values.
    "cks"       # Checksum value, always begins with *
    ]
gsaRecord = [   # $GNGSA,A,3,30,25,34,02,36,11,,,,,,,1.05,0.62,0.85,3*0A
    "id",       # 0 Message ID $GPGGA
    "selec",    # Selection mode:
                #     M=Manual, forced to operate in 2D or 3D, 
                #     A=Automatic, 2D/3D
    "mode",     # Mode:
                #     1 = no fix, 
                #     2 = 2D fix, 
                #     3 = 3D fix
    "sat1",     # ID of satellite 1  used for fix
    "sat2",     # ID of satellite 2  used for fix
    "sat3",     # ID of satellite 3  used for fix
    "sat4",     # ID of satellite 4  used for fix
    "sat5",     # ID of satellite 5  used for fix
    "sat6",     # ID of satellite 6  used for fix
    "sat7",     # ID of satellite 7  used for fix
    "sat8",     # ID of satellite 8  used for fix
    "sat9",     # ID of satellite 9  used for fix
    "sat10",    # ID of satellite 10 used for fix
    "sat11",    # ID of satellite 11 used for fix
    "sat12",    # ID of satellite 12 used for fix
	"PDOP",     # 
	"HDOP",     # Horizontal Dilution of Precision, HDOP
	"VDOP",     # Vertical Dilution of Precision, HDOP
	"sysID",    # System ID (NMEA 4.11)
                #     1 = GPS L1C/A, L2CL, L2CM
                #     2 = GLONASS L1 OF, L2 OF
                #     3 = Galileo E1C, E1B, E5 bl, E5 bQ
                #     4 = BeiDou B1I D1, B1I D2, B2I D1, B2I D12
    "cks"       # Checksum value, always begins with *
    ]
gstRecord = [   # GNGST,172425.00,31,1.5,0.96,145,0.56,0.48,1.2*4B
    "id",       # 0 Message ID $GxGST
    "tim",      # hhmmss.ss UTC of position fix, hh is hours, mm is minutes, ss.ss is seconds.
    "rms",      # Total RMS standard deviation of ranges inputs to the navigation solution 
    "maj",      # Standard deviation (meters) of semi-major axis of error ellipse
    "min",      # Standard deviation (meters) of semi-minor axis of error ellipse    
    "ord",      # Orientation of semi-major axis of error ellipse (true north degrees)    
    "rla",      # Standard deviation (meters) of latitude error  
    "rlo",      # Standard deviation (meters) of longitude error
    "ral",      # Standard deviation (meters) of altitude error  
    "cks"       # Checksum value, always begins with *
    ]
rmcRecord = [   # $GNRMC,065632.00,A,4545.2617589,N,00451.9051633,E,9.588,80.10,290124,,,D,V*3F
    "id",       # 0 Message ID $GxRMC
    "tim",      # hhmmss.ss UTC of position fix, hh is hours, mm is minutes, ss.ss is seconds.
    "sts",      # Status, 
                #     A = Valid
                #     V = Warning
    "lat",      # Latitude, dd is degrees. mm.mm is minutes..
    "ns",       # N or S
    "lon",      # Longitude, ddd is degrees. mm.mm is minutes.
    "ew",       # E or W
    "spd",      # Speed over ground, knots
    "trk",      # Track made good, degrees true
    "dat",      # Date, ddmmyy
    "mag",      # Magnetic Variation, degrees
    "ew2",      # E or W
    "faa",      # FAA mode indicator
    "nav",      # Nav Status (NMEA 4.1 and later) 
                #     A = autonomous
                #     D = differential
                #     E = estimated
                #     M = manual input mode 
                #     N = not valid 
                #     S = simulator 
                #     V = valid
    "cks"       # Checksum value, always begins with *
    ]