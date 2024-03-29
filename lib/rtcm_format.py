rtcmMsgList = [
"1001 L1-Only GPS RTK Observables (type x004 is to be preferred)",
"1002 Extended L1-Only GPS RTK Observables (type x004 is to be preferred)",
"1003 L1&L2 GPS RTK Observables (type x004 is to be preferred)",
"1004 Extended L1&L2 GPS RTK Observables for GPS RTK Use, the main msg ",
"1005 Stationary RTK Reference Station ARP ",
"1006 Stationary RTK Reference Station ARP plus the Antenna Height ",
"1007 Antenna Descriptor (msg 100.8 (X) is also commonly used)",
"1008 Antenna Descriptor and Serial Number",
"1009 L1-Only GLONASS RTK Observables (type x012 is to be preferred)",
"1010 Extended L1-Only GLONASS RTK Observables (type x012 is to be preferred)",
"1011 L1&L2 GLONASS RTK Observables (type x012 is to be preferred)",
"1012 Extended L1&L2 GLONASS RTK Observables, the other main msg ",
"1013 System Parameters, time offsets, lists of messages sent",
"1014 Network Auxiliary Station Data",
"1015 GPS Ionospheric Correction Differences",
"1016 GPS Geometric Correction Differences",
"1017 GPS Combined Geometric and Ionospheric Correction Differences",
"1018 RESERVED for Alternative Ionospheric Correction Difference Message",
"1019 GPS Broadcast Ephemeris (orbits) ",
"1020 GLONASS Broadcast Ephemeris (orbits) ",
"1021 Helmert / Abridged Molodenski Transformation Parameters",
"1022 Molodenski-Badekas Transformation Parameters",
"1023 Residuals, Ellipsoidal Grid Representation",
"1024 Residuals, Plane Grid Representation",
"1025 Projection Parameters, Projection Types other than Lambert Conic Conformal",
"1026 Projection Parameters, Projection Type LCC2SP (Lambert Conic Conformal",
"1027 Projection Parameters, Projection Type OM (Oblique Mercator)",
"1028 Reserved for Global to Plate-Fixed Transformation",
"1029 Unicode Text String (used for human readable text)",
"1030 GPS Network RTK Residual Message",
"1031 GLONASS Network RTK Residual",
"1032 Physical Reference Station Position",
"1033 Receiver and Antenna Descriptors",
"1034 GPS Network FKP Gradient",
"1035 GLONASS Network FKP Gradient",
"1036 Not defined at this time",
"1037 GLONASS Ionospheric Correction Differences",
"1038 GLONASS Geometric Correction Differences",
"1039 GLONASS Combined Geometric and Ionospheric Correction Differences",
"1042 BDS Satellite Ephemeris Data",
"1043 Not defined at this time",
"1044 QZSS Ephemerides",
"1045 Galileo Broadcast Ephemeris",
"1046 Galileo I/NAV Satellite Ephemeris Data",
# State Space Representation (SSR) Message Types
"1057 SSR GPS orbit corrections to Broadcast Ephemeris",
"1058 SSR GPS clock corrections to Broadcast Ephemeris",
"1059 SSR GPS code biases",
"1060 SSR Combined orbit and clock corrections to GPS Broadcast Ephemeris (popular)",
"1061 SSR GPS User Range Accuracy",
"1062 SSR High-rate GPS clock corrections to Broadcast Ephemeris",
"1063 SSR GLONASS orbit corrections for Broadcast Ephemeris",
"1064 SSR GLONASS clock corrections for Broadcast Ephemeris",
"1065 SSR GLONASS code biases",
"1066 SSR Combined orbit and clock corrections to GLONASS Broadcast Ephemeris (popular)",
"1067 SSR GLONASS User Range Accuracy (URA)",
"1068 SSR GLONASS High Rate Clock Correction",
"1070 Reserved for MSM",
"1071 GPS MSM1",
"1072 GPS MSM2",
"1073 GPS MSM3",
"1074 GPS MSM4",
"1075 GPS MSM5",
"1076 GPS MSM6",
"1077 GPS MSM7",
"1078 Reserved MSM",
"1079 Reserved MSM",
"1080 Reserved MSM",
"1081 GLONASS MSM1",
"1082 GLONASS MSM2",
"1083 GLONASS MSM3",
"1084 GLONASS MSM4",
"1085 GLONASS MSM5",
"1086 GLONASS MSM6",
"1087 GLONASS MSM7",
"1088 Reserved MSM",
"1089 Reserved MSM",
"1090 Reserved MSM",
"1091 Galileo MSM1",
"1092 Galileo MSM2",
"1093 Galileo MSM3",
"1094 Galileo MSM4",
"1095 Galileo MSM5",
"1096 Galileo MSM6",
"1097 Galileo MSM7",
"1098 Reserved MSM",
"1099 Reserved MSM",
"1100 Reserved MSM",
"1101 SBAS MSM1",
"1102 SBAS MSM2",
"1103 SBAS MSM3",
"1104 SBAS MSM4",
"1105 SBAS MSM5",
"1106 SBAS MSM6",
"1107 SBAS MSM7",
"1108 Reserved MSM",
"1109 Reserved MSM",
"1110 Reserved MSM",
"1111 QZSS MSM1",
"1112 QZSS MSM2",
"1113 QZSS MSM3",
"1114 QZSS MSM4",
"1115 QZSS MSM5",
"1116 QZSS MSM6",
"1117 QZSS MSM7",
"1118 Reserved MSM",
"1119 Reserved MSM",
"1120 Reserved MSM",
"1121 BeiDou MSM1",
"1122 BeiDou MSM2",
"1123 BeiDou MSM3",
"1124 BeiDou MSM4",
"1125 BeiDou MSM5",
"1126 BeiDou MSM6",
"1127 BeiDou MSM7",
"1128 Reserved MSM",
"1129 Reserved MSM",
"1130 Reserved MSM",
"1131 BeiDou MSM1",
"1132 BeiDou MSM2",
"1133 BeiDou MSM3",
"1134 BeiDou MSM4",
"1135 BeiDou MSM5",
"1136 BeiDou MSM6",
"1137 BeiDou MSM7",
"1138 Reserved MSM",
"1139 Reserved MSM",
"1140-1229 Reserved MSM",
#
# On ne teste que la plage 1000-1200
#
"1230 GLONASS L1 and L2 Code-Phase Biases",
"1300 Service-CRS",
"1301 The 15 Parameter Transformation",
"1302 RTCM-CRS",
"1303 BDS Network RTK Residual Message",
"1304 Galileo Network RTK Residual Message",
"4001-4095  Proprietary Messages",
"4078 Assigned to: ComNav Technology Ltd.",
"4077 Assigned to: Hemisphere GNSS Inc.",
"4076 Assigned to: International GNSS Service (IGS)",
"4075 Assigned to: Alberding GmbH",
"4074 Assigned to: Unicore Communications Inc.",
"4073 Assigned to: Mitsubishi Electric Corp.",
"4072 Assigned to: u-blox AG",
"4071 Assigned to: Wuhan Navigation and LBS",
"4070 Assigned to: Wuhan MengXin Technology",
"4069 Assigned to: VERIPOS Ltd",
"4068 Assigned to: Qianxun Location Networks Co. Ltd",
"4067 Assigned to: China Transport telecommunications & Information Center",
"4066 Assigned to: Lantmateriet",
"4065 Assigned to: Allystar Technology (Shenzhen) Co. Ltd.",
"4064 Assigned to: NTLab",
"4063 Assigned to: CHC Navigation (CHCNAV)",
"4062 Assigned to: SwiftNav Inc.",
"4061 Assigned to: Zhejiang GeeSpace Technology Co.,",
"Ocean engineering (KRISO)",
"4054 Assigned to: GEODNET",
"4053 Assigned to: Qualcomm Technologies, Inc.",
"4052 Assigned to: Furuno Electric Co., LTD.",
"4051 Assigned to: Hi-Target",
"4050 Assigned to: STMicroelectronics SRL",
"4049… Message Types 4049 — 4001 Are Reserved"
]