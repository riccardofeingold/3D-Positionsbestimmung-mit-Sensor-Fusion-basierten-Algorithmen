GPS.begin(9600);
GPS.sendCommand(PMTK_ENABLE_SBAS);//Enable search for SBAS satellite (works only with 1Hz output rate)
GPS.sendCommand(PMTK_ENABLE_WAAS);//Enable DGPS to correct the postion data
GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);//Means nothing else than: print only RMC (lon,lat) and GGA (height over sealevel) data => both have the necessary data
GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);//Updating frequence 
GPS.sendCommand(PGCMD_ANTENNA);//I assume this says the GPS-shield that an antenna is attached 
delay(1000);
mySerial.println(PMTK_Q_RELEASE);