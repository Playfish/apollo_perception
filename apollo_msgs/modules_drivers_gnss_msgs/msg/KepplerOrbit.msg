#This message defines main six keppler orbit parameters and pertubations,
#designed for gps, beidou, (also supporting qzss, galileo)

modules_drivers_gnss_msgs/GnssType gnss_type  #[default = GPS_SYS]
uint32 sat_prn 
modules_drivers_gnss_msgs/GnssTimeType gnss_time_type  #[default = GPS_TIME]
    #TOC: time of clock
uint32 year 
uint32 month 
uint32 day 
uint32 hour
uint32 minute 
float64 second_s 

    #GNSSS week number
uint32 week_num 
float64 reserved 
float64 af0                #clock correction(sec)
float64 af1                #clock correction(sec/sec)
float64 af2                #clock correction(sec/sec2)
float64 iode               #Issue Of Data, Ephemeris in subframes 2 and 3
float64 deltan             #mean anomoly correction semi-circles per sec*pi = rads
float64 m0                 #mean anomoly at ref time semi-circles*pi = rads
float64 e                  #eccentricity
float64 roota              #sqr root a ( metres 1/2 )
float64 toe                #ref time (sec) of ephemeris
float64 toc                #ref time (sec) of clock
float64 cic                #harmonic correction term(rads)
float64 crc                #harmonic correction term(metres)
float64 cis                #harmonic correction term(rads)
float64 crs                #harmonic correction term(metres)
float64 cuc                #harmonic correction term(rads)
float64 cus                #harmonic correction term(rads)
float64 omega0             #longitude  of asccending node semi-circles*pi = rads
float64 omega              #argument of perigee semi-circles*pi
float64 i0                 #inclination angle at ref time semi-circles*pi
float64 omegadot           #rate of right ascension semi-circles/sec*pi
float64 idot               #rate of inclination semi-circles/sec*pi
float64 codesonL2channel   #pseudo range codes on L2
uint32 L2Pdataflag        #data flag of L2P
uint32 accuracy           #user range accuracy
uint32 health             #satellite health: 0=good,1=bad
float64 tgd                #grup delay (s)
float64 iodc               #Issue Of Data, Clock 
