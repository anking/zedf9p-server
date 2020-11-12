/*
 Original library code from: https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library/blob/master/src/SparkFun_Ublox_Arduino_Library.cpp
 
 
 */


using System;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Threading;

namespace UBLOX
{
    class SFE_UBLOX_GPS
    {
        #region CLASS VARIABLES AND CONSTANTS

        //Variables
        SerialPort _serialPort;            //The generic connection to user's chosen Serial hardware
        Stream _nmeaOutputPort = null; //The user can assign an output port to print NMEA sentences if they wish
        public bool _debug { get; set; } = false;			//The stream to send debug messages to if enabled
        public Svin svin { get; }
        Func<byte, byte> _nmeaHandler = null;
        Func<byte, byte> _rtcmHandler = null;

        const int MAX_PAYLOAD_SIZE = 256; //We need ~220 bytes for getProtocolVersion on most ublox modules

        // A default of 250ms for maxWait seems fine for I2C but is not enough for SerialUSB.
        // If you know you are only going to be using I2C / Qwiic communication, you can
        // safely reduce defaultMaxWait to 250.
        const ushort defaultMaxWait = 1100; // Let's allow the user to define their own value if they want to

        // Global Status Returns
        enum sfe_ublox_status_e
        {
            SFE_UBLOX_STATUS_SUCCESS,
            SFE_UBLOX_STATUS_FAIL,
            SFE_UBLOX_STATUS_CRC_FAIL,
            SFE_UBLOX_STATUS_TIMEOUT,
            SFE_UBLOX_STATUS_COMMAND_NACK, // Indicates that the command was unrecognised, invalid or that the module is too busy to respond
            SFE_UBLOX_STATUS_OUT_OF_RANGE,
            SFE_UBLOX_STATUS_INVALID_ARG,
            SFE_UBLOX_STATUS_INVALID_OPERATION,
            SFE_UBLOX_STATUS_MEM_ERR,
            SFE_UBLOX_STATUS_HW_ERR,
            SFE_UBLOX_STATUS_DATA_SENT,     // This indicates that a 'set' was successful
            SFE_UBLOX_STATUS_DATA_RECEIVED, // This indicates that a 'get' (poll) was successful
            SFE_UBLOX_STATUS_I2C_COMM_FAILURE,
            SFE_UBLOX_STATUS_DATA_OVERWRITTEN // This is an error - the data was valid but has been or _is being_ overwritten by another packet
        };

        // ubxPacket validity
        enum sfe_ublox_packet_validity_e
        {
            SFE_UBLOX_PACKET_VALIDITY_NOT_VALID,
            SFE_UBLOX_PACKET_VALIDITY_VALID,
            SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED,
            SFE_UBLOX_PACKET_NOTACKNOWLEDGED // This indicates that we received a NACK
        };

        // Identify which packet buffer is in use:
        // packetCfg (or a custom packet), packetAck or packetBuf
        enum sfe_ublox_packet_buffer_e
        {
            SFE_UBLOX_PACKET_PACKETCFG,
            SFE_UBLOX_PACKET_PACKETACK,
            SFE_UBLOX_PACKET_PACKETBUF
        };        

        //Registers
        const byte UBX_SYNCH_1 = 0xB5;
        const byte UBX_SYNCH_2 = 0x62;

        //The following are UBX Class IDs. Descriptions taken from ZED-F9P Interface Description Document page 32, NEO-M8P Interface Description page 145
        const byte UBX_CLASS_NAV = 0x01;  //Navigation Results Messages: Position, Speed, Time, Acceleration, Heading, DOP, SVs used
        const byte UBX_CLASS_RXM = 0x02;  //Receiver Manager Messages: Satellite Status, RTC Status
        const byte UBX_CLASS_INF = 0x04;  //Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
        const byte UBX_CLASS_ACK = 0x05;  //Ack/Nak Messages: Acknowledge or Reject messages to UBX-CFG input messages
        const byte UBX_CLASS_CFG = 0x06;  //Configuration Input Messages: Configure the receiver.
        const byte UBX_CLASS_UPD = 0x09;  //Firmware Update Messages: Memory/Flash erase/write, Reboot, Flash identification, etc.
        const byte UBX_CLASS_MON = 0x0A;  //Monitoring Messages: Communication Status, CPU Load, Stack Usage, Task Status
        const byte UBX_CLASS_AID = 0x0B;  //(NEO-M8P ONLY!!!) AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input
        const byte UBX_CLASS_TIM = 0x0D;  //Timing Messages: Time Pulse Output, Time Mark Results
        const byte UBX_CLASS_ESF = 0x10;  //(NEO-M8P ONLY!!!) External Sensor Fusion Messages: External Sensor Measurements and Status Information
        const byte UBX_CLASS_MGA = 0x13;  //Multiple GNSS Assistance Messages: Assistance data for various GNSS
        const byte UBX_CLASS_LOG = 0x21;  //Logging Messages: Log creation, deletion, info and retrieval
        const byte UBX_CLASS_SEC = 0x27;  //Security Feature Messages
        const byte UBX_CLASS_HNR = 0x28;  //(NEO-M8P ONLY!!!) High Rate Navigation Results Messages: High rate time, position speed, heading
        const byte UBX_CLASS_NMEA = 0xF0; //NMEA Strings: standard NMEA strings

        //The following are used for configuration. Descriptions are from the ZED-F9P Interface Description pg 33-34 and NEO-M9N Interface Description pg 47-48
        const byte UBX_CFG_ANT = 0x13;       //Antenna Control Settings. Used to configure the antenna control settings
        const byte UBX_CFG_BATCH = 0x93;     //Get/set data batching configuration.
        const byte UBX_CFG_CFG = 0x09;       //Clear, Save, and Load Configurations. Used to save current configuration
        const byte UBX_CFG_DAT = 0x06;       //Set User-defined Datum or The currently defined Datum
        const byte UBX_CFG_DGNSS = 0x70;     //DGNSS configuration
        const byte UBX_CFG_GEOFENCE = 0x69;  //Geofencing configuration. Used to configure a geofence
        const byte UBX_CFG_GNSS = 0x3E;      //GNSS system configuration
        const byte UBX_CFG_INF = 0x02;       //Depending on packet length, either: poll configuration for one protocol, or information message configuration
        const byte UBX_CFG_ITFM = 0x39;      //Jamming/Interference Monitor configuration
        const byte UBX_CFG_LOGFILTER = 0x47; //Data Logger Configuration
        const byte UBX_CFG_MSG = 0x01;       //Poll a message configuration, or Set Message Rate(s), or Set Message Rate
        const byte UBX_CFG_NAV5 = 0x24;      //Navigation Engine Settings. Used to configure the navigation engine including the dynamic model.
        const byte UBX_CFG_NAVX5 = 0x23;     //Navigation Engine Expert Settings
        const byte UBX_CFG_NMEA = 0x17;      //Extended NMEA protocol configuration V1
        const byte UBX_CFG_ODO = 0x1E;       //Odometer, Low-speed COG Engine Settings
        const byte UBX_CFG_PM2 = 0x3B;       //Extended power management configuration
        const byte UBX_CFG_PMS = 0x86;       //Power mode setup
        const byte UBX_CFG_PRT = 0x00;       //Used to configure port specifics. Polls the configuration for one I/O Port, or Port configuration for UART ports, or Port configuration for USB port, or Port configuration for SPI port, or Port configuration for DDC port
        const byte UBX_CFG_PWR = 0x57;       //Put receiver in a defined power state
        const byte UBX_CFG_RATE = 0x08;      //Navigation/Measurement Rate Settings. Used to set port baud rates.
        const byte UBX_CFG_RINV = 0x34;      //Contents of Remote Inventory
        const byte UBX_CFG_RST = 0x04;       //Reset Receiver / Clear Backup Data Structures. Used to reset device.
        const byte UBX_CFG_RXM = 0x11;       //RXM configuration
        const byte UBX_CFG_SBAS = 0x16;      //SBAS configuration
        const byte UBX_CFG_TMODE3 = 0x71;    //Time Mode Settings 3. Used to enable Survey In Mode
        const byte UBX_CFG_TP5 = 0x31;       //Time Pulse Parameters
        const byte UBX_CFG_USB = 0x1B;       //USB Configuration
        const byte UBX_CFG_VALDEL = 0x8C;    //Used for config of higher version u-blox modules (ie protocol v27 and above). Deletes values corresponding to provided keys/ provided keys with a transaction
        const byte UBX_CFG_VALGET = 0x8B;    //Used for config of higher version u-blox modules (ie protocol v27 and above). Configuration Items
        const byte UBX_CFG_VALSET = 0x8A;    //Used for config of higher version u-blox modules (ie protocol v27 and above). Sets values corresponding to provided key-value pairs/ provided key-value pairs within a transaction.

        //The following are used to enable NMEA messages. Descriptions come from the NMEA messages overview in the ZED-F9P Interface Description
        const byte UBX_NMEA_MSB = 0xF0;  //All NMEA enable commands have 0xF0 as MSB
        const byte UBX_NMEA_DTM = 0x0A;  //GxDTM (datum reference)
        const byte UBX_NMEA_GAQ = 0x45;  //GxGAQ (poll a standard message (if the current talker ID is GA))
        const byte UBX_NMEA_GBQ = 0x44;  //GxGBQ (poll a standard message (if the current Talker ID is GB))
        const byte UBX_NMEA_GBS = 0x09;  //GxGBS (GNSS satellite fault detection)
        const byte UBX_NMEA_GGA = 0x00;  //GxGGA (Global positioning system fix data)
        const byte UBX_NMEA_GLL = 0x01;  //GxGLL (latitude and long, whith time of position fix and status)
        const byte UBX_NMEA_GLQ = 0x43;  //GxGLQ (poll a standard message (if the current Talker ID is GL))
        const byte UBX_NMEA_GNQ = 0x42;  //GxGNQ (poll a standard message (if the current Talker ID is GN))
        const byte UBX_NMEA_GNS = 0x0D;  //GxGNS (GNSS fix data)
        const byte UBX_NMEA_GPQ = 0x040; //GxGPQ (poll a standard message (if the current Talker ID is GP))
        const byte UBX_NMEA_GRS = 0x06;  //GxGRS (GNSS range residuals)
        const byte UBX_NMEA_GSA = 0x02;  //GxGSA (GNSS DOP and Active satellites)
        const byte UBX_NMEA_GST = 0x07;  //GxGST (GNSS Pseudo Range Error Statistics)
        const byte UBX_NMEA_GSV = 0x03;  //GxGSV (GNSS satellites in view)
        const byte UBX_NMEA_RMC = 0x04;  //GxRMC (Recommended minimum data)
        const byte UBX_NMEA_TXT = 0x41;  //GxTXT (text transmission)
        const byte UBX_NMEA_VLW = 0x0F;  //GxVLW (dual ground/water distance)
        const byte UBX_NMEA_VTG = 0x05;  //GxVTG (course over ground and Ground speed)
        const byte UBX_NMEA_ZDA = 0x08;  //GxZDA (Time and Date)

        //The following are used to configure the NMEA protocol main talker ID and GSV talker ID
        const byte UBX_NMEA_MAINTALKERID_NOTOVERRIDDEN = 0x00; //main talker ID is system dependent
        const byte UBX_NMEA_MAINTALKERID_GP = 0x01;            //main talker ID is GPS
        const byte UBX_NMEA_MAINTALKERID_GL = 0x02;            //main talker ID is GLONASS
        const byte UBX_NMEA_MAINTALKERID_GN = 0x03;            //main talker ID is combined receiver
        const byte UBX_NMEA_MAINTALKERID_GA = 0x04;            //main talker ID is Galileo
        const byte UBX_NMEA_MAINTALKERID_GB = 0x05;            //main talker ID is BeiDou
        const byte UBX_NMEA_GSVTALKERID_GNSS = 0x00;           //GNSS specific Talker ID (as defined by NMEA)
        const byte UBX_NMEA_GSVTALKERID_MAIN = 0x01;           //use the main Talker ID

        //The following are used to configure INF UBX messages (information messages).  Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 34)
        const byte UBX_INF_CLASS = 0x04;   //All INF messages have 0x04 as the class
        const byte UBX_INF_DEBUG = 0x04;   //ASCII output with debug contents
        const byte UBX_INF_ERROR = 0x00;   //ASCII output with error contents
        const byte UBX_INF_NOTICE = 0x02;  //ASCII output with informational contents
        const byte UBX_INF_TEST = 0x03;    //ASCII output with test contents
        const byte UBX_INF_WARNING = 0x01; //ASCII output with warning contents

        //The following are used to configure LOG UBX messages (loggings messages).  Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 34)
        const byte UBX_LOG_CREATE = 0x07;           //Create Log File
        const byte UBX_LOG_ERASE = 0x03;            //Erase Logged Data
        const byte UBX_LOG_FINDTIME = 0x0E;         //Find index of a log entry based on a given time, or response to FINDTIME requested
        const byte UBX_LOG_INFO = 0x08;             //Poll for log information, or Log information
        const byte UBX_LOG_RETRIEVEPOSEXTRA = 0x0F; //Odometer log entry
        const byte UBX_LOG_RETRIEVEPOS = 0x0B;      //Position fix log entry
        const byte UBX_LOG_RETRIEVESTRING = 0x0D;   //Byte string log entry
        const byte UBX_LOG_RETRIEVE = 0x09;         //Request log data
        const byte UBX_LOG_STRING = 0x04;           //Store arbitrary string on on-board flash

        //The following are used to configure MGA UBX messages (Multiple GNSS Assistance Messages).  Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 34)
        const byte UBX_MGA_ACK_DATA0 = 0x60;      //Multiple GNSS Acknowledge message
        const byte UBX_MGA_BDS_EPH = 0x03;        //BDS Ephemeris Assistance
        const byte UBX_MGA_BDS_ALM = 0x03;        //BDS Almanac Assistance
        const byte UBX_MGA_BDS_HEALTH = 0x03;     //BDS Health Assistance
        const byte UBX_MGA_BDS_UTC = 0x03;        //BDS UTC Assistance
        const byte UBX_MGA_BDS_IONO = 0x03;       //BDS Ionospheric Assistance
        const byte UBX_MGA_DBD = 0x80;            //Either: Poll the Navigation Database, or Navigation Database Dump Entry
        const byte UBX_MGA_GAL_EPH = 0x02;        //Galileo Ephemeris Assistance
        const byte UBX_MGA_GAL_ALM = 0x02;        //Galileo Almanac Assitance
        const byte UBX_MGA_GAL_TIMOFFSET = 0x02;  //Galileo GPS time offset assistance
        const byte UBX_MGA_GAL_UTC = 0x02;        //Galileo UTC Assistance
        const byte UBX_MGA_GLO_EPH = 0x06;        //GLONASS Ephemeris Assistance
        const byte UBX_MGA_GLO_ALM = 0x06;        //GLONASS Almanac Assistance
        const byte UBX_MGA_GLO_TIMEOFFSET = 0x06; //GLONASS Auxiliary Time Offset Assistance
        const byte UBX_MGA_GPS_EPH = 0x00;        //GPS Ephemeris Assistance
        const byte UBX_MGA_GPS_ALM = 0x00;        //GPS Almanac Assistance
        const byte UBX_MGA_GPS_HEALTH = 0x00;     //GPS Health Assistance
        const byte UBX_MGA_GPS_UTC = 0x00;        //GPS UTC Assistance
        const byte UBX_MGA_GPS_IONO = 0x00;       //GPS Ionosphere Assistance
        const byte UBX_MGA_INI_POS_XYZ = 0x40;    //Initial Position Assistance
        const byte UBX_MGA_INI_POS_LLH = 0x40;    //Initial Position Assitance
        const byte UBX_MGA_INI_TIME_UTC = 0x40;   //Initial Time Assistance
        const byte UBX_MGA_INI_TIME_GNSS = 0x40;  //Initial Time Assistance
        const byte UBX_MGA_INI_CLKD = 0x40;       //Initial Clock Drift Assitance
        const byte UBX_MGA_INI_FREQ = 0x40;       //Initial Frequency Assistance
        const byte UBX_MGA_INI_EOP = 0x40;        //Earth Orientation Parameters Assistance
        const byte UBX_MGA_QZSS_EPH = 0x05;       //QZSS Ephemeris Assistance
        const byte UBX_MGA_QZSS_ALM = 0x05;       //QZSS Almanac Assistance
        const byte UBX_MGA_QZAA_HEALTH = 0x05;    //QZSS Health Assistance

        //The following are used to configure the MON UBX messages (monitoring messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 35)
        const byte UBX_MON_COMMS = 0x36; //Comm port information
        const byte UBX_MON_GNSS = 0x28;  //Information message major GNSS selection
        const byte UBX_MON_HW2 = 0x0B;   //Extended Hardware Status
        const byte UBX_MON_HW3 = 0x37;   //HW I/O pin information
        const byte UBX_MON_HW = 0x09;    //Hardware Status
        const byte UBX_MON_IO = 0x02;    //I/O Subsystem Status
        const byte UBX_MON_MSGPP = 0x06; //Message Parse and Process Status
        const byte UBX_MON_PATCH = 0x27; //Output information about installed patches
        const byte UBX_MON_RF = 0x38;    //RF information
        const byte UBX_MON_RXBUF = 0x07; //Receiver Buffer Status
        const byte UBX_MON_RXR = 0x21;   //Receiver Status Information
        const byte UBX_MON_TXBUF = 0x08; //Transmitter Buffer Status. Used for query tx buffer size/state.
        const byte UBX_MON_VER = 0x04;   //Receiver/Software Version. Used for obtaining Protocol Version.

        //The following are used to configure the NAV UBX messages (navigation results messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 35-36)
        const byte UBX_NAV_ATT = 0x05;       //Vehicle "Attitude" Solution
        const byte UBX_NAV_CLOCK = 0x22;     //Clock Solution
        const byte UBX_NAV_DOP = 0x04;       //Dilution of precision
        const byte UBX_NAV_EOE = 0x61;       //End of Epoch
        const byte UBX_NAV_GEOFENCE = 0x39;  //Geofencing status. Used to poll the geofence status
        const byte UBX_NAV_HPPOSECEF = 0x13; //High Precision Position Solution in ECEF. Used to find our positional accuracy (high precision).
        const byte UBX_NAV_HPPOSLLH = 0x14;  //High Precision Geodetic Position Solution. Used for obtaining lat/long/alt in high precision
        const byte UBX_NAV_ODO = 0x09;       //Odometer Solution
        const byte UBX_NAV_ORB = 0x34;       //GNSS Orbit Database Info
        const byte UBX_NAV_POSECEF = 0x01;   //Position Solution in ECEF
        const byte UBX_NAV_POSLLH = 0x02;    //Geodetic Position Solution
        const byte UBX_NAV_PVT = 0x07;       //All the things! Position, velocity, time, PDOP, height, h/v accuracies, number of satellites. Navigation Position Velocity Time Solution.
        const byte UBX_NAV_RELPOSNED = 0x3C; //Relative Positioning Information in NED frame
        const byte UBX_NAV_RESETODO = 0x10;  //Reset odometer
        const byte UBX_NAV_SAT = 0x35;       //Satellite Information
        const byte UBX_NAV_SIG = 0x43;       //Signal Information
        const byte UBX_NAV_STATUS = 0x03;    //Receiver Navigation Status
        const byte UBX_NAV_SVIN = 0x3B;      //Survey-in data. Used for checking Survey In status
        const byte UBX_NAV_TIMEBDS = 0x24;   //BDS Time Solution
        const byte UBX_NAV_TIMEGAL = 0x25;   //Galileo Time Solution
        const byte UBX_NAV_TIMEGLO = 0x23;   //GLO Time Solution
        const byte UBX_NAV_TIMEGPS = 0x20;   //GPS Time Solution
        const byte UBX_NAV_TIMELS = 0x26;    //Leap second event information
        const byte UBX_NAV_TIMEUTC = 0x21;   //UTC Time Solution
        const byte UBX_NAV_VELECEF = 0x11;   //Velocity Solution in ECEF
        const byte UBX_NAV_VELNED = 0x12;    //Velocity Solution in NED

        //The following are used to configure the RXM UBX messages (receiver manager messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 36)
        const byte UBX_RXM_MEASX = 0x14; //Satellite Measurements for RRLP
        const byte UBX_RXM_PMREQ = 0x41; //Requests a Power Management task (two differenent packet sizes)
        const byte UBX_RXM_RAWX = 0x15;  //Multi-GNSS Raw Measurement Data
        const byte UBX_RXM_RLM = 0x59;   //Galileo SAR Short-RLM report (two different packet sizes)
        const byte UBX_RXM_RTCM = 0x32;  //RTCM input status
        const byte UBX_RXM_SFRBX = 0x13; //Boradcast Navigation Data Subframe

        //The following are used to configure the SEC UBX messages (security feature messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 36)
        const byte UBX_SEC_UNIQID = 0x03; //Unique chip ID

        //The following are used to configure the TIM UBX messages (timing messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 36)
        const byte UBX_TIM_TM2 = 0x03;  //Time mark data
        const byte UBX_TIM_TP = 0x01;   //Time Pulse Timedata
        const byte UBX_TIM_VRFY = 0x06; //Sourced Time Verification

        //The following are used to configure the UPD UBX messages (firmware update messages). Descriptions from UBX messages overview (ZED-F9P Interface Description Document page 36)
        const byte UBX_UPD_SOS = 0x14; //Poll Backup Fil Restore Status, Create Backup File in Flash, Clear Backup File in Flash, Backup File Creation Acknowledge, System Restored from Backup

        //The following are used to enable RTCM messages
        public const byte UBX_RTCM_MSB = 0xF5;    //All RTCM enable commands have 0xF5 as MSB
        public const byte UBX_RTCM_1005 = 0x05;   //Stationary RTK reference ARP
        public const byte UBX_RTCM_1074 = 0x4A;   //GPS MSM4
        public const byte UBX_RTCM_1077 = 0x4D;   //GPS MSM7
        public const byte UBX_RTCM_1084 = 0x54;   //GLONASS MSM4
        public const byte UBX_RTCM_1087 = 0x57;   //GLONASS MSM7
        public const byte UBX_RTCM_1094 = 0x5E;   //Galileo MSM4
        public const byte UBX_RTCM_1097 = 0x61;   //Galileo MSM7
        public const byte UBX_RTCM_1124 = 0x7C;   //BeiDou MSM4
        public const byte UBX_RTCM_1127 = 0x7F;   //BeiDou MSM7
        public const byte UBX_RTCM_1230 = 0xE6;   //GLONASS code-phase biases, set to once every 10 seconds
        public const byte UBX_RTCM_4072_0 = 0xFE; //Reference station PVT (ublox proprietary RTCM message)
        public const byte UBX_RTCM_4072_1 = 0xFD; //Additional reference station information (ublox proprietary RTCM message)

        const byte UBX_ACK_NACK = 0x00;
        const byte UBX_ACK_ACK = 0x01;
        const byte UBX_ACK_NONE = 0x02; //Not a real value

        ushort rtcmLen = 0;

        // The following constants are used to get External Sensor Measurements and Status
        // Information.
        const byte UBX_ESF_MEAS = 0x02;
        const byte UBX_ESF_RAW = 0x03;
        const byte UBX_ESF_STATUS = 0x10;
        const byte UBX_ESF_INS = 0x15; //36 bytes

        const byte SVIN_MODE_DISABLE = 0x00;
        const byte SVIN_MODE_ENABLE = 0x01;

        //The following consts are used to configure the various ports and streams for those ports. See -CFG-PRT.
        public const byte COM_PORT_I2C = 0;
        public const byte COM_PORT_UART1 = 1;
        public const byte COM_PORT_UART2 = 2;
        public const byte COM_PORT_USB = 3;
        public const byte COM_PORT_SPI = 4;


        const byte COM_TYPE_UBX = (1 << 0);
        const byte COM_TYPE_NMEA = (1 << 1);
        const byte COM_TYPE_RTCM3 = (1 << 5);

        // Configuration Sub-Section mask definitions for saveConfigSelective (UBX-CFG-CFG)
        const uint VAL_CFG_SUBSEC_IOPORT = 0x00000001;   // ioPort - communications port settings (causes IO system reset!)
        const uint VAL_CFG_SUBSEC_MSGCONF = 0x00000002;  // msgConf - message configuration
        const uint VAL_CFG_SUBSEC_INFMSG = 0x00000004;   // infMsg - INF message configuration
        const uint VAL_CFG_SUBSEC_NAVCONF = 0x00000008;  // navConf - navigation configuration
        const uint VAL_CFG_SUBSEC_RXMCONF = 0x00000010;  // rxmConf - receiver manager configuration
        const uint VAL_CFG_SUBSEC_SENCONF = 0x00000100;  // senConf - sensor interface configuration (requires protocol 19+)
        const uint VAL_CFG_SUBSEC_RINVCONF = 0x00000200; // rinvConf - remove inventory configuration
        const uint VAL_CFG_SUBSEC_ANTCONF = 0x00000400;  // antConf - antenna configuration
        const uint VAL_CFG_SUBSEC_LOGCONF = 0x00000800;  // logConf - logging configuration
        const uint VAL_CFG_SUBSEC_FTSCONF = 0x00001000;  // ftsConf - FTS configuration (FTS products only)

        // Bitfield wakeupSources for UBX_RXM_PMREQ
        const uint VAL_RXM_PMREQ_WAKEUPSOURCE_UARTRX = 0x00000008;  // uartrx
        const uint VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT0 = 0x00000020; // extint0
        const uint VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT1 = 0x00000040; // extint1
        const uint VAL_RXM_PMREQ_WAKEUPSOURCE_SPICS = 0x00000080;   // spics

        public enum dynModel // Possible values for the dynamic platform model, which provide more accuract position output for the situation. Description extracted from ZED-F9P Integration Manual
        {
            DYN_MODEL_PORTABLE = 0, //Applications with low acceleration, e.g. portable devices. Suitable for most situations.
                                    // 1 is not defined
            DYN_MODEL_STATIONARY = 2, //Used in timing applications (antenna must be stationary) or other stationary applications. Velocity restricted to 0 m/s. Zero dynamics assumed.
            DYN_MODEL_PEDESTRIAN,     //Applications with low acceleration and speed, e.g. how a pedestrian would move. Low acceleration assumed.
            DYN_MODEL_AUTOMOTIVE,     //Used for applications with equivalent dynamics to those of a passenger car. Low vertical acceleration assumed
            DYN_MODEL_SEA,            //Recommended for applications at sea, with zero vertical velocity. Zero vertical velocity assumed. Sea level assumed.
            DYN_MODEL_AIRBORNE1g,     //Airborne <1g acceleration. Used for applications with a higher dynamic range and greater vertical acceleration than a passenger car. No 2D position fixes supported.
            DYN_MODEL_AIRBORNE2g,     //Airborne <2g acceleration. Recommended for typical airborne environments. No 2D position fixes supported.
            DYN_MODEL_AIRBORNE4g,     //Airborne <4g acceleration. Only recommended for extremely dynamic environments. No 2D position fixes supported.
            DYN_MODEL_WRIST,          // Not supported in protocol versions less than 18. Only recommended for wrist worn applications. Receiver will filter out arm motion.
            DYN_MODEL_BIKE,           // Supported in protocol versions 19.2
        };

        //# ifndef MAX_PAYLOAD_SIZE

        //#define MAX_PAYLOAD_SIZE 256 //We need ~220 bytes for getProtocolVersion on most ublox modules
        //#define MAX_PAYLOAD_SIZE 768 //Worst case: UBX_CFG_VALSET packet with 64 keyIDs each with 64 bit values

        //#endif

        //-=-=-=-=- UBX binary specific variables
        class ubxPacket
        {

            public byte cls;
            public byte id;
            public short len;          //Length of the payload. Does not include cls, id, or checksum bytes
            public short counter;      //Keeps track of number of overall bytes received. Some responses are larger than 255 bytes.
            public short startingSpot; //The counter value needed to go past before we begin recording into payload array
            public byte[] payload;
            public byte checksumA; //Given to us from module. Checked against the rolling calculated A/B checksums.
            public byte checksumB;
            public sfe_ublox_packet_validity_e valid;           //Goes from NOT_DEFINED to VALID or NOT_VALID when checksum is checked
            public sfe_ublox_packet_validity_e classAndIDmatch; // Goes from NOT_DEFINED to VALID or NOT_VALID when the Class and ID match the requestedClass and requestedID

            public ubxPacket()
            {
                cls = 0;
                id = 0;
                len = 0;
                counter = 0;
                startingSpot = 0;
                payload = new byte[] { };
                checksumA = 0;
                checksumB = 0;
                valid = sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
                classAndIDmatch = sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
            }
        };

        // Struct to hold the results returned by getGeofenceState (returned by UBX-NAV-GEOFENCE)
        public class geofenceState
        {
            //public byte status;    // Geofencing status: 0 - Geofencing not available or not reliable; 1 - Geofencing active
            //public byte numFences; // Number of geofences
            //public byte combState; // Combined (logical OR) state of all geofences: 0 - Unknown; 1 - Inside; 2 - Outside
            //public byte states[4]; // Geofence states: 0 - Unknown; 1 - Inside; 2 - Outside
        };

        // Struct to hold the current geofence parameters
        public class geofenceParams
        {
            //public byte numFences; // Number of active geofences
            //public int lats[4];   // Latitudes of geofences (in degrees * 10^-7)
            //public int longs[4];  // Longitudes of geofences (in degrees * 10^-7)
            //public uint rads[4];  // Radii of geofences (in m * 10^-2)
        };

        //Survey-in specific controls
        public class Svin
        {
            public bool active;
            public bool valid;
            public ushort observationTime;
            public float meanAccuracy;
        };

        public enum SentenceTypes
        {
            NONE = 0,
            NMEA,
            UBX,
            RTCM
        };

        SentenceTypes currentSentence = SentenceTypes.NONE;
        short ubxFrameCounter;            //It counts all UBX frame. [Fixed header(2bytes), CLS(1byte), ID(1byte), length(2bytes), payload(x bytes), checksums(2bytes)]
        short rtcmFrameCounter = 0; //Tracks the type of incoming byte inside RTCM frame

        //Flag if this packet is unrequested (and so should be ignored and not copied into packetCfg or packetAck)
        bool ignoreThisPayload = false;


        //Identify which buffer is in use
        //Data is stored in packetBuf until the requested class and ID can be validated
        //If a match is seen, data is diverted into packetAck or packetCfg
        sfe_ublox_packet_buffer_e activePacketBuffer = sfe_ublox_packet_buffer_e.SFE_UBLOX_PACKET_PACKETBUF;

        byte rollingChecksumA; //Rolls forward as we receive incoming bytes. Checked against the last two A/B checksum bytes
        byte rollingChecksumB; //Rolls forward as we receive incoming bytes. Checked against the last two A/B checksum bytes

        //The major datums we want to globally store
        short gpsYear;
        byte gpsMonth;
        byte gpsDay;
        byte gpsHour;
        byte gpsMinute;
        byte gpsSecond;
        short gpsMillisecond;
        int gpsNanosecond;
        bool gpsDateValid;
        bool gpsTimeValid;

        int latitude;        //Degrees * 10^-7 (more accurate than floats)
        int longitude;       //Degrees * 10^-7 (more accurate than floats)
        int altitude;        //Number of mm above ellipsoid
        int altitudeMSL;     //Number of mm above Mean Sea Level
        byte SIV;             //Number of satellites used in position solution
        byte fixType;         //Tells us when we have a solution aka lock
        byte carrierSolution; //Tells us when we have an RTK float/fixed solution
        int groundSpeed;     //mm/s
        int headingOfMotion; //degrees * 10^-5
        short pDOP;           //Positional dilution of precision * 10^-2 (dimensionless)
        byte versionLow;      //Loaded from getProtocolVersion().
        byte versionHigh;

        uint timeOfWeek;         // ms
        int highResLatitude;     // Degrees * 10^-7
        int highResLongitude;    // Degrees * 10^-7
        int elipsoid;            // Height above ellipsoid in mm (Typo! Should be eLLipsoid! **Uncorrected for backward-compatibility.**)
        int meanSeaLevel;        // Height above mean sea level in mm
        int geoidSeparation;     // This seems to only be provided in NMEA GGA and GNS messages
        uint horizontalAccuracy; // mm * 10^-1 (i.e. 0.1mm)
        uint verticalAccuracy;   // mm * 10^-1 (i.e. 0.1mm)
        byte elipsoidHp;           // High precision component of the height above ellipsoid in mm * 10^-1 (Deliberate typo! Should be eLLipsoidHp!)
        byte meanSeaLevelHp;       // High precision component of Height above mean sea level in mm * 10^-1
        byte highResLatitudeHp;    // High precision component of latitude: Degrees * 10^-9
        byte highResLongitudeHp;   // High precision component of longitude: Degrees * 10^-9

        //The packet buffers
        //These are pointed at from within the ubxPacket
        public static byte[] payloadAck = new byte[2];                // Holds the requested ACK/NACK
        public static byte[] payloadCfg = new byte[MAX_PAYLOAD_SIZE]; // Holds the requested data packet
        public static byte[] payloadBuf = new byte[2];				  // Temporary buffer used to screen incoming packets or dump unrequested packets

        //Init the packet structures and init them with pointers to the payloadAck, payloadCfg and payloadBuf arrays
        ubxPacket packetAck = new ubxPacket() { payload = payloadAck };
        ubxPacket packetCfg = new ubxPacket() { payload = payloadCfg };
        ubxPacket packetBuf = new ubxPacket() { payload = payloadBuf };

        //Create bit field for staleness of each datum in PVT we want to monitor
        //moduleQueried.latitude goes true each time we call getPVT()
        //This reduces the number of times we have to call getPVT as this can take up to ~1s per read
        //depending on update rate
        class moduleQueried
        {
            public static bool gpsiTOW = true;
            public static bool gpsYear = true;
            public static bool gpsMonth = true;
            public static bool gpsDay = true;
            public static bool gpsHour = true;
            public static bool gpsMinute = true;
            public static bool gpsSecond = true;
            public static bool gpsDateValid = true;
            public static bool gpsTimeValid = true;
            public static bool gpsNanosecond = true;

            public static bool all = true;
            public static bool longitude = true;
            public static bool latitude = true;
            public static bool altitude = true;
            public static bool altitudeMSL = true;
            public static bool SIV = true;
            public static bool fixType = true;
            public static bool carrierSolution = true;
            public static bool groundSpeed = true;
            public static bool headingOfMotion = true;
            public static bool pDOP = true;
            public static bool versionNumber = true;
        };

        class highResModuleQueried
        {

            public static bool all = true;
            public static bool timeOfWeek = true;
            public static bool highResLatitude = true;
            public static bool highResLongitude = true;
            public static bool elipsoid = true;
            public static bool meanSeaLevel = true;
            public static bool geoidSeparation = true; // Redundant but kept for backward-compatibility
            public static bool horizontalAccuracy = true;
            public static bool verticalAccuracy = true;
            public static bool elipsoidHp = true;
            public static bool meanSeaLevelHp = true;
            public static bool highResLatitudeHp = true;
            public static bool highResLongitudeHp = true;
        };

        #endregion

        public SFE_UBLOX_GPS(bool debug) : this(SerialPort.GetPortNames().Last(), debug){}

        public SFE_UBLOX_GPS(string portName, bool debug)
        {
            _debug = debug;

            svin = new Svin();

            // Create a new SerialPort object with default settings.
            _serialPort = new SerialPort();            

            // Allow the user to set the appropriate properties.
            _serialPort.PortName = portName;
            _serialPort.BaudRate = 115200;
            _serialPort.Parity = Parity.None;
            _serialPort.DataBits = 8;
            _serialPort.StopBits = StopBits.One;

            // Set the read/write timeouts
            _serialPort.ReadTimeout = 500;
            _serialPort.WriteTimeout = 500;

            Console.WriteLine("Trying to open port "+portName);

            _serialPort.Open();
        }

        //Begin Survey-In for NEO-M8P
        public bool enableSurveyMode(ushort observationTime, float requiredAccuracy, ushort maxWait = defaultMaxWait)
        {
            return setSurveyMode(SVIN_MODE_ENABLE, observationTime, requiredAccuracy, maxWait);
        }

        //Stop Survey-In for NEO-M8P
        bool disableSurveyMode(ushort maxWait)
        {
            return setSurveyMode(SVIN_MODE_DISABLE, 0, 0, maxWait);
        }

        //Reads survey in status and sets the global variables
        //for status, position valid, observation time, and mean 3D StdDev
        //Returns true if commands was successful
        public bool getSurveyStatus(ushort maxWait)
        {
            //Reset variables
            svin.active = false;
            svin.valid = false;
            svin.observationTime = 0;
            svin.meanAccuracy = 0;

            packetCfg.cls = UBX_CLASS_NAV;
            packetCfg.id = UBX_NAV_SVIN;
            packetCfg.len = 0;
            packetCfg.startingSpot = 0;

            if (sendCommand(packetCfg, maxWait) != sfe_ublox_status_e.SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
                return (false);                                                         //If command send fails then bail

            //We got a response, now parse the bits into the svin structure

            //dur (Passed survey-in observation time) is U4 (uint) seconds. We truncate to 16 bits
            //(waiting more than 65535 seconds (18.2 hours) seems excessive!)
            uint tmpObsTime = extractLong(8);
            if (tmpObsTime <= 0xFFFF)
            {
                svin.observationTime = (ushort)tmpObsTime;
            }
            else
            {
                svin.observationTime = 0xFFFF;
            }

            // meanAcc is U4 (uint) in 0.1mm. We convert this to float.
            uint tempFloat = extractLong(28);
            svin.meanAccuracy = tempFloat / 10000.0F; //Convert 0.1mm to m

            svin.valid = Convert.ToBoolean(payloadCfg[36]);  //1 if survey-in position is valid, 0 otherwise
            svin.active = Convert.ToBoolean(payloadCfg[37]); //1 if survey-in in progress, 0 otherwise

            return true;
        }

        //Control Survey-In for NEO-M8P
        bool setSurveyMode(byte mode, ushort observationTime, float requiredAccuracy, ushort maxWait)
        {
            if (getSurveyMode(maxWait) == false) //Ask module for the current TimeMode3 settings. Loads into payloadCfg.
                return (false);

            packetCfg.cls = UBX_CLASS_CFG;
            packetCfg.id = UBX_CFG_TMODE3;
            packetCfg.len = 40;
            packetCfg.startingSpot = 0;

            //Clear packet payload
            for (byte x = 0; x < packetCfg.len; x++)
                packetCfg.payload[x] = 0;

            //payloadCfg should be loaded with poll response. Now modify only the bits we care about
            payloadCfg[2] = mode; //Set mode. Survey-In and Disabled are most common. Use ECEF (not LAT/LON/ALT).

            //svinMinDur is U4 (uint) but we'll only use a ushort (waiting more than 65535 seconds seems excessive!)
            payloadCfg[24] = (byte)(observationTime & 0xFF); //svinMinDur in seconds
            payloadCfg[25] = (byte)(observationTime >> 8);   //svinMinDur in seconds
            payloadCfg[26] = 0;                      //Truncate to 16 bits
            payloadCfg[27] = 0;                      //Truncate to 16 bits

            //svinAccLimit is U4 (uint) in 0.1mm.
            uint svinAccLimit = (uint)(requiredAccuracy * 10000.0); //Convert m to 0.1mm
            payloadCfg[28] = (byte)(svinAccLimit & 0xFF);                           //svinAccLimit in 0.1mm increments
            payloadCfg[29] = (byte)(svinAccLimit >> 8);
            payloadCfg[30] = (byte)(svinAccLimit >> 16);
            payloadCfg[31] = (byte)(svinAccLimit >> 24);

            return sendCommand(packetCfg, maxWait) == sfe_ublox_status_e.SFE_UBLOX_STATUS_DATA_SENT; // We are only expecting an ACK
        }

        //Get the current TimeMode3 settings - these contain survey in statuses
        bool getSurveyMode(ushort maxWait)
        {
            packetCfg.cls = UBX_CLASS_CFG;
            packetCfg.id = UBX_CFG_TMODE3;
            packetCfg.len = 0;
            packetCfg.startingSpot = 0;

            return sendCommand(packetCfg, maxWait) == sfe_ublox_status_e.SFE_UBLOX_STATUS_DATA_RECEIVED; // We are expecting data and an ACK
        }

        //Much of this configuration is not documented and instead discerned from u-center binary console
        public bool enableRTCMmessage(byte messageNumber, byte portID, byte sendRate, ushort maxWait = defaultMaxWait) => configureMessage(UBX_RTCM_MSB, messageNumber, portID, sendRate, maxWait);

        //Configure a given message type for a given port (UART1, I2C, SPI, etc)
        bool configureMessage(byte msgClass, byte msgID, byte portID, byte sendRate, ushort maxWait)
        {
            //Poll for the current settings for a given message
            packetCfg.cls = UBX_CLASS_CFG;
            packetCfg.id = UBX_CFG_MSG;
            packetCfg.len = 2;
            packetCfg.startingSpot = 0;

            payloadCfg[0] = msgClass;
            payloadCfg[1] = msgID;

            //This will load the payloadCfg array with current settings of the given register
            if (sendCommand(packetCfg, maxWait) != sfe_ublox_status_e.SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
                return false;                                                       //If command send fails then bail

            //Now send it back with new mods
            packetCfg.len = 8;

            //payloadCfg is now loaded with current bytes. Change only the ones we need to
            payloadCfg[2 + portID] = sendRate; //Send rate is relative to the event a message is registered on. For example, if the rate of a navigation message is set to 2, the message is sent every 2nd navigation solution.

            return sendCommand(packetCfg, maxWait) == sfe_ublox_status_e.SFE_UBLOX_STATUS_DATA_SENT; // We are only expecting an ACK
        }

        //Given a packet and payload, send everything including CRC bytes via I2C port
        sfe_ublox_status_e sendCommand(ubxPacket outgoingUBX, ushort maxWait)
        {
            sfe_ublox_status_e retVal = sfe_ublox_status_e.SFE_UBLOX_STATUS_SUCCESS;

            calcChecksum(outgoingUBX); //Sets checksum A and B bytes of the packet

            if (_debug == true)
            {
                Console.WriteLine("Sending command...");
                //printPacket(outgoingUBX);
            }

            sendSerialCommand(outgoingUBX);

            if (maxWait > 0)
            {
                //Depending on what we just sent, either we need to look for an ACK or not
                if (outgoingUBX.cls == UBX_CLASS_CFG)
                {
                    if (_debug == true)
                    {
                        Console.WriteLine("sendCommand: Waiting for ACK response");
                    }
                    retVal = waitForACKResponse(outgoingUBX, outgoingUBX.cls, outgoingUBX.id, maxWait); //Wait for Ack response
                }
                else
                {
                    if (_debug == true)
                    {
                        Console.WriteLine("sendCommand: Waiting for No ACK response");
                    }
                    retVal = waitForNoACKResponse(outgoingUBX, outgoingUBX.cls, outgoingUBX.id, maxWait); //Wait for Ack response
                }
            }

            return retVal;
        }

        //Given a packet and payload, send everything including CRC bytesA via Serial port
        void sendSerialCommand(ubxPacket outgoingUBX)
        {
            //Write header bytes
            byte[] command = new byte[0];
            addByteToArray(ref command, UBX_SYNCH_1);
            addByteToArray(ref command, UBX_SYNCH_2);
            addByteToArray(ref command, outgoingUBX.cls);
            addByteToArray(ref command, outgoingUBX.id);
            addByteToArray(ref command, (byte)(outgoingUBX.len & 0xFF));
            addByteToArray(ref command, (byte)(outgoingUBX.len >> 8));
            command = command.Concat(outgoingUBX.payload.Take(outgoingUBX.len).ToArray()).ToArray();
            addByteToArray(ref command, outgoingUBX.checksumA);
            addByteToArray(ref command, outgoingUBX.checksumB);

            _serialPort.Write(command, 0, command.Length);
        }

        public void addByteToArray(ref byte[] bArray, byte newByte)
        {
            byte[] newArray = new byte[bArray.Length + 1];
            bArray.CopyTo(newArray, 0);
            newArray[bArray.Length] = newByte;
            bArray = newArray;
        }




        //Given a message, calc and store the two byte "8-Bit Fletcher" checksum over the entirety of the message
        //This is called before we send a command message
        void calcChecksum(ubxPacket msg)
        {
            msg.checksumA = 0;
            msg.checksumB = 0;

            msg.checksumA += msg.cls;
            msg.checksumB += msg.checksumA;

            msg.checksumA += msg.id;
            msg.checksumB += msg.checksumA;

            msg.checksumA += (byte)(msg.len & 0xFF);
            msg.checksumB += msg.checksumA;

            msg.checksumA += (byte)(msg.len >> 8);
            msg.checksumB += msg.checksumA;

            for (ushort i = 0; i < msg.len; i++)
            {
                msg.checksumA += msg.payload[i];
                msg.checksumB += msg.checksumA;
            }
        }

        long millis() => DateTimeOffset.Now.ToUnixTimeMilliseconds();

        sfe_ublox_status_e waitForACKResponse(ubxPacket outgoingUBX, byte requestedClass, byte requestedID, ushort maxTime)
        {
            outgoingUBX.valid = sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; //This will go VALID (or NOT_VALID) when we receive a response to the packet we sent
            packetAck.valid = sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
            packetBuf.valid = sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
            outgoingUBX.classAndIDmatch = sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or NOT_VALID) when we receive a packet that matches the requested class and ID
            packetAck.classAndIDmatch = sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
            packetBuf.classAndIDmatch = sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;

            long startTime = millis();
            while (millis() - startTime < maxTime)
            {
                if (checkUbloxSerial(outgoingUBX, requestedClass, requestedID) == true) //See if new data is available. Process bytes as they come in.
                {
                    // If both the outgoingUBX->classAndIDmatch and packetAck.classAndIDmatch are VALID
                    // and outgoingUBX->valid is _still_ VALID and the class and ID _still_ match
                    // then we can be confident that the data in outgoingUBX is valid
                    if ((outgoingUBX.classAndIDmatch == sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX.valid == sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX.cls == requestedClass) && (outgoingUBX.id == requestedID))
                    {
                        if (_debug == true)
                        {
                            Console.WriteLine("waitForACKResponse: valid data and valid ACK received after " + (millis() - startTime) + " msec");
                        }
                        return (sfe_ublox_status_e.SFE_UBLOX_STATUS_DATA_RECEIVED); //We received valid data and a correct ACK!
                    }

                    // We can be confident that the data packet (if we are going to get one) will always arrive
                    // before the matching ACK. So if we sent a config packet which only produces an ACK
                    // then outgoingUBX->classAndIDmatch will be NOT_DEFINED and the packetAck.classAndIDmatch will VALID.
                    // We should not check outgoingUBX->valid, outgoingUBX->cls or outgoingUBX->id
                    // as these may have been changed by a PVT packet.
                    else if ((outgoingUBX.classAndIDmatch == sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (packetAck.classAndIDmatch == sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_VALID))
                    {
                        if (_debug == true)
                        {
                            Console.WriteLine("waitForACKResponse: valid ACK (without data) after " + (millis() - startTime) + " msec");
                        }
                        return sfe_ublox_status_e.SFE_UBLOX_STATUS_DATA_SENT; //We got an ACK but no data...
                    }

                    // If both the outgoingUBX->classAndIDmatch and packetAck.classAndIDmatch are VALID
                    // but the outgoingUBX->cls or ID no longer match then we can be confident that we had
                    // valid data but it has been or is currently being overwritten by another packet (e.g. PVT).
                    // If (e.g.) a PVT packet is _being_ received: outgoingUBX->valid will be NOT_DEFINED
                    // If (e.g.) a PVT packet _has been_ received: outgoingUBX->valid will be VALID (or just possibly NOT_VALID)
                    // So we cannot use outgoingUBX->valid as part of this check.
                    // Note: the addition of packetBuf should make this check redundant!
                    else if ((outgoingUBX.classAndIDmatch == sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_VALID) && ((outgoingUBX.cls != requestedClass) || (outgoingUBX.id != requestedID)))
                    {
                        if (_debug == true)
                        {
                            Console.WriteLine("waitForACKResponse: data being OVERWRITTEN after " + (millis() - startTime) + " msec");
                        }
                        return (sfe_ublox_status_e.SFE_UBLOX_STATUS_DATA_OVERWRITTEN); // Data was valid but has been or is being overwritten
                    }

                    // If packetAck.classAndIDmatch is VALID but both outgoingUBX->valid and outgoingUBX->classAndIDmatch
                    // are NOT_VALID then we can be confident we have had a checksum failure on the data packet
                    else if ((packetAck.classAndIDmatch == sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX.classAndIDmatch == sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) && (outgoingUBX.valid == sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_NOT_VALID))
                    {
                        if (_debug == true)
                        {
                            Console.WriteLine("waitForACKResponse: CRC failed after " + (millis() - startTime) + " msec");
                        }
                        return (sfe_ublox_status_e.SFE_UBLOX_STATUS_CRC_FAIL); //Checksum fail
                    }

                    // If our packet was not-acknowledged (NACK) we do not receive a data packet - we only get the NACK.
                    // So you would expect outgoingUBX->valid and outgoingUBX->classAndIDmatch to still be NOT_DEFINED
                    // But if a full PVT packet arrives afterwards outgoingUBX->valid could be VALID (or just possibly NOT_VALID)
                    // but outgoingUBX->cls and outgoingUBX->id would not match...
                    // So I think this is telling us we need a special state for packetAck.classAndIDmatch to tell us
                    // the packet was definitely NACK'd otherwise we are possibly just guessing...
                    // Note: the addition of packetBuf changes the logic of this, but we'll leave the code as is for now.
                    else if (packetAck.classAndIDmatch == sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_NOTACKNOWLEDGED)
                    {
                        if (_debug == true)
                        {
                            Console.WriteLine("waitForACKResponse: data was NOTACKNOWLEDGED (NACK) after " + (millis() - startTime) + " msec");
                        }
                        return (sfe_ublox_status_e.SFE_UBLOX_STATUS_COMMAND_NACK); //We received a NACK!
                    }

                    // If the outgoingUBX->classAndIDmatch is VALID but the packetAck.classAndIDmatch is NOT_VALID
                    // then the ack probably had a checksum error. We will take a gamble and return DATA_RECEIVED.
                    // If we were playing safe, we should return FAIL instead
                    else if ((outgoingUBX.classAndIDmatch == sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) && (outgoingUBX.valid == sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX.cls == requestedClass) && (outgoingUBX.id == requestedID))
                    {
                        if (_debug == true)
                        {
                            Console.WriteLine("waitForACKResponse: VALID data and INVALID ACK received after " + (millis() - startTime) + " msec");
                        }
                        return (sfe_ublox_status_e.SFE_UBLOX_STATUS_DATA_RECEIVED); //We received valid data and an invalid ACK!
                    }

                    // If the outgoingUBX->classAndIDmatch is NOT_VALID and the packetAck.classAndIDmatch is NOT_VALID
                    // then we return a FAIL. This must be a double checksum failure?
                    else if ((outgoingUBX.classAndIDmatch == sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) && (packetAck.classAndIDmatch == sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_NOT_VALID))
                    {
                        if (_debug == true)
                        {
                            Console.WriteLine("waitForACKResponse: INVALID data and INVALID ACK received after " + (millis() - startTime) + " msec");
                        }
                        return (sfe_ublox_status_e.SFE_UBLOX_STATUS_FAIL); //We received invalid data and an invalid ACK!
                    }

                    // If the outgoingUBX->classAndIDmatch is VALID and the packetAck.classAndIDmatch is NOT_DEFINED
                    // then the ACK has not yet been received and we should keep waiting for it
                    else if ((outgoingUBX.classAndIDmatch == sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED))
                    {
                        if (_debug == true)
                        {
                            Console.WriteLine("waitForACKResponse: valid data after " + (millis() - startTime) + " msec. Waiting for ACK.");
                        }
                    }

                } //checkUbloxInternal == true

                //delayMicroseconds(500);
                Thread.Sleep(1);
            } //while (millis() - startTime < maxTime)

            // We have timed out...
            // If the outgoingUBX->classAndIDmatch is VALID then we can take a gamble and return DATA_RECEIVED
            // even though we did not get an ACK
            if ((outgoingUBX.classAndIDmatch == sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (outgoingUBX.valid == sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX.cls == requestedClass) && (outgoingUBX.id == requestedID))
            {
                if (_debug == true)
                {
                    Console.WriteLine("waitForACKResponse: TIMEOUT with valid data after " + (millis() - startTime) + " msec. ");
                }
                return sfe_ublox_status_e.SFE_UBLOX_STATUS_DATA_RECEIVED; //We received valid data... But no ACK!
            }

            if (_debug == true)
            {
                Console.WriteLine("waitForACKResponse: TIMEOUT after " + (millis() - startTime) + " msec.");
            }

            return sfe_ublox_status_e.SFE_UBLOX_STATUS_TIMEOUT;
        }



        //For non-CFG queries no ACK is sent so we use this function
        //Returns SFE_UBLOX_STATUS_DATA_RECEIVED if we got a config packet full of response data that has CLS/ID match to our query packet
        //Returns SFE_UBLOX_STATUS_CRC_FAIL if we got a corrupt config packet that has CLS/ID match to our query packet
        //Returns SFE_UBLOX_STATUS_TIMEOUT if we timed out
        //Returns SFE_UBLOX_STATUS_DATA_OVERWRITTEN if we got an a valid packetCfg but that the packetCfg has been
        // or is currently being overwritten (remember that Serial data can arrive very slowly)
        sfe_ublox_status_e waitForNoACKResponse(ubxPacket outgoingUBX, byte requestedClass, byte requestedID, ushort maxTime)
        {
            outgoingUBX.valid = sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; //This will go VALID (or NOT_VALID) when we receive a response to the packet we sent
            packetAck.valid = sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
            packetBuf.valid = sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
            outgoingUBX.classAndIDmatch = sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or NOT_VALID) when we receive a packet that matches the requested class and ID
            packetAck.classAndIDmatch = sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
            packetBuf.classAndIDmatch = sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;

            long startTime = millis();
            while (millis() - startTime < maxTime)
            {
                if (checkUbloxSerial(outgoingUBX, requestedClass, requestedID) == true) //See if new data is available. Process bytes as they come in.
                {

                    // If outgoingUBX->classAndIDmatch is VALID
                    // and outgoingUBX->valid is _still_ VALID and the class and ID _still_ match
                    // then we can be confident that the data in outgoingUBX is valid
                    if ((outgoingUBX.classAndIDmatch == sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX.valid == sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX.cls == requestedClass) && (outgoingUBX.id == requestedID))
                    {
                        if (_debug == true)
                        {
                            Console.WriteLine("waitForNoACKResponse: valid data with CLS/ID match after " + (millis() - startTime) + " msec");
                        }
                        return sfe_ublox_status_e.SFE_UBLOX_STATUS_DATA_RECEIVED; //We received valid data!
                    }

                    // If the outgoingUBX->classAndIDmatch is VALID
                    // but the outgoingUBX->cls or ID no longer match then we can be confident that we had
                    // valid data but it has been or is currently being overwritten by another packet (e.g. PVT).
                    // If (e.g.) a PVT packet is _being_ received: outgoingUBX->valid will be NOT_DEFINED
                    // If (e.g.) a PVT packet _has been_ received: outgoingUBX->valid will be VALID (or just possibly NOT_VALID)
                    // So we cannot use outgoingUBX->valid as part of this check.
                    // Note: the addition of packetBuf should make this check redundant!
                    else if ((outgoingUBX.classAndIDmatch == sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_VALID) && ((outgoingUBX.cls != requestedClass) || (outgoingUBX.id != requestedID)))
                    {
                        if (_debug == true)
                        {
                            Console.WriteLine("waitForNoACKResponse: data being OVERWRITTEN after " + (millis() - startTime) + " msec");
                        }
                        return sfe_ublox_status_e.SFE_UBLOX_STATUS_DATA_OVERWRITTEN; // Data was valid but has been or is being overwritten
                    }

                    // If outgoingUBX->classAndIDmatch is NOT_DEFINED
                    // and outgoingUBX->valid is VALID then this must be (e.g.) a PVT packet
                    else if ((outgoingUBX.classAndIDmatch == sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (outgoingUBX.valid == sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_VALID))
                    {
                        if (_debug == true)
                        {
                            Console.WriteLine("waitForNoACKResponse: valid but UNWANTED data after ");
                        }
                    }

                    // If the outgoingUBX->classAndIDmatch is NOT_VALID then we return CRC failure
                    else if (outgoingUBX.classAndIDmatch == sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_NOT_VALID)
                    {
                        if (_debug == true)
                        {
                            Console.WriteLine("waitForNoACKResponse: CLS/ID match but failed CRC after "+(millis() - startTime)+" msec");
                        }
                        return sfe_ublox_status_e.SFE_UBLOX_STATUS_CRC_FAIL; //We received invalid data
                    }
                }

                Thread.Sleep(1);
            }

            if (_debug == true)
            {
                Console.WriteLine("waitForNoACKResponse: TIMEOUT after "+(millis() - startTime)+" msec. No packet received.");
            }

            return sfe_ublox_status_e.SFE_UBLOX_STATUS_TIMEOUT;
        }

        //Checks Serial for data, passing any new bytes to process()
        bool checkUbloxSerial(ubxPacket incomingUBX, byte requestedClass, byte requestedID)
        {
            while (_serialPort.BytesToRead > 0)
            {
                //create read buffer
                byte[] buff = new byte[MAX_PAYLOAD_SIZE];
                var bytesReceivedLen = _serialPort.Read(buff, 0, MAX_PAYLOAD_SIZE);

                for (int i = 0; i < bytesReceivedLen; i++) {
                    process(buff[i], incomingUBX, requestedClass, requestedID);
                }
            }
            return true;
        }

        //Called regularly to check for available bytes on the user' specified port
        public bool checkUblox(byte requestedClass = 0, byte requestedID = 0)
        {
            return checkUbloxSerial(packetCfg, requestedClass, requestedID);
        }

        //Processes NMEA and UBX binary sentences one byte at a time
        //Take a given byte and file it into the proper array
        void process(byte incoming, ubxPacket incomingUBX, byte requestedClass, byte requestedID)
        {
            if ((currentSentence == SentenceTypes.NONE) || (currentSentence == SentenceTypes.NMEA))
            {
                if (incoming == 0xB5) //UBX binary frames start with 0xB5, aka μ
                {
                    //This is the start of a binary sentence. Reset flags.
                    //We still don't know the response class
                    ubxFrameCounter = 0;
                    currentSentence = SentenceTypes.UBX;
                    //Reset the packetBuf.counter even though we will need to reset it again when ubxFrameCounter == 2
                    packetBuf.counter = 0;
                    ignoreThisPayload = false; //We should not ignore this payload - yet
                                               //Store data in packetBuf until we know if we have a requested class and ID match
                    activePacketBuffer = sfe_ublox_packet_buffer_e.SFE_UBLOX_PACKET_PACKETBUF;
                    //Console.WriteLine("UB message received");
                }
                else if (incoming == '$')
                {
                    currentSentence = SentenceTypes.NMEA;
                }
                else if (incoming == 0xD3) //RTCM frames start with 0xD3
                {
                    rtcmFrameCounter = 0;
                    currentSentence = SentenceTypes.RTCM;
                }
                else
                {
                    //This character is unknown or we missed the previous start of a sentence
                }
            }

            //Depending on the sentence, pass the character to the individual processor
            if (currentSentence == SentenceTypes.UBX)
            {
                //Decide what type of response this is
                if ((ubxFrameCounter == 0) && (incoming != 0xB5))      //ISO 'μ'
                    currentSentence = SentenceTypes.NONE;                              //Something went wrong. Reset.
                else if ((ubxFrameCounter == 1) && (incoming != 0x62)) //ASCII 'b'
                    currentSentence = SentenceTypes.NONE;                              //Something went wrong. Reset.
                                                                                           // Note to future self:
                                                                                           // There may be some duplication / redundancy in the next few lines as processUBX will also
                                                                                           // load information into packetBuf, but we'll do it here too for clarity
                else if (ubxFrameCounter == 2) //Class
                {
                    // Record the class in packetBuf until we know what to do with it
                    packetBuf.cls = incoming; // (Duplication)
                    rollingChecksumA = 0;     //Reset our rolling checksums here (not when we receive the 0xB5)
                    rollingChecksumB = 0;
                    packetBuf.counter = 0;                                   //Reset the packetBuf.counter (again)
                    packetBuf.valid = sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // Reset the packet validity (redundant?)
                    packetBuf.startingSpot = incomingUBX.startingSpot;      //Copy the startingSpot
                }
                else if (ubxFrameCounter == 3) //ID
                {
                    // Record the ID in packetBuf until we know what to do with it
                    packetBuf.id = incoming; // (Duplication)
                                             //We can now identify the type of response
                                             //If the packet we are receiving is not an ACK then check for a class and ID match
                    if (packetBuf.cls != UBX_CLASS_ACK)
                    {
                        //This is not an ACK so check for a class and ID match
                        if ((packetBuf.cls == requestedClass) && (packetBuf.id == requestedID))
                        {
                            //This is not an ACK and we have a class and ID match
                            //So start diverting data into incomingUBX (usually packetCfg)
                            activePacketBuffer = sfe_ublox_packet_buffer_e.SFE_UBLOX_PACKET_PACKETCFG;
                            incomingUBX.cls = packetBuf.cls; //Copy the class and ID into incomingUBX (usually packetCfg)
                            incomingUBX.id = packetBuf.id;
                            incomingUBX.counter = packetBuf.counter; //Copy over the .counter too
                        }
                        //This is not an ACK and we do not have a complete class and ID match
                        //So let's check for an HPPOSLLH message arriving when we were expecting PVT and vice versa
                        else if ((packetBuf.cls == requestedClass) &&
                          (((packetBuf.id == UBX_NAV_PVT) && (requestedID == UBX_NAV_HPPOSLLH)) ||
                          ((packetBuf.id == UBX_NAV_HPPOSLLH) && (requestedID == UBX_NAV_PVT))))
                        {
                            //This is not the message we were expecting but we start diverting data into incomingUBX (usually packetCfg) and process it anyway
                            activePacketBuffer = sfe_ublox_packet_buffer_e.SFE_UBLOX_PACKET_PACKETCFG;
                            incomingUBX.cls = packetBuf.cls; //Copy the class and ID into incomingUBX (usually packetCfg)
                            incomingUBX.id = packetBuf.id;
                            incomingUBX.counter = packetBuf.counter; //Copy over the .counter too
                        }
                        else
                        {
                            //This is not an ACK and we do not have a class and ID match
                            //so we should keep diverting data into packetBuf and ignore the payload
                            ignoreThisPayload = true;
                        }
                    }
                    else
                    {
                        // This is an ACK so it is to early to do anything with it
                        // We need to wait until we have received the length and data bytes
                        // So we should keep diverting data into packetBuf
                    }
                }
                else if (ubxFrameCounter == 4) //Length LSB
                {
                    //We should save the length in packetBuf even if activePacketBuffer == SFE_UBLOX_PACKET_PACKETCFG
                    packetBuf.len = incoming; // (Duplication)
                }
                else if (ubxFrameCounter == 5) //Length MSB
                {
                    //We should save the length in packetBuf even if activePacketBuffer == SFE_UBLOX_PACKET_PACKETCFG
                    packetBuf.len |= (short)(incoming << 8); // (Duplication)
                }
                else if (ubxFrameCounter == 6) //This should be the first byte of the payload unless .len is zero
                {
                    if (packetBuf.len == 0) // Check if length is zero (hopefully this is impossible!)
                    {
                        //If length is zero (!) this will be the first byte of the checksum so record it
                        packetBuf.checksumA = incoming;
                    }
                    else
                    {
                        //The length is not zero so record this byte in the payload
                        packetBuf.payload[0] = incoming;
                    }
                }
                else if (ubxFrameCounter == 7) //This should be the second byte of the payload unless .len is zero or one
                {
                    if (packetBuf.len == 0) // Check if length is zero (hopefully this is impossible!)
                    {
                        //If length is zero (!) this will be the second byte of the checksum so record it
                        packetBuf.checksumB = incoming;
                    }
                    else if (packetBuf.len == 1) // Check if length is one
                    {
                        //The length is one so this is the first byte of the checksum
                        packetBuf.checksumA = incoming;
                    }
                    else // Length is >= 2 so this must be a payload byte
                    {
                        packetBuf.payload[1] = incoming;
                    }
                    // Now that we have received two payload bytes, we can check for a matching ACK/NACK
                    if ((activePacketBuffer == sfe_ublox_packet_buffer_e.SFE_UBLOX_PACKET_PACKETBUF) // If we are not already processing a data packet
                        && (packetBuf.cls == UBX_CLASS_ACK)                // and if this is an ACK/NACK
                        && (packetBuf.payload[0] == requestedClass)        // and if the class matches
                        && (packetBuf.payload[1] == requestedID))          // and if the ID matches
                    {
                        if (packetBuf.len == 2) // Check if .len is 2
                        {
                            // Then this is a matching ACK so copy it into packetAck
                            activePacketBuffer = sfe_ublox_packet_buffer_e.SFE_UBLOX_PACKET_PACKETACK;
                            packetAck.cls = packetBuf.cls;
                            packetAck.id = packetBuf.id;
                            packetAck.len = packetBuf.len;
                            packetAck.counter = packetBuf.counter;
                            packetAck.payload[0] = packetBuf.payload[0];
                            packetAck.payload[1] = packetBuf.payload[1];
                        }
                        else // Length is not 2 (hopefully this is impossible!)
                        {

                        }
                    }
                }

                //Divert incoming into the correct buffer
                if (activePacketBuffer == sfe_ublox_packet_buffer_e.SFE_UBLOX_PACKET_PACKETACK)
                    processUBX(incoming, packetAck, requestedClass, requestedID);
                else if (activePacketBuffer == sfe_ublox_packet_buffer_e.SFE_UBLOX_PACKET_PACKETCFG)
                    processUBX(incoming, incomingUBX, requestedClass, requestedID);
                else // if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETBUF)
                    processUBX(incoming, packetBuf, requestedClass, requestedID);

                //Finally, increment the frame counter
                ubxFrameCounter++;
            }
            else if (currentSentence == SentenceTypes.NMEA)
            {
                processNMEA(incoming); //Process each NMEA character
            }
            else if (currentSentence == SentenceTypes.RTCM)
            {
                processRTCMframe(incoming); //Deal with RTCM bytes
            }
        }

        //Given a character, file it away into the uxb packet structure
        //Set valid to VALID or NOT_VALID once sentence is completely received and passes or fails CRC
        //The payload portion of the packet can be 100s of bytes but the max array
        //size is MAX_PAYLOAD_SIZE bytes. startingSpot can be set so we only record
        //a subset of bytes within a larger packet.
        void processUBX(byte incoming, ubxPacket incomingUBX, byte requestedClass, byte requestedID)
        {
            //Add all incoming bytes to the rolling checksum
            //Stop at len+4 as this is the checksum bytes to that should not be added to the rolling checksum
            if (incomingUBX.counter < incomingUBX.len + 4)
                addToChecksum(incoming);

            if (incomingUBX.counter == 0)
            {
                incomingUBX.cls = incoming;
            }
            else if (incomingUBX.counter == 1)
            {
                incomingUBX.id = incoming;
            }
            else if (incomingUBX.counter == 2) //Len LSB
            {
                incomingUBX.len = incoming;
            }
            else if (incomingUBX.counter == 3) //Len MSB
            {
                incomingUBX.len |= (byte)(incoming << 8);
            }
            else if (incomingUBX.counter == incomingUBX.len + 4) //ChecksumA
            {
                incomingUBX.checksumA = incoming;
            }
            else if (incomingUBX.counter == incomingUBX.len + 5) //ChecksumB
            {
                incomingUBX.checksumB = incoming;

                currentSentence = SentenceTypes.NONE; //We're done! Reset the sentence to being looking for a new start char

                //Validate this sentence
                if ((incomingUBX.checksumA == rollingChecksumA) && (incomingUBX.checksumB == rollingChecksumB))
                {
                    incomingUBX.valid = sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_VALID; // Flag the packet as valid

                    // Let's check if the class and ID match the requestedClass and requestedID
                    // Remember - this could be a data packet or an ACK packet
                    if ((incomingUBX.cls == requestedClass) && (incomingUBX.id == requestedID))
                    {
                        incomingUBX.classAndIDmatch = sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_VALID; // If we have a match, set the classAndIDmatch flag to valid
                    }

                    // If this is an ACK then let's check if the class and ID match the requestedClass and requestedID
                    else if ((incomingUBX.cls == UBX_CLASS_ACK) && (incomingUBX.id == UBX_ACK_ACK) && (incomingUBX.payload[0] == requestedClass) && (incomingUBX.payload[1] == requestedID))
                    {
                        incomingUBX.classAndIDmatch = sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_VALID; // If we have a match, set the classAndIDmatch flag to valid
                    }

                    // If this is a NACK then let's check if the class and ID match the requestedClass and requestedID
                    else if ((incomingUBX.cls == UBX_CLASS_ACK) && (incomingUBX.id == UBX_ACK_NACK) && (incomingUBX.payload[0] == requestedClass) && (incomingUBX.payload[1] == requestedID))
                    {
                        incomingUBX.classAndIDmatch = sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_NOTACKNOWLEDGED; // If we have a match, set the classAndIDmatch flag to NOTACKNOWLEDGED
                    }

                    //This is not an ACK and we do not have a complete class and ID match
                    //So let's check for an HPPOSLLH message arriving when we were expecting PVT and vice versa
                    else if ((incomingUBX.cls == requestedClass) &&
                      (((incomingUBX.id == UBX_NAV_PVT) && (requestedID == UBX_NAV_HPPOSLLH)) ||
                      ((incomingUBX.id == UBX_NAV_HPPOSLLH) && (requestedID == UBX_NAV_PVT))))


                    //We've got a valid packet, now do something with it but only if ignoreThisPayload is false
                    if (ignoreThisPayload == false)
                    {
                        processUBXpacket(incomingUBX);
                    }
                }
                else // Checksum failure
                {
                    incomingUBX.valid = sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_NOT_VALID;

                    // Let's check if the class and ID match the requestedClass and requestedID.
                    // This is potentially risky as we are saying that we saw the requested Class and ID
                    // but that the packet checksum failed. Potentially it could be the class or ID bytes
                    // that caused the checksum error!
                    if ((incomingUBX.cls == requestedClass) && (incomingUBX.id == requestedID))
                    {
                        incomingUBX.classAndIDmatch = sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_NOT_VALID; // If we have a match, set the classAndIDmatch flag to not valid
                    }
                    // If this is an ACK then let's check if the class and ID match the requestedClass and requestedID
                    else if ((incomingUBX.cls == UBX_CLASS_ACK) && (incomingUBX.payload[0] == requestedClass) && (incomingUBX.payload[1] == requestedID))
                    {
                        incomingUBX.classAndIDmatch = sfe_ublox_packet_validity_e.SFE_UBLOX_PACKET_VALIDITY_NOT_VALID; // If we have a match, set the classAndIDmatch flag to not valid
                    }
                }
            }
            else //Load this byte into the payload array
            {
                //If a UBX_NAV_PVT packet comes in asynchronously, we need to fudge the startingSpot
                short startingSpot = incomingUBX.startingSpot;
                if (incomingUBX.cls == UBX_CLASS_NAV && incomingUBX.id == UBX_NAV_PVT)
                    startingSpot = 0;
                //Begin recording if counter goes past startingSpot
                if ((incomingUBX.counter - 4) >= startingSpot)
                {
                    //Check to see if we have room for this byte
                    if (((incomingUBX.counter - 4) - startingSpot) < MAX_PAYLOAD_SIZE) //If counter = 208, starting spot = 200, we're good to record.
                    {
                        // Check if this is payload data which should be ignored
                        if (ignoreThisPayload == false)
                        {
                            incomingUBX.payload[incomingUBX.counter - 4 - startingSpot] = incoming; //Store this byte into payload array
                        }
                    }
                }
            }

            //Increment the counter
            incomingUBX.counter++;

            if (incomingUBX.counter == MAX_PAYLOAD_SIZE)
            {
                //Something has gone very wrong
                currentSentence = SentenceTypes.NONE; //Reset the sentence to being looking for a new start char
                //if (_printDebug == true)
                //{
                //    _debugSerial.println(F("processUBX: counter hit MAX_PAYLOAD_SIZE"));
                //}
            }
        }

        //Given a message and a byte, add to rolling "8-Bit Fletcher" checksum
        //This is used when receiving messages from module
        void addToChecksum(byte incoming)
        {
            rollingChecksumA += incoming;
            rollingChecksumB += rollingChecksumA;
        }

        //Once a packet has been received and validated, identify this packet's class/id and update internal flags
        //Note: if the user requests a PVT or a HPPOSLLH message using a custom packet, the data extraction will
        //      not work as expected beacuse extractLong etc are hardwired to packetCfg payloadCfg. Ideally
        //      extractLong etc should be updated so they receive a pointer to the packet buffer.
        void processUBXpacket(ubxPacket msg)
        {
            switch (msg.cls)
            {
                case UBX_CLASS_NAV:
                    if (msg.id == UBX_NAV_PVT && msg.len == 92)
                    {
                        //Parse various byte fields into global vars
                        const int startingSpot = 0; //fixed value used in processUBX

                        timeOfWeek = extractLong(0);
                        gpsMillisecond = (short)(extractLong(0) % 1000); //Get last three digits of iTOW
                        gpsYear = (short)extractInt(4);
                        gpsMonth = extractByte(6);
                        gpsDay = extractByte(7);
                        gpsHour = extractByte(8);
                        gpsMinute = extractByte(9);
                        gpsSecond = extractByte(10);
                        //gpsDateValid = extractByte(11) & 0x01;
                        //gpsTimeValid = (extractByte(11) & 0x02) >> 1;
                        gpsNanosecond = (int)extractLong(16); //Includes milliseconds

                        fixType = extractByte(20 - startingSpot);
                        //carrierSolution = extractByte(21 - startingSpot) >> 6; //Get 6th&7th bits of this byte
                        SIV = extractByte(23 - startingSpot);
                        longitude = (int)extractLong(24 - startingSpot);
                        latitude = (int)extractLong(28 - startingSpot);
                        altitude = (int)extractLong(32 - startingSpot);
                        altitudeMSL = (int)extractLong(36 - startingSpot);
                        groundSpeed = (int)extractLong(60 - startingSpot);
                        headingOfMotion = (int)extractLong(64 - startingSpot);
                        pDOP = (short)extractInt(76 - startingSpot);

                        //Mark all datums as fresh (not read before)
                        moduleQueried.gpsiTOW = true;
                        moduleQueried.gpsYear = true;
                        moduleQueried.gpsMonth = true;
                        moduleQueried.gpsDay = true;
                        moduleQueried.gpsHour = true;
                        moduleQueried.gpsMinute = true;
                        moduleQueried.gpsSecond = true;
                        moduleQueried.gpsDateValid = true;
                        moduleQueried.gpsTimeValid = true;
                        moduleQueried.gpsNanosecond = true;

                        moduleQueried.all = true;
                        moduleQueried.longitude = true;
                        moduleQueried.latitude = true;
                        moduleQueried.altitude = true;
                        moduleQueried.altitudeMSL = true;
                        moduleQueried.SIV = true;
                        moduleQueried.fixType = true;
                        moduleQueried.carrierSolution = true;
                        moduleQueried.groundSpeed = true;
                        moduleQueried.headingOfMotion = true;
                        moduleQueried.pDOP = true;
                    }
                    else if (msg.id == UBX_NAV_HPPOSLLH && msg.len == 36)
                    {
                        timeOfWeek = extractLong(4);
                        highResLongitude = (int)extractLong(8);
                        highResLatitude = (int)extractLong(12);
                        elipsoid = (int)extractLong(16);
                        meanSeaLevel = (int)extractLong(20);
                        highResLongitudeHp = extractSignedChar(24);
                        highResLatitudeHp = extractSignedChar(25);
                        elipsoidHp = extractSignedChar(26);
                        meanSeaLevelHp = extractSignedChar(27);
                        horizontalAccuracy = extractLong(28);
                        verticalAccuracy = extractLong(32);

                        highResModuleQueried.all = true;
                        highResModuleQueried.highResLatitude = true;
                        highResModuleQueried.highResLatitudeHp = true;
                        highResModuleQueried.highResLongitude = true;
                        highResModuleQueried.highResLongitudeHp = true;
                        highResModuleQueried.elipsoid = true;
                        highResModuleQueried.elipsoidHp = true;
                        highResModuleQueried.meanSeaLevel = true;
                        highResModuleQueried.meanSeaLevelHp = true;
                        highResModuleQueried.geoidSeparation = true;
                        highResModuleQueried.horizontalAccuracy = true;
                        highResModuleQueried.verticalAccuracy = true;
                        moduleQueried.gpsiTOW = true; // this can arrive via HPPOS too.
                    }
                    break;
            }
        }

        //This is the default or generic NMEA processor. We're only going to pipe the data to serial port so we can see it.
        //User could overwrite this function to pipe characters to nmea.process(c) of tinyGPS or MicroNMEA
        //Or user could pipe each character to a buffer, radio, etc.
        void processNMEA(byte incoming)
        {
            //If user has assigned an output port then pipe the characters there
            //if (_nmeaOutputPort != null)
            //    _nmeaOutputPort.WriteByte(incoming); //Echo this byte to the serial port

            if (_nmeaHandler != null) _nmeaHandler(incoming);

        }

        //We need to be able to identify an RTCM packet and then the length
        //so that we know when the RTCM message is completely received and we then start
        //listening for other sentences (like NMEA or UBX)
        //RTCM packet structure is very odd. I never found RTCM STANDARD 10403.2 but
        //http://d1.amobbs.com/bbs_upload782111/files_39/ourdev_635123CK0HJT.pdf is good
        //https://dspace.cvut.cz/bitstream/handle/10467/65205/F3-BP-2016-Shkalikava-Anastasiya-Prenos%20polohove%20informace%20prostrednictvim%20datove%20site.pdf?sequence=-1
        //Lead me to: https://forum.u-blox.com/index.php/4348/how-to-read-rtcm-messages-from-neo-m8p
        //RTCM 3.2 bytes look like this:
        //Byte 0: Always 0xD3
        //Byte 1: 6-bits of zero
        //Byte 2: 10-bits of length of this packet including the first two-ish header bytes, + 6.
        //byte 3 + 4 bits: Msg type 12 bits
        //Example: D3 00 7C 43 F0 ... / 0x7C = 124+6 = 130 bytes in this packet, 0x43F = Msg type 1087
        void processRTCMframe(byte incoming)
        {
            if (rtcmFrameCounter == 1)
            {
                rtcmLen = (ushort)((incoming & 0x03) << 8); //Get the last two bits of this byte. Bits 8&9 of 10-bit length
            }
            else if (rtcmFrameCounter == 2)
            {
                rtcmLen |= incoming; //Bits 0-7 of packet length
                rtcmLen += 6;        //There are 6 additional bytes of what we presume is header, msgType, CRC, and stuff
            }
            /*else if (rtcmFrameCounter == 3)
            {
              rtcmMsgType = incoming << 4; //Message Type, MS 4 bits
            }
            else if (rtcmFrameCounter == 4)
            {
              rtcmMsgType |= (incoming >> 4); //Message Type, bits 0-7
            }*/

            rtcmFrameCounter++;

            processRTCM(incoming); //Here is where we expose this byte to the user

            if (rtcmFrameCounter == rtcmLen)
            {
                //We're done!
                currentSentence = SentenceTypes.NONE; //Reset and start looking for next sentence type
            }
        }

        //This function is called for each byte of an RTCM frame
        //Ths user can overwrite this function and process the RTCM frame as they please
        //Bytes can be piped to Serial or other interface. The consumer could be a radio or the internet (Ntrip broadcaster)
        void processRTCM(byte incoming)
        {
            //Radio.sendReliable((String)incoming); //An example of passing this byte to a radio

            //_debugSerial->write(incoming); //An example of passing this byte out the serial port

            //Debug printing
            //  _debugSerial->print(F(" "));
            //  if(incoming < 0x10) _debugSerial->print(F("0"));
            //  if(incoming < 0x10) _debugSerial->print(F("0"));
            //  _debugSerial->print(incoming, HEX);
            //  if(rtcmFrameCounter % 16 == 0) _debugSerial->println();

            if (_rtcmHandler != null) _rtcmHandler(incoming);
        }

        //Given a spot in the payload array, extract four bytes and build a long
        uint extractLong(byte spotToStart)
        {
            uint val = 0;
            val |= (uint)payloadCfg[spotToStart + 0] << 8 * 0;
            val |= (uint)payloadCfg[spotToStart + 1] << 8 * 1;
            val |= (uint)payloadCfg[spotToStart + 2] << 8 * 2;
            val |= (uint)payloadCfg[spotToStart + 3] << 8 * 3;
            return (val);
        }

        //Given a spot in the payload array, extract two bytes and build an int
        ushort extractInt(byte spotToStart)
        {
            ushort val = 0;
            val |= (ushort)((ushort)(payloadCfg[spotToStart + 0] << 8) * 0);
            val |= (ushort)((ushort)(payloadCfg[spotToStart + 1] << 8) * 1);
            return (val);
        }

        //Given a spot, extract a byte from the payload
        byte extractByte(byte spotToStart)
        {
            return (payloadCfg[spotToStart]);
        }

        //Given a spot, extract a signed 8-bit value from the payload
        byte extractSignedChar(byte spotToStart)
        {
            return ((byte)payloadCfg[spotToStart]);
        }

        public void attachNMEAHandler(Func<byte, byte> handler) => _nmeaHandler = handler;
        public void attachRTCMHandler(Func<byte, byte> handler) => _rtcmHandler = handler;
    }
}
