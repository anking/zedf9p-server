using System.IO.Ports;
using System.Threading.Tasks;
using UBLOX.Models;

namespace UBLOX
{
    interface IUblox
    {

        //By default use the default I2C address, and use Wire port
        //bool begin(TwoWire &wirePort = Wire, byte deviceAddress = 0x42); //Returns true if module is detected
        //serialPort needs to be perviously initialized to correct baud rate
        //bool begin(Stream &serialPort); //Returns true if module is detected
        Task<bool> begin(SerialPort serialPort); //Returns true if module is detected

        //Control the size of the internal I2C transaction amount
        //void setI2CTransactionSize(byte bufferSize);
        //byte getI2CTransactionSize(void);

        //Set the max number of bytes set in a given I2C transaction
        //byte i2cTransactionSize = 32; //Default to ATmega328 limit

        //Returns true if device answers on _gpsI2Caddress address or via Serial
        //maxWait is only used for Serial
        Task<bool> isConnected(ushort maxWait = 1100);

        //Changed in V1.8.1: provides backward compatibility for the examples that call checkUblox directly
        //Will default to using packetCfg to look for explicit autoPVT packets so they get processed correctly by processUBX
        Task<bool> checkUblox(byte requestedClass = Constants.UBX_CLASS_NAV, byte requestedID = Constants.UBX_NAV_PVT); //Checks module with user selected commType

        //bool checkUbloxI2C(ubxPacket incomingUBX, byte requestedClass, byte requestedID);    //Method for I2C polling of data, passing any new bytes to process()
        Task<bool> checkUbloxSerial(ubxPacket incomingUBX, byte requestedClass, byte requestedID); //Method for serial polling of data, passing any new bytes to process()

        Task process(byte incoming, ubxPacket incomingUBX, byte requestedClass, byte requestedID);    //Processes NMEA and UBX binary sentences one byte at a time
        void processUBX(byte incoming, ubxPacket incomingUBX, byte requestedClass, byte requestedID); //Given a character, file it away into the uxb packet structure
        void processRTCMframe(byte incoming);                                                                //Monitor the incoming bytes for start and length bytes
        void processRTCM(byte incoming); //__attribute__((weak));                                               //Given rtcm byte, do something with it. User can overwrite if desired to pipe bytes to radio, internet, etc.

        void processUBXpacket(ubxPacket msg);                 //Once a packet has been received and validated, identify this packet's class/id and update internal flags
        void processNMEA(char incoming); //__attribute__((weak)); //Given a NMEA character, do something with it. User can overwrite if desired to use something like tinyGPS or MicroNMEA libraries

        //void calcChecksum(ubxPacket msg);                                                         //Sets the checksumA and checksumB of a given messages
        Task<Enums.SfeUbloxStatus> sendCommand(ubxPacket outgoingUBX, ushort maxWait = Constants.DefaultMaxWait); //Given a packet and payload, send everything including CRC bytes, return true if we got a response
        //Enums.sfe_ublox_status_e sendI2cCommand(ubxPacket outgoingUBX, ushort maxWait = 250);
        //void sendSerialCommand(ubxPacket outgoingUBX);

        void printPacket(ubxPacket packet); //Useful for debugging

        Task factoryReset(); //Send factory reset sequence (i.e. load "default" configuration and perform hardReset)
        Task hardReset();    //Perform a reset leading to a cold start (zero info start-up)

        //bool setI2CAddress(byte deviceAddress, ushort maxTime = 250);                                        //Changes the I2C address of the u-blox module
        Task setSerialRate(uint baudrate, byte uartPort = Constants.COM_PORT_UART1, ushort maxTime = Constants.DefaultMaxWait); //Changes the serial baud rate of the u-blox module, uartPort should be COM_PORT_UART1/2
        //void setNMEAOutputPort(Stream &nmeaOutputPort);                                                              //Sets the internal variable for the port to direct NMEA characters to
        void setNMEAOutputPort(SerialPort nmeaOutputPort);                                                              //Sets the internal variable for the port to direct NMEA characters to

        Task<bool> setNavigationFrequency(byte navFreq, ushort maxWait = Constants.DefaultMaxWait);  //Set the number of nav solutions sent per second
        Task<byte> getNavigationFrequency(ushort maxWait = Constants.DefaultMaxWait);                   //Get the number of nav solutions sent per second currently being output by module
        Task<bool> saveConfiguration(ushort maxWait = Constants.DefaultMaxWait);                        //Save current configuration to flash and BBR (battery backed RAM)
        Task<bool> factoryDefault(ushort maxWait = Constants.DefaultMaxWait);                           //Reset module to factory defaults
        Task<bool> saveConfigSelective(uint configMask, ushort maxWait = Constants.DefaultMaxWait); //Save the selected configuration sub-sections to flash and BBR (battery backed RAM)

        //Enums.sfe_ublox_status_e waitForACKResponse(ubxPacket outgoingUBX, byte requestedClass, byte requestedID, ushort maxTime = Constants.defaultMaxWait);   //Poll the module until a config packet and an ACK is received
        //Enums.sfe_ublox_status_e waitForNoACKResponse(ubxPacket outgoingUBX, byte requestedClass, byte requestedID, ushort maxTime = Constants.defaultMaxWait); //Poll the module until a config packet is received

        // getPVT will only return data once in each navigation cycle. By default, that is once per second.
        // Therefore we should set getPVTmaxWait to slightly longer than that.
        // If you change the navigation frequency to (e.g.) 4Hz using setNavigationFrequency(4)
        // then you should use a shorter maxWait for getPVT. 300msec would be about right: getPVT(300)
        // The same is true for getHPPOSLLH.
        //#define getPVTmaxWait 1100		// Default maxWait for getPVT and all functions which call it
        //#define getHPPOSLLHmaxWait 1100 // Default maxWait for getHPPOSLLH and all functions which call it

        bool assumeAutoPVT(bool enabled, bool implicitUpdate = true);                          //In case no config access to the GPS is possible and PVT is send cyclically already
        Task<bool> setAutoPVT(bool enabled, ushort maxWait = Constants.DefaultMaxWait);                         //Enable/disable automatic PVT reports at the navigation frequency
        Task<bool> getPVT(ushort maxWait = Constants.getPVTmaxWait);                                               //Query module for latest group of datums and load global vars: lat, long, alt, speed, SIV, accuracies, etc. If autoPVT is disabled, performs an explicit poll and waits, if enabled does not block. Returns true if new PVT is available.
        Task<bool> setAutoPVT(bool enabled, bool implicitUpdate, ushort maxWait = Constants.DefaultMaxWait); //Enable/disable automatic PVT reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
        bool assumeAutoHPPOSLLH(bool enabled, bool implicitUpdate = true);                         //In case no config access to the GPS is possible and HPPOSLLH is send cyclically already
        Task<bool> setAutoHPPOSLLH(bool enabled, ushort maxWait = Constants.DefaultMaxWait);                            //Enable/disable automatic HPPOSLLH reports at the navigation frequency
        Task<bool> setAutoHPPOSLLH(bool enabled, bool implicitUpdate, ushort maxWait = Constants.DefaultMaxWait); //Enable/disable automatic HPPOSLLH reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
        Task<bool> getHPPOSLLH(ushort maxWait = Constants.getHPPOSLLHmaxWait);                                     //Query module for latest group of datums and load global vars: lat, long, alt, speed, SIV, accuracies, etc. If autoPVT is disabled, performs an explicit poll and waits, if enabled does not block. Returns true if new HPPOSLLH is available.
        void flushPVT();                                                                                //Mark all the PVT data as read/stale. This is handy to get data alignment after CRC failure
        void flushHPPOSLLH();                                                                               //Mark all the PVT data as read/stale. This is handy to get data alignment after CRC failure

        Task<int> getLatitude(ushort maxWait = Constants.getPVTmaxWait);            //Returns the current latitude in degrees * 10^-7. Auto selects between HighPrecision and Regular depending on ability of module.
        Task<int> getLongitude(ushort maxWait = Constants.getPVTmaxWait);           //Returns the current longitude in degrees * 10-7. Auto selects between HighPrecision and Regular depending on ability of module.
        Task<int> getAltitude(ushort maxWait = Constants.getPVTmaxWait);            //Returns the current altitude in mm above ellipsoid
        Task<int> getAltitudeMSL(ushort maxWait = Constants.getPVTmaxWait);         //Returns the current altitude in mm above mean sea level
        Task<byte> getSIV(ushort maxWait = Constants.getPVTmaxWait);                 //Returns number of sats used in fix
        Task<byte> getFixType(ushort maxWait = Constants.getPVTmaxWait);             //Returns the type of fix: 0=no, 3=3D, 4=GNSS+Deadreckoning
        Task<byte> getCarrierSolutionType(ushort maxWait = Constants.getPVTmaxWait); //Returns RTK solution: 0=no, 1=float solution, 2=fixed solution
        Task<int> getGroundSpeed(ushort maxWait = Constants.getPVTmaxWait);         //Returns speed in mm/s
        Task<int> getHeading(ushort maxWait = Constants.getPVTmaxWait);             //Returns heading in degrees * 10^-5
        Task<ushort> getPDOP(ushort maxWait = Constants.getPVTmaxWait);               //Returns positional dillution of precision * 10^-2 (dimensionless)
        Task<ushort> getYear(ushort maxWait = Constants.getPVTmaxWait);
        Task<byte> getMonth(ushort maxWait = Constants.getPVTmaxWait);
        Task<byte> getDay(ushort maxWait = Constants.getPVTmaxWait);
        Task<byte> getHour(ushort maxWait = Constants.getPVTmaxWait);
        Task<byte> getMinute(ushort maxWait = Constants.getPVTmaxWait);
        Task<byte> getSecond(ushort maxWait = Constants.getPVTmaxWait);
        Task<ushort> getMillisecond(ushort maxWait = Constants.getPVTmaxWait);
        Task<int> getNanosecond(ushort maxWait = Constants.getPVTmaxWait);
        Task<uint> getTimeOfWeek(ushort maxWait = Constants.getPVTmaxWait);
        Task<bool> getDateValid(ushort maxWait = Constants.getPVTmaxWait);
        Task<bool> getTimeValid(ushort maxWait = Constants.getPVTmaxWait);

        Task<int> getHighResLatitude(ushort maxWait = Constants.getHPPOSLLHmaxWait);
        Task<byte> getHighResLatitudeHp(ushort maxWait = Constants.getHPPOSLLHmaxWait);
        Task<int> getHighResLongitude(ushort maxWait = Constants.getHPPOSLLHmaxWait);
        Task<byte> getHighResLongitudeHp(ushort maxWait = Constants.getHPPOSLLHmaxWait);
        Task<int> getElipsoid(ushort maxWait = Constants.getHPPOSLLHmaxWait);
        Task<byte> getElipsoidHp(ushort maxWait = Constants.getHPPOSLLHmaxWait);
        Task<int> getMeanSeaLevel(ushort maxWait = Constants.getHPPOSLLHmaxWait);
        Task<byte> getMeanSeaLevelHp(ushort maxWait = Constants.getHPPOSLLHmaxWait);
        Task<int> getGeoidSeparation(ushort maxWait = Constants.getHPPOSLLHmaxWait);
        Task<uint> getHorizontalAccuracy(ushort maxWait = Constants.getHPPOSLLHmaxWait);
        Task<uint> getVerticalAccuracy(ushort maxWait = Constants.getHPPOSLLHmaxWait);

        //Port configurations
        Task<bool> setPortOutput(byte portID, byte comSettings, ushort maxWait = Constants.DefaultMaxWait); //Configure a given port to output UBX, NMEA, RTCM3 or a combination thereof
        Task<bool> setPortInput(byte portID, byte comSettings, ushort maxWait = Constants.DefaultMaxWait);  //Configure a given port to input UBX, NMEA, RTCM3 or a combination thereof
        Task<bool> getPortSettings(byte portID, ushort maxWait = Constants.DefaultMaxWait);                    //Returns the current protocol bits in the UBX-CFG-PRT command for a given port

        //bool setI2COutput(byte comSettings, ushort maxWait = 250);              //Configure I2C port to output UBX, NMEA, RTCM3 or a combination thereof
        Task<bool> setUART1Output(byte comSettings, ushort maxWait = Constants.DefaultMaxWait); //Configure UART1 port to output UBX, NMEA, RTCM3 or a combination thereof
        Task<bool> setUART2Output(byte comSettings, ushort maxWait = Constants.DefaultMaxWait); //Configure UART2 port to output UBX, NMEA, RTCM3 or a combination thereof
        Task<bool> setUSBOutput(byte comSettings, ushort maxWait = 250);              //Configure USB port to output UBX, NMEA, RTCM3 or a combination thereof
        Task<bool> setSPIOutput(byte comSettings, ushort maxWait = 250);              //Configure SPI port to output UBX, NMEA, RTCM3 or a combination thereof

        //Functions to turn on/off message types for a given port ID (see COM_PORT_I2C, etc above)
        Task<bool> configureMessage(byte msgClass, byte msgID, byte portID, byte sendRate, ushort maxWait = Constants.DefaultMaxWait);
        Task<bool> enableMessage(byte msgClass, byte msgID, byte portID, byte sendRate = 1, ushort maxWait = Constants.DefaultMaxWait);
        Task<bool> disableMessage(byte msgClass, byte msgID, byte portID, ushort maxWait = Constants.DefaultMaxWait);
        Task<bool> enableNMEAMessage(byte msgID, byte portID, byte sendRate = 1, ushort maxWait = Constants.DefaultMaxWait);
        Task<bool> disableNMEAMessage(byte msgID, byte portID, ushort maxWait = Constants.DefaultMaxWait);
        Task<bool> enableRTCMmessage(byte messageNumber, byte portID, byte sendRate, ushort maxWait = Constants.DefaultMaxWait); //Given a message number turns on a message ID for output over given PortID
        Task<bool> disableRTCMmessage(byte messageNumber, byte portID, ushort maxWait = Constants.DefaultMaxWait);                  //Turn off given RTCM message from a given port

        //General configuration (used only on protocol v27 and higher - ie, ZED-F9P)
        //It is probably safe to assume that users of the ZED-F9P will be using I2C / Qwiic.
        //If they are using Serial then the higher baud rate will also help. So let's leave maxWait set to 250ms.
        uint createKey(ushort group, ushort id, byte size); //Form 32-bit key from group/id/size

        Task<Enums.SfeUbloxStatus> getVal(uint keyID, byte layer = Constants.VAL_LAYER_RAM, ushort maxWait = 250);                    //Load payload with response
        Task<byte> getVal8(uint keyID, byte layer = Constants.VAL_LAYER_RAM, ushort maxWait = 250);                              //Returns the value at a given key location
        Task<ushort> getVal16(uint keyID, byte layer = Constants.VAL_LAYER_RAM, ushort maxWait = 250);                            //Returns the value at a given key location
        Task<uint> getVal32(uint keyID, byte layer = Constants.VAL_LAYER_RAM, ushort maxWait = 250);                            //Returns the value at a given key location
        Task<byte> getVal8(ushort group, ushort id, byte size, byte layer = Constants.VAL_LAYER_RAM, ushort maxWait = 250);   //Returns the value at a given group/id/size location
        Task<ushort> getVal16(ushort group, ushort id, byte size, byte layer = Constants.VAL_LAYER_RAM, ushort maxWait = 250); //Returns the value at a given group/id/size location
        Task<uint> getVal32(ushort group, ushort id, byte size, byte layer = Constants.VAL_LAYER_RAM, ushort maxWait = 250); //Returns the value at a given group/id/size location
        Task<byte> setVal(uint keyID, ushort value, byte layer = Constants.VAL_LAYER_ALL, ushort maxWait = 250);               //Sets the 16-bit value at a given group/id/size location
        Task<bool> setVal8(uint keyID, byte value, byte layer = Constants.VAL_LAYER_ALL, ushort maxWait = 250);               //Sets the 8-bit value at a given group/id/size location
        Task<bool> setVal16(uint keyID, ushort value, byte layer = Constants.VAL_LAYER_ALL, ushort maxWait = 250);             //Sets the 16-bit value at a given group/id/size location
        Task<bool> setVal32(uint keyID, uint value, byte layer = Constants.VAL_LAYER_ALL, ushort maxWait = 250);             //Sets the 32-bit value at a given group/id/size location
        Task<bool> newCfgValset8(uint keyID, byte value, byte layer = Constants.VAL_LAYER_BBR);                                 //Define a new UBX-CFG-VALSET with the given KeyID and 8-bit value
        Task<bool> newCfgValset16(uint keyID, ushort value, byte layer = Constants.VAL_LAYER_BBR);                               //Define a new UBX-CFG-VALSET with the given KeyID and 16-bit value
        bool newCfgValset32(uint keyID, uint value, byte layer = Constants.VAL_LAYER_BBR);                               //Define a new UBX-CFG-VALSET with the given KeyID and 32-bit value
        bool addCfgValset8(uint keyID, byte value);                                                                //Add a new KeyID and 8-bit value to an existing UBX-CFG-VALSET ubxPacket
        bool addCfgValset16(uint keyID, ushort value);                                                              //Add a new KeyID and 16-bit value to an existing UBX-CFG-VALSET ubxPacket
        Task<bool> addCfgValset32(uint keyID, uint value);                                                              //Add a new KeyID and 32-bit value to an existing UBX-CFG-VALSET ubxPacket
        Task<bool> sendCfgValset8(uint keyID, byte value, ushort maxWait = 250);                                       //Add the final KeyID and 8-bit value to an existing UBX-CFG-VALSET ubxPacket and send it
        Task<bool> sendCfgValset16(uint keyID, ushort value, ushort maxWait = 250);                                     //Add the final KeyID and 16-bit value to an existing UBX-CFG-VALSET ubxPacket and send it
        Task<bool> sendCfgValset32(uint keyID, uint value, ushort maxWait = 250);                                     //Add the final KeyID and 32-bit value to an existing UBX-CFG-VALSET ubxPacket and send it

        //Functions used for RTK and base station setup
        //It is probably safe to assume that users of the RTK will be using I2C / Qwiic. So let's leave maxWait set to 250ms.
        Task<SurveyMode> getTmode3(ushort maxWait = 250);                                                                 //Get the current TimeMode3 settings
        Task<bool> setSurveyMode(byte mode, ushort observationTime, float requiredAccuracy, ushort maxWait = 250); //Control survey in mode
        Task<bool> enableFixedMode(int latitude, int longitude, int altitude, ushort maxWait = 250);                   //Set fixed mode for GPS, will need lat, lng and altitude provided
        Task<bool> enableSurveyMode(ushort observationTime, float requiredAccuracy, ushort maxWait = 250);            //Begin Survey-In for NEO-M8P
        Task<bool> disableSurveyMode(ushort maxWait = 250);                                                             //Stop Survey-In mode

        Task<bool> getSurveyStatus(ushort maxWait); //Reads survey in status and sets the global variables

        Task<uint> getPositionAccuracy(ushort maxWait = 1100); //Returns the 3D accuracy of the current high-precision fix, in mm. Supported on NEO-M8P, ZED-F9P,

        Task<byte> getProtocolVersionHigh(ushort maxWait = 500); //Returns the PROTVER XX.00 from UBX-MON-VER register
        Task<byte> getProtocolVersionLow(ushort maxWait = 500);  //Returns the PROTVER 00.XX from UBX-MON-VER register
        Task<bool> getProtocolVersion(ushort maxWait = 500);     //Queries module, loads low/high bytes

        Task<bool> getRELPOSNED(ushort maxWait = 1100); //Get Relative Positioning Information of the NED frame

        void enableDebugging(SerialPort debugPort, bool printLimitedDebug = false); //Given a port to print to, enable debug messages. Default to all, not limited.
        void disableDebugging();                                                         //Turn off debug statements
        void debugPrint(string message);                                                      //Safely print debug statements
        void debugPrintln(string message);                                                    //Safely print debug statements
        //const char statusString(sfe_ublox_status_e stat);                                   //Pretty print the return value

        //Support for geofences
        Task<bool> addGeofence(int latitude, int longitude, uint radius, byte confidence = 0, byte pinPolarity = 0, byte pin = 0, ushort maxWait = 1100); // Add a new geofence
        Task<bool> clearGeofences(ushort maxWait = 1100);                                                                                                             //Clears all geofences
        Task<bool> getGeofenceState(geofenceState currentGeofenceState, ushort maxWait = 1100);                                                                      //Returns the combined geofence state
        Task<bool> clearAntPIO(ushort maxWait = 1100);                                                                                                                //Clears the antenna control pin settings to release the PIOs
        //geofenceParams currentGeofenceParams;                                                                                                                        // Global to store the geofence parameters

        Task<bool> powerSaveMode(bool power_save = true, ushort maxWait = 1100);
        Task<byte> getPowerSaveMode(ushort maxWait = 1100); // Returns 255 if the sendCommand fails
        Task<bool> powerOff(uint durationInMs, ushort maxWait = 1100);
        Task<bool> powerOffWithInterrupt(uint durationInMs, uint wakeupSources = Constants.VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT0, bool forceWhileUsb = true, ushort maxWait = 1100);

        //Change the dynamic platform model using UBX-CFG-NAV5
        Task<bool> setDynamicModel(Enums.DynModel newDynamicModel = Enums.DynModel.DYN_MODEL_PORTABLE, ushort maxWait = 1100);
        Task<byte> getDynamicModel(ushort maxWait = 1100); // Get the dynamic model - returns 255 if the sendCommand fails

        Task<bool> getEsfInfo(ushort maxWait = 1100);
        Task<bool> getEsfIns(ushort maxWait = 1100);
        Task<bool> getEsfDataInfo(ushort maxWait = 1100);
        Task<bool> getEsfRawDataInfo(ushort maxWait = 1100);
        Task<Enums.SfeUbloxStatus> getSensState(byte sensor, ushort maxWait = 1100);
        Task<bool> getVehAtt(ushort maxWait = 1100);

        // Given coordinates, put receiver into static position. Set latlong to true to pass in lat/long values instead of ecef.
        // For ECEF the units are: cm, 0.1mm, cm, 0.1mm, cm, 0.1mm
        // For Lat/Lon/Alt the units are: degrees^-7, degrees^-9, degrees^-7, degrees^-9, cm, 0.1mm
        Task<bool> setStaticPosition(int ecefXOrLat, byte ecefXOrLatHP, int ecefYOrLon, byte ecefYOrLonHP, int ecefZOrAlt, byte ecefZOrAltHP, bool latLong = false, ushort maxWait = 250);
        Task<bool> setStaticPosition(int ecefXOrLat, int ecefYOrLon, int ecefZOrAlt, bool latLong = false, ushort maxWait = 250);
    }
}