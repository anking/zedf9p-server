using System;
using System.IO.Ports;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using UBLOX.Models;
using UBLOX.Enums;

namespace UBLOX
{
    class SFE_UBLOX_GPS : IUblox
    {
        geofenceParams currentGeofenceParams = new geofenceParams();
        moduleQueried moduleQueried = new moduleQueried();
        highResModuleQueried highResModuleQueried = new highResModuleQueried();
        frelPosInfoStructure relPosInfo = new frelPosInfoStructure();
        deadReckData imuMeas = new deadReckData();
        indivImuData ubloxSen = new indivImuData();
        vehicleAttitude vehAtt = new vehicleAttitude();
        CommTypes commType = CommTypes.COMM_TYPE_SERIAL;
        SentenceTypes currentSentence = SentenceTypes.NONE;

        //sentence handlers
        Func<char, Task> _nmeaHandler = null;
        Func<byte, Task> _rtcmHandler = null;

        //Variables
        //TwoWire* _i2cPort;              //The generic connection to user's chosen I2C hardware
        SerialPort _serialPort;            //The generic connection to user's chosen Serial hardware
        SerialPort _nmeaOutputPort; //The user can assign an output port to print NMEA sentences if they wish
        SerialPort _debugSerial;            //The stream to send debug messages to if enabled

        byte _gpsI2Caddress = 0x42; //Default 7-bit unshifted address of the ublox 6/7/8/M8/F9 series
                                       //This can be changed using the ublox configuration software

        bool _printDebug = false;        //Flag to print the serial commands we are sending to the Serial port for debug
        bool _printLimitedDebug = false; //Flag to print limited debug messages. Useful for I2C debugging or high navigation rates

        //The packet buffers
        //These are pointed at from within the ubxPacket
        static byte[] payloadAck = new byte[2];                // Holds the requested ACK/NACK
        static byte[] payloadCfg = new byte[Constants.MAX_PAYLOAD_SIZE]; // Holds the requested data packet
        static byte[] payloadBuf = new byte[2];                // Temporary buffer used to screen incoming packets or dump unrequested packets

        //Init the packet structures and init them with pointers to the payloadAck, payloadCfg and payloadBuf arrays
        //Models.ubxPacket packetAck = { 0, 0, 0, 0, 0, payloadAck, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED };
        //Models.ubxPacket packetCfg = { 0, 0, 0, 0, 0, payloadCfg, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED };
        //Models.ubxPacket packetBuf = { 0, 0, 0, 0, 0, payloadBuf, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED };
        ubxPacket packetAck = new ubxPacket() { payload = payloadAck };
        ubxPacket packetCfg = new ubxPacket() { payload = payloadCfg };
        ubxPacket packetBuf = new ubxPacket() { payload = payloadBuf };


        //Flag if this packet is unrequested (and so should be ignored and not copied into packetCfg or packetAck)
        bool ignoreThisPayload = false;

        //Identify which buffer is in use
        //Data is stored in packetBuf until the requested class and ID can be validated
        //If a match is seen, data is diverted into packetAck or packetCfg
        SfeUbloxPacketBuffer activePacketBuffer = SfeUbloxPacketBuffer.SFE_UBLOX_PACKET_PACKETBUF;

        //Limit checking of new data to every X ms
        //If we are expecting an update every X Hz then we should check every half that amount of time
        //Otherwise we may block ourselves from seeing new data
        byte i2cPollingWait = 100; //Default to 100ms. Adjusted when user calls setNavigationFrequency()

        long lastCheck = 0;
        bool autoPVT = false;              //Whether autoPVT is enabled or not
        bool autoPVTImplicitUpdate = true; // Whether autoPVT is triggered by accessing stale data (=true) or by a call to checkUblox (=false)
        bool autoHPPOSLLH = false;             //Whether autoHPPOSLLH is enabled or not
        bool autoHPPOSLLHImplicitUpdate = true; // Whether autoHPPOSLLH is triggered by accessing stale data (=true) or by a call to checkUblox (=false)
        ushort ubxFrameCounter;             //It counts all UBX frame. [Fixed header(2bytes), CLS(1byte), ID(1byte), length(2bytes), payload(x bytes), checksums(2bytes)]

        byte rollingChecksumA; //Rolls forward as we receive incoming bytes. Checked against the last two A/B checksum bytes
        byte rollingChecksumB; //Rolls forward as we receive incoming bytes. Checked against the last two A/B checksum bytes

        //The major datums we want to globally store
        ushort gpsYear;
        byte gpsMonth;
        byte gpsDay;
        byte gpsHour;
        byte gpsMinute;
        byte gpsSecond;
        ushort gpsMillisecond;
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
        ushort pDOP;           //Positional dilution of precision * 10^-2 (dimensionless)
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

        ushort rtcmFrameCounter = 0; //Tracks the type of incoming byte inside RTCM frame

        ushort rtcmLen = 0;

        public svinStructure svin;

        public SFE_UBLOX_GPS()
        {
            // Constructor
            currentGeofenceParams.numFences = 0; // Zero the number of geofences currently in use
            moduleQueried.versionNumber = 0;

            //if (checksumFailurePin >= 0)
            //{
            //    pinMode((byte)checksumFailurePin, OUTPUT);
            //    digitalWrite((byte)checksumFailurePin, HIGH);
            //}

            //Define the size of the I2C buffer based on the platform the user has
            //In general we found that most platforms use 32 bytes as the I2C buffer size. We could
            //implement platform gaurds here but as you can see, none currently benefit from >32
            //so we'll leave it up to the user to set it using setI2CTransactionSize if they will benefit from it
            // //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
            // #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)

            // i2cTransactionSize = 32;

            // #elif defined(__SAMD21G18A__)

            // i2cTransactionSize = 32;

            //#elif __MK20DX256__
            //Teensy

            // #elif defined(ARDUINO_ARCH_ESP32)

            // i2cTransactionSize = 32; //The ESP32 has an I2C buffer length of 128. We reduce it to 32 bytes to increase stability with the module

            // #endif
        }

        //Initialize the i2c port
        //bool begin(TwoWire &wirePort, byte deviceAddress)
        //{
        //    commType = COMM_TYPE_I2C;
        //    _i2cPort = &wirePort; //Grab which port the user wants us to use

        //    //We expect caller to begin their I2C port, with the speed of their choice external to the library
        //    //But if they forget, we start the hardware here.

        //    //We're moving away from the practice of starting Wire hardware in a library. This is to avoid cross platform issues.
        //    //ie, there are some platforms that don't handle multiple starts to the wire hardware. Also, every time you start the wire
        //    //hardware the clock speed reverts back to 100kHz regardless of previous Wire.setClocks().
        //    //_i2cPort.begin();

        //    _gpsI2Caddress = deviceAddress; //Store the I2C address from user

        //    return (isConnected());
        //}

        //Initialize the Serial port
        public async Task<bool> begin(SerialPort serialPort)
        {
            commType = CommTypes.COMM_TYPE_SERIAL;


            //_debug = debug;

            svin = new svinStructure();

            // Create a new SerialPort object with default settings.
            _serialPort = serialPort;

            return await isConnected();
        }

        //Sets the global size for I2C transactions
        //Most platforms use 32 bytes (the default) but this allows users to increase the transaction
        //size if the platform supports it
        //Note: If the transaction size is set larger than the platforms buffer size, bad things will happen.
        //void setI2CTransactionSize(byte transactionSize)
        //{
        //    i2cTransactionSize = transactionSize;
        //}
        //byte getI2CTransactionSize(void)
        //{
        //    return (i2cTransactionSize);
        //}

        //Enable or disable the printing of sent/response HEX values.
        //Use this in conjunction with 'Transport Logging' from the Universal Reader Assistant to see what they're doing that we're not
        public void enableDebugging(SerialPort debugPort, bool printLimitedDebug)
        {
            _debugSerial = debugPort; //Grab which port the user wants us to use for debugging
            if (printLimitedDebug == false)
            {
                _printDebug = true; //Should we print the commands we send? Good for debugging
            }
            else
            {
                _printLimitedDebug = true; //Should we print limited debug messages? Good for debugging high navigation rates
            }
        }
        public void disableDebugging()
        {
            _printDebug = false; //Turn off extra print statements
            _printLimitedDebug = false;
        }

        //Safely print messages
        public void debugPrint(string message)
        {
            if (_printDebug == true)
            {
                _debugSerial.Write(message);
            }
        }
        //Safely print messages
        public void debugPrintln(string message)
        {
            if (_printDebug == true)
            {
                _debugSerial.WriteLine(message);
            }
        }

        public string statusString(SfeUbloxStatus stat)
        {
            switch (stat)
            {
                case SfeUbloxStatus.SFE_UBLOX_STATUS_SUCCESS:
                    return "Success";
                case SfeUbloxStatus.SFE_UBLOX_STATUS_FAIL:
                    return "General Failure";
                case SfeUbloxStatus.SFE_UBLOX_STATUS_CRC_FAIL:
                    return "CRC Fail";
                case SfeUbloxStatus.SFE_UBLOX_STATUS_TIMEOUT:
                    return "Timeout";
                case SfeUbloxStatus.SFE_UBLOX_STATUS_COMMAND_NACK:
                    return "Command not acknowledged (NACK)";
                case SfeUbloxStatus.SFE_UBLOX_STATUS_OUT_OF_RANGE:
                    return "Out of range";
                case SfeUbloxStatus.SFE_UBLOX_STATUS_INVALID_ARG:
                    return "Invalid Arg";
                case SfeUbloxStatus.SFE_UBLOX_STATUS_INVALID_OPERATION:
                    return "Invalid operation";
                case SfeUbloxStatus.SFE_UBLOX_STATUS_MEM_ERR:
                    return "Memory Error";
                case SfeUbloxStatus.SFE_UBLOX_STATUS_HW_ERR:
                    return "Hardware Error";
                case SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_SENT:
                    return "Data Sent";
                case SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_RECEIVED:
                    return "Data Received";
                case SfeUbloxStatus.SFE_UBLOX_STATUS_I2C_COMM_FAILURE:
                    return "I2C Comm Failure";
                case SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_OVERWRITTEN:
                    return "Data Packet Overwritten";
                default:
                    return "Unknown Status";
            }
        }

        public async Task factoryReset()
        {
            // Copy default settings to permanent
            // Note: this does not load the permanent configuration into the current configuration. Calling factoryDefault() will do that.
            packetCfg.cls = Constants.UBX_CLASS_CFG;
            packetCfg.id = Constants.UBX_CLASS_CFG;
            packetCfg.len = 13;
            packetCfg.startingSpot = 0;
            for (byte i = 0; i < 4; i++)
            {
                payloadCfg[0 + i] = 0xff; // clear mask: copy default config to permanent config
                payloadCfg[4 + i] = 0x00; // save mask: don't save current to permanent
                payloadCfg[8 + i] = 0x00; // load mask: don't copy permanent config to current
            }
            payloadCfg[12] = 0xff;      // all forms of permanent memory
            await sendCommand(packetCfg, 0); // don't expect ACK
            await hardReset();                // cause factory default config to actually be loaded and used cleanly
        }

        public async Task hardReset()
        {
            // Issue hard reset
            packetCfg.cls = Constants.UBX_CLASS_CFG;
            packetCfg.id = Constants.UBX_CFG_RST;
            packetCfg.len = 4;
            packetCfg.startingSpot = 0;
            payloadCfg[0] = 0xff;       // cold start
            payloadCfg[1] = 0xff;       // cold start
            payloadCfg[2] = 0;          // 0=HW reset
            payloadCfg[3] = 0;          // reserved

            await sendCommand(packetCfg, 0); // don't expect ACK
        }

        //Changes the serial baud rate of the u-blox module, can't return success/fail 'cause ACK from modem
        //is lost due to baud rate change
        public async Task setSerialRate(uint baudrate, byte uartPort, ushort maxWait)
        {
            //Get the current config values for the UART port
            await getPortSettings(uartPort, maxWait); //This will load the payloadCfg array with current port settings

            if (_printDebug == true)
            {
                _debugSerial.Write("Current baud rate: ");
                //_debugSerial.WriteLine(((uint)payloadCfg[10] << 16) | ((uint)payloadCfg[9] << 8) | payloadCfg[8]);
            }

            packetCfg.cls = Constants.UBX_CLASS_CFG;
            packetCfg.id = Constants.UBX_CFG_PRT;
            packetCfg.len = 20;
            packetCfg.startingSpot = 0;

            //payloadCfg is now loaded with current bytes. Change only the ones we need to
            //payloadCfg[8] = baudrate;
            //payloadCfg[9] = baudrate >> 8;
            //payloadCfg[10] = baudrate >> 16;
            //payloadCfg[11] = baudrate >> 24;
            BitConverter.GetBytes(baudrate).CopyTo(payloadCfg, 8);

            if (_printDebug == true)
            {
                _debugSerial.Write("New baud rate:");
                //_debugSerial.WriteLine(((uint)payloadCfg[10] << 16) | ((uint)payloadCfg[9] << 8) | payloadCfg[8]);
            }

            SfeUbloxStatus retVal = await sendCommand(packetCfg, maxWait);
            if (_printDebug == true)
            {
                _debugSerial.Write("setSerialRate: sendCommand returned: ");
                _debugSerial.WriteLine(statusString(retVal));
            }
        }

        //Changes the I2C address that the u-blox module responds to
        //0x42 is the default but can be changed with this command
        public async Task<bool> setI2CAddress(byte deviceAddress, ushort maxWait)
        {
            //Get the current config values for the I2C port
            await getPortSettings(Constants.COM_PORT_I2C, maxWait); //This will load the payloadCfg array with current port settings

            packetCfg.cls = Constants.UBX_CLASS_CFG;
            packetCfg.id = Constants.UBX_CFG_PRT;
            packetCfg.len = 20;
            packetCfg.startingSpot = 0;

            //payloadCfg is now loaded with current bytes. Change only the ones we need to
            payloadCfg[4] = (byte)(deviceAddress << 1); //DDC mode LSB

            if (await sendCommand(packetCfg, maxWait) == SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_SENT) // We are only expecting an ACK
            {
                //Success! Now change our internal global.
                _gpsI2Caddress = deviceAddress; //Store the I2C address from user
                return true;
            }
            return false;
        }

        //Want to see the NMEA messages on the Serial port? Here's how
        public void setNMEAOutputPort(SerialPort nmeaOutputPort)
        {
            _nmeaOutputPort = nmeaOutputPort; //Store the port from user
        }

        //Called regularly to check for available bytes on the user' specified port
        public async Task<bool> checkUblox(byte requestedClass = Constants.UBX_CLASS_NAV, byte requestedID = Constants.UBX_NAV_PVT)
        {
            return await checkUbloxInternal(packetCfg, requestedClass, requestedID);
        }

        //Called regularly to check for available bytes on the user' specified port
        public async Task<bool> checkUbloxInternal(ubxPacket incomingUBX, byte requestedClass, byte requestedID)
        {
            if (commType == CommTypes.COMM_TYPE_I2C)
                return false;
            //return checkUbloxI2C(incomingUBX, requestedClass, requestedID);
            else if (commType == CommTypes.COMM_TYPE_SERIAL)
                return await checkUbloxSerial(incomingUBX, requestedClass, requestedID);
            return false;
        }

        //Polls I2C for data, passing any new bytes to process()
        //Returns true if new bytes are available
        //bool checkUbloxI2C(ubxPacket incomingUBX, byte requestedClass, byte requestedID)
        //{
        //    if (millis() - lastCheck >= i2cPollingWait)
        //    {
        //        //Get the number of bytes available from the module
        //        ushort bytesAvailable = 0;
        //        _i2cPort.beginTransmission(_gpsI2Caddress);
        //        _i2cPort.write(0xFD);                     //0xFD (MSB) and 0xFE (LSB) are the registers that contain number of bytes available
        //        if (_i2cPort.endTransmission(false) != 0) //Send a restart command. Do not release bus.
        //            return false;                          //Sensor did not ACK

        //        _i2cPort.requestFrom((byte)_gpsI2Caddress, (byte)2);
        //        if (_i2cPort.available())
        //        {
        //            byte msb = _i2cPort.read();
        //            byte lsb = _i2cPort.read();
        //            if (lsb == 0xFF)
        //            {
        //                //I believe this is a u-blox bug. Device should never present an 0xFF.
        //                if ((_printDebug == true) || (_printLimitedDebug == true)) // Print this if doing limited debugging
        //                {
        //                    _debugSerial.WriteLine"checkUbloxI2C: u-blox bug, length lsb is 0xFF"));
        //                }
        //                if (checksumFailurePin >= 0)
        //                {
        //                    digitalWrite((byte)checksumFailurePin, LOW);
        //                    delay(10);
        //                    digitalWrite((byte)checksumFailurePin, HIGH);
        //                }
        //                lastCheck = millis(); //Put off checking to avoid I2C bus traffic
        //                return false;
        //            }
        //            bytesAvailable = (ushort)msb << 8 | lsb;
        //        }

        //        if (bytesAvailable == 0)
        //        {
        //            if (_printDebug == true)
        //            {
        //                _debugSerial.WriteLine"checkUbloxI2C: OK, zero bytes available"));
        //            }
        //            lastCheck = millis(); //Put off checking to avoid I2C bus traffic
        //            return false;
        //        }

        //        //Check for undocumented bit error. We found this doing logic scans.
        //        //This error is rare but if we incorrectly interpret the first bit of the two 'data available' bytes as 1
        //        //then we have far too many bytes to check. May be related to I2C setup time violations: https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library/issues/40
        //        if (bytesAvailable & ((ushort)1 << 15))
        //        {
        //            //Clear the MSbit
        //            bytesAvailable &= ~((ushort)1 << 15);

        //            if ((_printDebug == true) || (_printLimitedDebug == true)) // Print this if doing limited debugging
        //            {
        //                _debugSerial.Write("checkUbloxI2C: Bytes available error:"));
        //                _debugSerial.WriteLine(bytesAvailable);
        //                if (checksumFailurePin >= 0)
        //                {
        //                    digitalWrite((byte)checksumFailurePin, LOW);
        //                    delay(10);
        //                    digitalWrite((byte)checksumFailurePin, HIGH);
        //                }
        //            }
        //        }

        //        if (bytesAvailable > 100)
        //        {
        //            if (_printDebug == true)
        //            {
        //                _debugSerial.Write("checkUbloxI2C: Large packet of "));
        //                _debugSerial.Write((bytesAvailable);
        //                _debugSerial.WriteLine" bytes received"));
        //            }
        //        }
        //        else
        //        {
        //            if (_printDebug == true)
        //            {
        //                _debugSerial.Write("checkUbloxI2C: Reading "));
        //                _debugSerial.Write((bytesAvailable);
        //                _debugSerial.WriteLine" bytes"));
        //            }
        //        }

        //        while (bytesAvailable)
        //        {
        //            _i2cPort.beginTransmission(_gpsI2Caddress);
        //            _i2cPort.write(0xFF);                     //0xFF is the register to read data from
        //            if (_i2cPort.endTransmission(false) != 0) //Send a restart command. Do not release bus.
        //                return false;                          //Sensor did not ACK

        //            //Limit to 32 bytes or whatever the buffer limit is for given platform
        //            ushort bytesToRead = bytesAvailable;
        //            if (bytesToRead > i2cTransactionSize)
        //                bytesToRead = i2cTransactionSize;

        //            TRY_AGAIN:

        //            _i2cPort.requestFrom((byte)_gpsI2Caddress, (byte)bytesToRead);
        //            if (_i2cPort.available())
        //            {
        //                for (ushort x = 0; x < bytesToRead; x++)
        //                {
        //                    byte incoming = _i2cPort.read(); //Grab the actual character

        //                    //Check to see if the first read is 0x7F. If it is, the module is not ready
        //                    //to respond. Stop, wait, and try again
        //                    if (x == 0)
        //                    {
        //                        if (incoming == 0x7F)
        //                        {
        //                            if ((_printDebug == true) || (_printLimitedDebug == true)) // Print this if doing limited debugging
        //                            {
        //                                _debugSerial.WriteLine"checkUbloxU2C: u-blox error, module not ready with data"));
        //                            }
        //                            delay(5); //In logic analyzation, the module starting responding after 1.48ms
        //                            if (checksumFailurePin >= 0)
        //                            {
        //                                digitalWrite((byte)checksumFailurePin, LOW);
        //                                delay(10);
        //                                digitalWrite((byte)checksumFailurePin, HIGH);
        //                            }
        //                            goto TRY_AGAIN;
        //                        }
        //                    }

        //                    process(incoming, incomingUBX, requestedClass, requestedID); //Process this valid character
        //                }
        //            }
        //            else
        //                return false; //Sensor did not respond

        //            bytesAvailable -= bytesToRead;
        //        }
        //    }

        //    return true;

        //} //end checkUbloxI2C()

        //Checks Serial for data, passing any new bytes to process()
        //public bool checkUbloxSerial(ubxPacket incomingUBX, byte requestedClass, byte requestedID)
        //{
        //    while (_serialPort.IsOpen)
        //    {
        //        process(_serialPort.ReadByte(), incomingUBX, requestedClass, requestedID);
        //    }
        //    return true;

        //} //end checkUbloxSerial()

        //Checks Serial for data, passing any new bytes to process()
        public async Task<bool> checkUbloxSerial(ubxPacket incomingUBX, byte requestedClass, byte requestedID)
        {
            while (_serialPort.BytesToRead > 0)
            {
                //create read buffer
                byte[] buff = new byte[Constants.MAX_PAYLOAD_SIZE];
                var bytesReceivedLen = _serialPort.Read(buff, 0, Constants.MAX_PAYLOAD_SIZE);

                for (int i = 0; i < bytesReceivedLen; i++)
                {
                    await process(buff[i], incomingUBX, requestedClass, requestedID);
                }
            }
            return true;
        }

        //Processes NMEA and UBX binary sentences one byte at a time
        //Take a given byte and file it into the proper array
        public async Task process(byte incoming, ubxPacket incomingUBX, byte requestedClass, byte requestedID)
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
                    activePacketBuffer = SfeUbloxPacketBuffer.SFE_UBLOX_PACKET_PACKETBUF;
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
                    packetBuf.valid = SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // Reset the packet validity (redundant?)
                    packetBuf.startingSpot = incomingUBX.startingSpot;      //Copy the startingSpot
                }
                else if (ubxFrameCounter == 3) //ID
                {
                    // Record the ID in packetBuf until we know what to do with it
                    packetBuf.id = incoming; // (Duplication)
                                             //We can now identify the type of response
                                             //If the packet we are receiving is not an ACK then check for a class and ID match
                    if (packetBuf.cls != Constants.UBX_CLASS_ACK)
                    {
                        //This is not an ACK so check for a class and ID match
                        if ((packetBuf.cls == requestedClass) && (packetBuf.id == requestedID))
                        {
                            //This is not an ACK and we have a class and ID match
                            //So start diverting data into incomingUBX (usually packetCfg)
                            activePacketBuffer = SfeUbloxPacketBuffer.SFE_UBLOX_PACKET_PACKETCFG;
                            incomingUBX.cls = packetBuf.cls; //Copy the class and ID into incomingUBX (usually packetCfg)
                            incomingUBX.id = packetBuf.id;
                            incomingUBX.counter = packetBuf.counter; //Copy over the .counter too
                        }
                        //This is not an ACK and we do not have a complete class and ID match
                        //So let's check for an HPPOSLLH message arriving when we were expecting PVT and vice versa
                        else if ((packetBuf.cls == requestedClass) &&
                          (((packetBuf.id == Constants.UBX_NAV_PVT) && (requestedID == Constants.UBX_NAV_HPPOSLLH)) ||
                          ((packetBuf.id == Constants.UBX_NAV_HPPOSLLH) && (requestedID == Constants.UBX_NAV_PVT))))
                        {
                            //This is not the message we were expecting but we start diverting data into incomingUBX (usually packetCfg) and process it anyway
                            activePacketBuffer = SfeUbloxPacketBuffer.SFE_UBLOX_PACKET_PACKETCFG;
                            incomingUBX.cls = packetBuf.cls; //Copy the class and ID into incomingUBX (usually packetCfg)
                            incomingUBX.id = packetBuf.id;
                            incomingUBX.counter = packetBuf.counter; //Copy over the .counter too
                            if (_printDebug == true)
                            {
                                //_debugSerial.Write("process: auto PVT/HPPOSLLH collision: Requested ID: 0x"));
                                //_debugSerial.Write((requestedID, HEX);
                                //_debugSerial.Write(" Message ID: 0x"));
                                //_debugSerial.WriteLine(packetBuf.id, HEX);
                            }
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
                    //We should save the length in packetBuf even if activePacketBuffer == sfe_ublox_packet_buffer_e.SFE_UBLOX_PACKET_PACKETCFG
                    packetBuf.len = incoming; // (Duplication)
                }
                else if (ubxFrameCounter == 5) //Length MSB
                {
                    //We should save the length in packetBuf even if activePacketBuffer == sfe_ublox_packet_buffer_e.SFE_UBLOX_PACKET_PACKETCFG
                    //packetBuf.len |= incoming << 8; // (Duplication)
                    packetBuf.len |= (ushort)(incoming << 8); // (Duplication)
                }
                else if (ubxFrameCounter == 6) //This should be the first byte of the payload unless .len is zero
                {
                    if (packetBuf.len == 0) // Check if length is zero (hopefully this is impossible!)
                    {
                        if (_printDebug == true)
                        {
                            //_debugSerial.Write("process: ZERO LENGTH packet received: Class: 0x"));
                            //_debugSerial.Write((packetBuf.cls, HEX);
                            //_debugSerial.Write(" ID: 0x"));
                            //_debugSerial.WriteLine(packetBuf.id, HEX);
                        }
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
                    if ((activePacketBuffer == SfeUbloxPacketBuffer.SFE_UBLOX_PACKET_PACKETBUF) // If we are not already processing a data packet
                        && (packetBuf.cls == Constants.UBX_CLASS_ACK)                // and if this is an ACK/NACK
                        && (packetBuf.payload[0] == requestedClass)        // and if the class matches
                        && (packetBuf.payload[1] == requestedID))          // and if the ID matches
                    {
                        if (packetBuf.len == 2) // Check if .len is 2
                        {
                            // Then this is a matching ACK so copy it into packetAck
                            activePacketBuffer = SfeUbloxPacketBuffer.SFE_UBLOX_PACKET_PACKETACK;
                            packetAck.cls = packetBuf.cls;
                            packetAck.id = packetBuf.id;
                            packetAck.len = packetBuf.len;
                            packetAck.counter = packetBuf.counter;
                            packetAck.payload[0] = packetBuf.payload[0];
                            packetAck.payload[1] = packetBuf.payload[1];
                        }
                        else // Length is not 2 (hopefully this is impossible!)
                        {
                            if (_printDebug == true)
                            {
                                _debugSerial.Write("process: ACK received with .len != 2: Class: 0x");
                                //_debugSerial.Write(packetBuf.payload[0], HEX);
                                _debugSerial.Write(" ID: 0x");
                                //_debugSerial.Write(packetBuf.payload[1], HEX);
                                _debugSerial.Write(" len: ");
                                //_debugSerial.WriteLine(packetBuf.len);
                            }
                        }
                    }
                }

                //Divert incoming into the correct buffer
                if (activePacketBuffer == SfeUbloxPacketBuffer.SFE_UBLOX_PACKET_PACKETACK)
                    processUBX(incoming, packetAck, requestedClass, requestedID);
                else if (activePacketBuffer == SfeUbloxPacketBuffer.SFE_UBLOX_PACKET_PACKETCFG)
                    processUBX(incoming, incomingUBX, requestedClass, requestedID);
                else // if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETBUF)
                    processUBX(incoming, packetBuf, requestedClass, requestedID);

                //Finally, increment the frame counter
                ubxFrameCounter++;
            }
            else if (currentSentence == SentenceTypes.NMEA)
            {
                processNMEA(Convert.ToChar(incoming)); //Process each NMEA character
            }
            else if (currentSentence == SentenceTypes.RTCM)
            {
                processRTCMframe(incoming); //Deal with RTCM bytes
            }
        }

        //This is the default or generic NMEA processor. We're only going to pipe the data to serial port so we can see it.
        //User could overwrite this function to pipe characters to nmea.process(c) of tinyGPS or MicroNMEA
        //Or user could pipe each character to a buffer, radio, etc.
        public void processNMEA(char incoming)
        {
            //If user has assigned an output port then pipe the characters there
            //if (_nmeaOutputPort != null)
            //    _nmeaOutputPort.Write(incoming.ToString()); //Echo this byte to the serial port

            _nmeaHandler?.Invoke(incoming);
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
        public void processRTCMframe(byte incoming)
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
        public void processRTCM(byte incoming)
        {
            //Radio.sendReliable((String)incoming); //An example of passing this byte to a radio

            //_debugSerial.write(incoming); //An example of passing this byte out the serial port

            //Debug printing
            //  _debugSerial.Write(" "));
            //  if(incoming < 0x10) _debugSerial.Write("0"));
            //  if(incoming < 0x10) _debugSerial.Write("0"));
            //  _debugSerial.Write((incoming, HEX);
            //  if(rtcmFrameCounter % 16 == 0) _debugSerial.WriteLine();

            _rtcmHandler?.Invoke(incoming);
        }

        //Given a character, file it away into the uxb packet structure
        //Set valid to VALID or NOT_VALID once sentence is completely received and passes or fails CRC
        //The payload portion of the packet can be 100s of bytes but the max array
        //size is Constants.MAX_PAYLOAD_SIZE bytes. startingSpot can be set so we only record
        //a subset of bytes within a larger packet.
        public void processUBX(byte incoming, ubxPacket incomingUBX, byte requestedClass, byte requestedID)
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
                incomingUBX.len |= (ushort)(incoming << 8);
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
                    incomingUBX.valid = SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_VALID; // Flag the packet as valid

                    // Let's check if the class and ID match the requestedClass and requestedID
                    // Remember - this could be a data packet or an ACK packet
                    if ((incomingUBX.cls == requestedClass) && (incomingUBX.id == requestedID))
                    {
                        incomingUBX.classAndIDmatch = SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_VALID; // If we have a match, set the classAndIDmatch flag to valid
                    }

                    // If this is an ACK then let's check if the class and ID match the requestedClass and requestedID
                    else if ((incomingUBX.cls == Constants.UBX_CLASS_ACK) && (incomingUBX.id == Constants.UBX_ACK_ACK) && (incomingUBX.payload[0] == requestedClass) && (incomingUBX.payload[1] == requestedID))
                    {
                        incomingUBX.classAndIDmatch = SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_VALID; // If we have a match, set the classAndIDmatch flag to valid
                    }

                    // If this is a NACK then let's check if the class and ID match the requestedClass and requestedID
                    else if ((incomingUBX.cls == Constants.UBX_CLASS_ACK) && (incomingUBX.id == Constants.UBX_ACK_NACK) && (incomingUBX.payload[0] == requestedClass) && (incomingUBX.payload[1] == requestedID))
                    {
                        incomingUBX.classAndIDmatch = SfeUbloxPacketValidity.SFE_UBLOX_PACKET_NOTACKNOWLEDGED; // If we have a match, set the classAndIDmatch flag to NOTACKNOWLEDGED
                        if (_printDebug == true)
                        {
                            //_debugSerial.Write("processUBX: NACK received: Requested Class: 0x"));
                            //_debugSerial.Write((incomingUBX.payload[0], HEX);
                            //_debugSerial.Write(" Requested ID: 0x"));
                            //_debugSerial.WriteLine(incomingUBX.payload[1], HEX);
                        }
                    }

                    //This is not an ACK and we do not have a complete class and ID match
                    //So let's check for an HPPOSLLH message arriving when we were expecting PVT and vice versa
                    else if ((incomingUBX.cls == requestedClass) &&
                      (((incomingUBX.id == Constants.UBX_NAV_PVT) && (requestedID == Constants.UBX_NAV_HPPOSLLH)) ||
                      ((incomingUBX.id == Constants.UBX_NAV_HPPOSLLH) && (requestedID == Constants.UBX_NAV_PVT))))
                    {
                        // This isn't the message we are looking for...
                        // Let's say so and leave incomingUBX.classAndIDmatch _unchanged_
                        if (_printDebug == true)
                        {
                            //_debugSerial.Write("processUBX: auto PVT/HPPOSLLH collision: Requested ID: 0x"));
                            //_debugSerial.Write((requestedID, HEX);
                            //_debugSerial.Write(" Message ID: 0x"));
                            //_debugSerial.WriteLine(incomingUBX.id, HEX);
                        }
                    }

                    if (_printDebug == true)
                    {
                        //_debugSerial.Write("Incoming: Size: "));
                        //_debugSerial.Write((incomingUBX.len);
                        //_debugSerial.Write(" Received: "));
                        printPacket(incomingUBX);

                        if (incomingUBX.valid == SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_VALID)
                        {
                            _debugSerial.WriteLine("packetCfg now valid");
                        }
                        if (packetAck.valid == SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_VALID)
                        {
                            _debugSerial.WriteLine("packetAck now valid");
                        }
                        if (incomingUBX.classAndIDmatch == SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_VALID)
                        {
                            _debugSerial.WriteLine("packetCfg classAndIDmatch");
                        }
                        if (packetAck.classAndIDmatch == SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_VALID)
                        {
                            _debugSerial.WriteLine("packetAck classAndIDmatch");
                        }
                    }

                    //We've got a valid packet, now do something with it but only if ignoreThisPayload is false
                    if (ignoreThisPayload == false)
                    {
                        processUBXpacket(incomingUBX);
                    }
                }
                else // Checksum failure
                {
                    incomingUBX.valid = SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_NOT_VALID;

                    // Let's check if the class and ID match the requestedClass and requestedID.
                    // This is potentially risky as we are saying that we saw the requested Class and ID
                    // but that the packet checksum failed. Potentially it could be the class or ID bytes
                    // that caused the checksum error!
                    if ((incomingUBX.cls == requestedClass) && (incomingUBX.id == requestedID))
                    {
                        incomingUBX.classAndIDmatch = SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_NOT_VALID; // If we have a match, set the classAndIDmatch flag to not valid
                    }
                    // If this is an ACK then let's check if the class and ID match the requestedClass and requestedID
                    else if ((incomingUBX.cls == Constants.UBX_CLASS_ACK) && (incomingUBX.payload[0] == requestedClass) && (incomingUBX.payload[1] == requestedID))
                    {
                        incomingUBX.classAndIDmatch = SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_NOT_VALID; // If we have a match, set the classAndIDmatch flag to not valid
                    }

                    if ((_printDebug == true) || (_printLimitedDebug == true)) // Print this if doing limited debugging
                    {
                        //Drive an external pin to allow for easier logic analyzation
                        //if (checksumFailurePin >= 0)
                        //{
                        //    digitalWrite((byte)checksumFailurePin, LOW);
                        //    delay(10);
                        //    digitalWrite((byte)checksumFailurePin, HIGH);
                        //}

                        //_debugSerial.Write("Checksum failed:"));
                        //_debugSerial.Write(" checksumA: "));
                        //_debugSerial.Write((incomingUBX.checksumA);
                        //_debugSerial.Write(" checksumB: "));
                        //_debugSerial.Write((incomingUBX.checksumB);

                        //_debugSerial.Write(" rollingChecksumA: "));
                        //_debugSerial.Write((rollingChecksumA);
                        //_debugSerial.Write(" rollingChecksumB: "));
                        //_debugSerial.Write((rollingChecksumB);
                        //_debugSerial.WriteLine();

                        //_debugSerial.Write("Failed  : "));
                        //_debugSerial.Write("Size: "));
                        //_debugSerial.Write((incomingUBX.len);
                        //_debugSerial.Write(" Received: "));
                        printPacket(incomingUBX);
                    }
                }
            }
            else //Load this byte into the payload array
            {
                //If a UBX_NAV_PVT packet comes in asynchronously, we need to fudge the startingSpot
                ushort startingSpot = incomingUBX.startingSpot;
                if (incomingUBX.cls == Constants.UBX_CLASS_NAV && incomingUBX.id == Constants.UBX_NAV_PVT)
                    startingSpot = 0;
                //Begin recording if counter goes past startingSpot
                if ((incomingUBX.counter - 4) >= startingSpot)
                {
                    //Check to see if we have room for this byte
                    if (((incomingUBX.counter - 4) - startingSpot) < Constants.MAX_PAYLOAD_SIZE) //If counter = 208, starting spot = 200, we're good to record.
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

            if (incomingUBX.counter == Constants.MAX_PAYLOAD_SIZE)
            {
                //Something has gone very wrong
                currentSentence = SentenceTypes.NONE; //Reset the sentence to being looking for a new start char
                if (_printDebug == true)
                {
                    _debugSerial.WriteLine("processUBX: counter hit Constants.MAX_PAYLOAD_SIZE");
                }
            }
        }

        //Once a packet has been received and validated, identify this packet's class/id and update internal flags
        //Note: if the user requests a PVT or a HPPOSLLH message using a custom packet, the data extraction will
        //      not work as expected beacuse extractLong etc are hardwired to packetCfg payloadCfg. Ideally
        //      extractLong etc should be updated so they receive a pointer to the packet buffer.
        public void processUBXpacket(ubxPacket msg)
        {
            switch (msg.cls)
            {
                case Constants.UBX_CLASS_NAV:
                    if (msg.id == Constants.UBX_NAV_PVT && msg.len == 92)
                    {
                        //Parse various byte fields into global vars
                        int startingSpot = 0; //fixed value used in processUBX

                        timeOfWeek = extractLong(0);
                        gpsMillisecond = (ushort)(extractLong(0) % 1000); //Get last three digits of iTOW
                        gpsYear = extractInt(4);
                        gpsMonth = extractByte(6);
                        gpsDay = extractByte(7);
                        gpsHour = extractByte(8);
                        gpsMinute = extractByte(9);
                        gpsSecond = extractByte(10);
                        gpsDateValid = (extractByte(11) & 0x01) > 0;
                        gpsTimeValid = ((extractByte(11) & 0x02) >> 1) > 0;
                        gpsNanosecond = (int)extractLong(16); //Includes milliseconds

                        fixType = extractByte((byte)(20 - startingSpot));
                        carrierSolution = (byte)(extractByte((byte)(21 - startingSpot)) >> 6); //Get 6th&7th bits of this byte
                        SIV = extractByte((byte)(23 - startingSpot));
                        longitude = (int)extractLong((byte)(24 - startingSpot));
                        latitude = (int)extractLong((byte)(28 - startingSpot));
                        altitude = (int)extractLong((byte)(32 - startingSpot));
                        altitudeMSL = (int)extractLong((byte)(36 - startingSpot));
                        groundSpeed = (int)extractLong((byte)(60 - startingSpot));
                        headingOfMotion = (int)extractLong((byte)(64 - startingSpot));
                        pDOP = extractInt((byte)(76 - startingSpot));

                        //Mark all datums as fresh (not read before)
                        moduleQueried.gpsiTOW = 1;
                        moduleQueried.gpsYear = 1;
                        moduleQueried.gpsMonth = 1;
                        moduleQueried.gpsDay = 1;
                        moduleQueried.gpsHour = 1;
                        moduleQueried.gpsMinute = 1;
                        moduleQueried.gpsSecond = 1;
                        moduleQueried.gpsDateValid = 1;
                        moduleQueried.gpsTimeValid = 1;
                        moduleQueried.gpsNanosecond = 1;

                        moduleQueried.all = 1;
                        moduleQueried.longitude = 1;
                        moduleQueried.latitude = 1;
                        moduleQueried.altitude = 1;
                        moduleQueried.altitudeMSL = 1;
                        moduleQueried.SIV = 1;
                        moduleQueried.fixType = 1;
                        moduleQueried.carrierSolution = 1;
                        moduleQueried.groundSpeed = 1;
                        moduleQueried.headingOfMotion = 1;
                        moduleQueried.pDOP = 1;
                    }
                    else if (msg.id == Constants.UBX_NAV_HPPOSLLH && msg.len == 36)
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

                        highResModuleQueried.all = 1;
                        highResModuleQueried.highResLatitude = 1;
                        highResModuleQueried.highResLatitudeHp = 1;
                        highResModuleQueried.highResLongitude = 1;
                        highResModuleQueried.highResLongitudeHp = 1;
                        highResModuleQueried.elipsoid = 1;
                        highResModuleQueried.elipsoidHp = 1;
                        highResModuleQueried.meanSeaLevel = 1;
                        highResModuleQueried.meanSeaLevelHp = 1;
                        highResModuleQueried.geoidSeparation = 1;
                        highResModuleQueried.horizontalAccuracy = 1;
                        highResModuleQueried.verticalAccuracy = 1;
                        moduleQueried.gpsiTOW = 1; // this can arrive via HPPOS too.

                        /*
                              if (_printDebug == true)
                              {
                                _debugSerial.Write("Sec: "));
                                _debugSerial.Write((((float)extractLong(4)) / 1000.0f);
                                _debugSerial.Write(" "));
                                _debugSerial.Write("LON: "));
                                _debugSerial.Write((((float)(int)extractLong(8)) / 10000000.0f);
                                _debugSerial.Write(" "));
                                _debugSerial.Write("LAT: "));
                                _debugSerial.Write((((float)(int)extractLong(12)) / 10000000.0f);
                                _debugSerial.Write(" "));
                                _debugSerial.Write("ELI M: "));
                                _debugSerial.Write((((float)(int)extractLong(16)) / 1000.0f);
                                _debugSerial.Write(" "));
                                _debugSerial.Write("MSL M: "));
                                _debugSerial.Write((((float)(int)extractLong(20)) / 1000.0f);
                                _debugSerial.Write(" "));
                                _debugSerial.Write("LON HP: "));
                                _debugSerial.Write((extractSignedChar(24));
                                _debugSerial.Write(" "));
                                _debugSerial.Write("LAT HP: "));
                                _debugSerial.Write((extractSignedChar(25));
                                _debugSerial.Write(" "));
                                _debugSerial.Write("ELI HP: "));
                                _debugSerial.Write((extractSignedChar(26));
                                _debugSerial.Write(" "));
                                _debugSerial.Write("MSL HP: "));
                                _debugSerial.Write((extractSignedChar(27));
                                _debugSerial.Write(" "));
                                _debugSerial.Write("HA 2D M: "));
                                _debugSerial.Write((((float)(int)extractLong(28)) / 10000.0f);
                                _debugSerial.Write(" "));
                                _debugSerial.Write("VERT M: "));
                                _debugSerial.WriteLine(((float)(int)extractLong(32)) / 10000.0f);
                              }
                        */
                    }
                    break;
            }
        }

        //Given a packet and payload, send everything including CRC bytes via I2C port
        public async Task<SfeUbloxStatus> sendCommand(ubxPacket outgoingUBX, ushort maxWait = Constants.DefaultMaxWait)
        {
            SfeUbloxStatus retVal = SfeUbloxStatus.SFE_UBLOX_STATUS_SUCCESS;

            calcChecksum(outgoingUBX); //Sets checksum A and B bytes of the packet

            if (_printDebug == true)
            {
                _debugSerial.Write("\nSending: ");
                printPacket(outgoingUBX);
            }

            if (commType == CommTypes.COMM_TYPE_I2C)
            {
                //retVal = sendI2cCommand(outgoingUBX, maxWait);
                //if (retVal != sfe_ublox_status_e.SFE_UBLOX_STATUS_SUCCESS)
                //{
                //    if (_printDebug == true)
                //    {
                //        _debugSerial.WriteLine("Send I2C Command failed");
                //    }
                //    return retVal;
                //}
            }
            else if (commType == CommTypes.COMM_TYPE_SERIAL)
            {
                sendSerialCommand(outgoingUBX);
            }

            if (maxWait > 0)
            {
                //Depending on what we just sent, either we need to look for an ACK or not
                if (outgoingUBX.cls == Constants.UBX_CLASS_CFG)
                {
                    if (_printDebug == true)
                    {
                        _debugSerial.WriteLine("sendCommand: Waiting for ACK response");
                    }
                    retVal = await waitForACKResponse(outgoingUBX, outgoingUBX.cls, outgoingUBX.id, maxWait); //Wait for Ack response
                }
                else
                {
                    if (_printDebug == true)
                    {
                        _debugSerial.WriteLine("sendCommand: Waiting for No ACK response");
                    }
                    retVal = await waitForNoACKResponse(outgoingUBX, outgoingUBX.cls, outgoingUBX.id, maxWait); //Wait for Ack response
                }
            }
            return retVal;
        }

        //Returns false if sensor fails to respond to I2C traffic
        //sfe_ublox_status_e sendI2cCommand(ubxPacket outgoingUBX, ushort maxWait)
        //{
        //    //Point at 0xFF data register
        //    _i2cPort.beginTransmission((byte)_gpsI2Caddress); //There is no register to write to, we just begin writing data bytes
        //    _i2cPort.write(0xFF);
        //    if (_i2cPort.endTransmission() != 0)         //Don't release bus
        //        return (SFE_UBLOX_STATUS_I2C_COMM_FAILURE); //Sensor did not ACK

        //    //Write header bytes
        //    _i2cPort.beginTransmission((byte)_gpsI2Caddress); //There is no register to write to, we just begin writing data bytes
        //    _i2cPort.write(UBX_SYNCH_1);                         //μ - oh ublox, you're funny. I will call you micro-blox from now on.
        //    _i2cPort.write(UBX_SYNCH_2);                         //b
        //    _i2cPort.write(outgoingUBX.cls);
        //    _i2cPort.write(outgoingUBX.id);
        //    _i2cPort.write(outgoingUBX.len & 0xFF);     //LSB
        //    _i2cPort.write(outgoingUBX.len >> 8);       //MSB
        //    if (_i2cPort.endTransmission(false) != 0)    //Do not release bus
        //        return (SFE_UBLOX_STATUS_I2C_COMM_FAILURE); //Sensor did not ACK

        //    //Write payload. Limit the sends into 32 byte chunks
        //    //This code based on ublox: https://forum.u-blox.com/index.php/20528/how-to-use-i2c-to-get-the-nmea-frames
        //    ushort bytesToSend = outgoingUBX.len;

        //    //"The number of data bytes must be at least 2 to properly distinguish
        //    //from the write access to set the address counter in random read accesses."
        //    ushort startSpot = 0;
        //    while (bytesToSend > 1)
        //    {
        //        byte len = bytesToSend;
        //        if (len > i2cTransactionSize)
        //            len = i2cTransactionSize;

        //        _i2cPort.beginTransmission((byte)_gpsI2Caddress);
        //        //_i2cPort.write(outgoingUBX.payload, len); //Write a portion of the payload to the bus

        //        for (ushort x = 0; x < len; x++)
        //            _i2cPort.write(outgoingUBX.payload[startSpot + x]); //Write a portion of the payload to the bus

        //        if (_i2cPort.endTransmission(false) != 0)    //Don't release bus
        //            return (SFE_UBLOX_STATUS_I2C_COMM_FAILURE); //Sensor did not ACK

        //        //*outgoingUBX.payload += len; //Move the pointer forward
        //        startSpot += len; //Move the pointer forward
        //        bytesToSend -= len;
        //    }

        //    //Write checksum
        //    _i2cPort.beginTransmission((byte)_gpsI2Caddress);
        //    if (bytesToSend == 1)
        //        _i2cPort.write(outgoingUBX.payload, 1);
        //    _i2cPort.write(outgoingUBX.checksumA);
        //    _i2cPort.write(outgoingUBX.checksumB);

        //    //All done transmitting bytes. Release bus.
        //    if (_i2cPort.endTransmission() != 0)
        //        return (SFE_UBLOX_STATUS_I2C_COMM_FAILURE); //Sensor did not ACK
        //    return (SFE_UBLOX_STATUS_SUCCESS);
        //}

        //Given a packet and payload, send everything including CRC bytesA via Serial port
        void sendSerialCommand(ubxPacket outgoingUBX)
        {
            //Write header bytes
            byte[] command = new byte[0];
            addByteToArray(ref command, Constants.UBX_SYNCH_1);
            addByteToArray(ref command, Constants.UBX_SYNCH_2);
            addByteToArray(ref command, outgoingUBX.cls);
            addByteToArray(ref command, outgoingUBX.id);
            addByteToArray(ref command, (byte)(outgoingUBX.len & 0xFF));
            addByteToArray(ref command, (byte)(outgoingUBX.len >> 8));
            command = command.Concat(outgoingUBX.payload.Take(outgoingUBX.len).ToArray()).ToArray();
            addByteToArray(ref command, outgoingUBX.checksumA);
            addByteToArray(ref command, outgoingUBX.checksumB);

            _serialPort.Write(command, 0, command.Length);
        }

        void addByteToArray(ref byte[] bArray, byte newByte)
        {
            byte[] newArray = new byte[bArray.Length + 1];
            bArray.CopyTo(newArray, 0);
            newArray[bArray.Length] = newByte;
            bArray = newArray;
        }
        //public void sendSerialCommand(ubxPacket outgoingUBX)
        //{
        //    //Write header bytes
        //    _serialPort.Write(Constants.UBX_SYNCH_1); //μ - oh ublox, you're funny. I will call you micro-blox from now on.
        //    _serialPort.Write(Constants.UBX_SYNCH_2); //b
        //    _serialPort.Write(outgoingUBX.cls);
        //    _serialPort.Write(outgoingUBX.id);
        //    _serialPort.Write(outgoingUBX.len & 0xFF); //LSB
        //    _serialPort.Write(outgoingUBX.len >> 8);   //MSB

        //    //Write payload.
        //    for (int i = 0; i < outgoingUBX.len; i++)
        //    {
        //        _serialPort.Write(outgoingUBX.payload[i]);
        //    }

        //    //Write checksum
        //    _serialPort.Write(outgoingUBX.checksumA);
        //    _serialPort.Write(outgoingUBX.checksumB);
        //}

        //Returns true if I2C device ack's
        public async Task<bool> isConnected(ushort maxWait = 1100)
        {
            //if (commType == commTypes.COMM_TYPE_I2C)
            //{
            //    _i2cPort.beginTransmission((byte)_gpsI2Caddress);
            //    if (_i2cPort.endTransmission() != 0)
            //        return false; //Sensor did not ack
            //}

            if (!_serialPort.IsOpen) return false;

            // Query navigation rate to see whether we get a meaningful response
            packetCfg.cls = Constants.UBX_CLASS_CFG;
            packetCfg.id = Constants.UBX_CFG_RATE;
            packetCfg.len = 0;
            packetCfg.startingSpot = 0;

            return await sendCommand(packetCfg, maxWait) == SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_RECEIVED; // We are polling the RATE so we expect data and an ACK
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
        //public void calcChecksum(ubxPacket msg)
        //{
        //    msg.checksumA = 0;
        //    msg.checksumB = 0;

        //    msg.checksumA += msg.cls;
        //    msg.checksumB += msg.checksumA;

        //    msg.checksumA += msg.id;
        //    msg.checksumB += msg.checksumA;

        //    msg.checksumA += (msg.len & 0xFF);
        //    msg.checksumB += msg.checksumA;

        //    msg.checksumA += (msg.len >> 8);
        //    msg.checksumB += msg.checksumA;

        //    for (ushort i = 0; i < msg.len; i++)
        //    {
        //        msg.checksumA += msg.payload[i];
        //        msg.checksumB += msg.checksumA;
        //    }
        //}

        //Given a message and a byte, add to rolling "8-Bit Fletcher" checksum
        //This is used when receiving messages from module
        public void addToChecksum(byte incoming)
        {
            rollingChecksumA += incoming;
            rollingChecksumB += rollingChecksumA;
        }

        //Pretty prints the current ubxPacket
        public void printPacket(ubxPacket packet)
        {
            if (_printDebug == true)
            {
                _debugSerial.Write("CLS:");
                if (packet.cls == Constants.UBX_CLASS_NAV) //1
                    _debugSerial.Write("NAV");
                else if (packet.cls == Constants.UBX_CLASS_ACK) //5
                    _debugSerial.Write("ACK");
                else if (packet.cls == Constants.UBX_CLASS_CFG) //6
                    _debugSerial.Write("CFG");
                else if (packet.cls == Constants.UBX_CLASS_MON) //0x0A
                    _debugSerial.Write("MON");
                else
                {
                    _debugSerial.Write("0x");
                    //_debugSerial.Write((packet.cls, HEX);
                }

                _debugSerial.Write(" ID:");
                if (packet.cls == Constants.UBX_CLASS_NAV && packet.id == Constants.UBX_NAV_PVT)
                    _debugSerial.Write("PVT");
                else if (packet.cls == Constants.UBX_CLASS_CFG && packet.id == Constants.UBX_CFG_RATE)
                    _debugSerial.Write("RATE");
                else if (packet.cls == Constants.UBX_CLASS_CFG && packet.id == Constants.UBX_CLASS_CFG)
                    _debugSerial.Write("SAVE");
                else
                {
                    _debugSerial.Write("0x");
                    //_debugSerial.Write(packet.id, HEX);
                }

                _debugSerial.Write(" Len: 0x");
                //_debugSerial.Write((packet.len, HEX);

                // Only print the payload is ignoreThisPayload is false otherwise
                // we could be printing gibberish from beyond the end of packetBuf
                if (ignoreThisPayload == false)
                {
                    _debugSerial.Write(" Payload:");

                    for (int x = 0; x < packet.len; x++)
                    {
                        _debugSerial.Write(" ");
                        //_debugSerial.Write((packet.payload[x], HEX);
                    }
                }
                else
                {
                    _debugSerial.Write(" Payload: IGNORED");
                }
                _debugSerial.WriteLine("");
            }
        }

        //=-=-=-=-=-=-=-= Specific commands =-=-=-=-=-=-=-==-=-=-=-=-=-=-=
        //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-==-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

        //When messages from the class CFG are sent to the receiver, the receiver will send an "acknowledge"(UBX - ACK - ACK) or a
        //"not acknowledge"(UBX-ACK-NAK) message back to the sender, depending on whether or not the message was processed correctly.
        //Some messages from other classes also use the same acknowledgement mechanism.

        //When we poll or get a setting, we will receive _both_ a config packet and an ACK
        //If the poll or get request is not valid, we will receive _only_ a NACK

        //If we are trying to get or poll a setting, then packetCfg.len will be 0 or 1 when the packetCfg is _sent_.
        //If we poll the setting for a particular port using UBX-CFG-PRT then .len will be 1 initially
        //For all other gets or polls, .len will be 0 initially
        //(It would be possible for .len to be 2 _if_ we were using UBX-CFG-MSG to poll the settings for a particular message - but we don't use that (currently))

        //If the get or poll _fails_, i.e. is NACK'd, then packetCfg.len could still be 0 or 1 after the NACK is received
        //But if the get or poll is ACK'd, then packetCfg.len will have been updated by the incoming data and will always be at least 2

        //If we are going to set the value for a setting, then packetCfg.len will be at least 3 when the packetCfg is _sent_.
        //(UBX-CFG-MSG appears to have the shortest set length of 3 bytes)

        //We need to think carefully about how interleaved PVT packets affect things.
        //It is entirely possible that our packetCfg and packetAck were received successfully
        //but while we are still in the "if (checkUblox() == true)" loop a PVT packet is processed
        //or _starts_ to arrive (remember that Serial data can arrive very slowly).

        //Returns SFE_UBLOX_STATUS_DATA_RECEIVED if we got an ACK and a valid packetCfg (module is responding with register content)
        //Returns sfe_ublox_status_e.SFE_UBLOX_STATUS_DATA_SENT if we got an ACK and no packetCfg (no valid packetCfg needed, module absorbs new register data)
        //Returns SFE_UBLOX_STATUS_FAIL if something very bad happens (e.g. a double checksum failure)
        //Returns SFE_UBLOX_STATUS_COMMAND_NACK if the packet was not-acknowledged (NACK)
        //Returns SFE_UBLOX_STATUS_CRC_FAIL if we had a checksum failure
        //Returns SFE_UBLOX_STATUS_TIMEOUT if we timed out
        //Returns SFE_UBLOX_STATUS_DATA_OVERWRITTEN if we got an ACK and a valid packetCfg but that the packetCfg has been
        // or is currently being overwritten (remember that Serial data can arrive very slowly)
        public async Task<SfeUbloxStatus> waitForACKResponse(ubxPacket outgoingUBX, byte requestedClass, byte requestedID, ushort maxTime = Constants.DefaultMaxWait)
        {
            outgoingUBX.valid = SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; //This will go VALID (or NOT_VALID) when we receive a response to the packet we sent
            packetAck.valid = SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
            packetBuf.valid = SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
            outgoingUBX.classAndIDmatch = SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or NOT_VALID) when we receive a packet that matches the requested class and ID
            packetAck.classAndIDmatch = SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
            packetBuf.classAndIDmatch = SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;

            long startTime = Zedf9p.Utils.millis();
            while (Zedf9p.Utils.millis() - startTime < maxTime)
            {
                if (await checkUbloxInternal(outgoingUBX, requestedClass, requestedID) == true) //See if new data is available. Process bytes as they come in.
                {
                    // If both the outgoingUBX.classAndIDmatch and packetAck.classAndIDmatch are VALID
                    // and outgoingUBX.valid is _still_ VALID and the class and ID _still_ match
                    // then we can be confident that the data in outgoingUBX is valid
                    if ((outgoingUBX.classAndIDmatch == SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX.valid == SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX.cls == requestedClass) && (outgoingUBX.id == requestedID))
                    {
                        if (_printDebug == true)
                        {
                            _debugSerial.Write("waitForACKResponse: valid data and valid ACK received after ");
                            //_debugSerial.Write((Zedf9p.Utils.millis() - startTime);
                            //_debugSerial.WriteLine" msec"));
                        }
                        return (SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_RECEIVED); //We received valid data and a correct ACK!
                    }

                    // We can be confident that the data packet (if we are going to get one) will always arrive
                    // before the matching ACK. So if we sent a config packet which only produces an ACK
                    // then outgoingUBX.classAndIDmatch will be NOT_DEFINED and the packetAck.classAndIDmatch will VALID.
                    // We should not check outgoingUBX.valid, outgoingUBX.cls or outgoingUBX.id
                    // as these may have been changed by a PVT packet.
                    else if ((outgoingUBX.classAndIDmatch == SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (packetAck.classAndIDmatch == SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_VALID))
                    {
                        if (_printDebug == true)
                        {
                            _debugSerial.Write("waitForACKResponse: no data and valid ACK after ");
                            //_debugSerial.Write((Zedf9p.Utils.millis() - startTime);
                            _debugSerial.WriteLine(" msec");
                        }
                        return (SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_SENT); //We got an ACK but no data...
                    }

                    // If both the outgoingUBX.classAndIDmatch and packetAck.classAndIDmatch are VALID
                    // but the outgoingUBX.cls or ID no longer match then we can be confident that we had
                    // valid data but it has been or is currently being overwritten by another packet (e.g. PVT).
                    // If (e.g.) a PVT packet is _being_ received: outgoingUBX.valid will be NOT_DEFINED
                    // If (e.g.) a PVT packet _has been_ received: outgoingUBX.valid will be VALID (or just possibly NOT_VALID)
                    // So we cannot use outgoingUBX.valid as part of this check.
                    // Note: the addition of packetBuf should make this check redundant!
                    else if ((outgoingUBX.classAndIDmatch == SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_VALID) && ((outgoingUBX.cls != requestedClass) || (outgoingUBX.id != requestedID)))
                    {
                        if (_printDebug == true)
                        {
                            _debugSerial.Write("waitForACKResponse: data being OVERWRITTEN after ");
                            _debugSerial.Write((Zedf9p.Utils.millis() - startTime).ToString());
                            _debugSerial.WriteLine(" msec");
                        }
                        return SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_OVERWRITTEN; // Data was valid but has been or is being overwritten
                    }

                    // If packetAck.classAndIDmatch is VALID but both outgoingUBX.valid and outgoingUBX.classAndIDmatch
                    // are NOT_VALID then we can be confident we have had a checksum failure on the data packet
                    else if ((packetAck.classAndIDmatch == SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX.classAndIDmatch == SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) && (outgoingUBX.valid == SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_NOT_VALID))
                    {
                        if (_printDebug == true)
                        {
                            _debugSerial.Write("waitForACKResponse: CRC failed after ");
                            //_debugSerial.Write((Zedf9p.Utils.millis() - startTime);
                            //_debugSerial.WriteLine" msec"));
                        }
                        return (SfeUbloxStatus.SFE_UBLOX_STATUS_CRC_FAIL); //Checksum fail
                    }

                    // If our packet was not-acknowledged (NACK) we do not receive a data packet - we only get the NACK.
                    // So you would expect outgoingUBX.valid and outgoingUBX.classAndIDmatch to still be NOT_DEFINED
                    // But if a full PVT packet arrives afterwards outgoingUBX.valid could be VALID (or just possibly NOT_VALID)
                    // but outgoingUBX.cls and outgoingUBX.id would not match...
                    // So I think this is telling us we need a special state for packetAck.classAndIDmatch to tell us
                    // the packet was definitely NACK'd otherwise we are possibly just guessing...
                    // Note: the addition of packetBuf changes the logic of this, but we'll leave the code as is for now.
                    else if (packetAck.classAndIDmatch == SfeUbloxPacketValidity.SFE_UBLOX_PACKET_NOTACKNOWLEDGED)
                    {
                        if (_printDebug == true)
                        {
                            _debugSerial.Write("waitForACKResponse: data was NOTACKNOWLEDGED (NACK) after ");
                            //_debugSerial.Write(Zedf9p.Utils.millis() - startTime);
                            //_debugSerial.WriteLine(" msec"));
                        }
                        return (SfeUbloxStatus.SFE_UBLOX_STATUS_COMMAND_NACK); //We received a NACK!
                    }

                    // If the outgoingUBX.classAndIDmatch is VALID but the packetAck.classAndIDmatch is NOT_VALID
                    // then the ack probably had a checksum error. We will take a gamble and return DATA_RECEIVED.
                    // If we were playing safe, we should return FAIL instead
                    else if ((outgoingUBX.classAndIDmatch == SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) && (outgoingUBX.valid == SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX.cls == requestedClass) && (outgoingUBX.id == requestedID))
                    {
                        if (_printDebug == true)
                        {
                            _debugSerial.Write("waitForACKResponse: VALID data and INVALID ACK received after ");
                            //_debugSerial.Write(Zedf9p.Utils.millis() - startTime);
                            //_debugSerial.WriteLine(" msec"));
                        }
                        return SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_RECEIVED; //We received valid data and an invalid ACK!
                    }

                    // If the outgoingUBX.classAndIDmatch is NOT_VALID and the packetAck.classAndIDmatch is NOT_VALID
                    // then we return a FAIL. This must be a double checksum failure?
                    else if ((outgoingUBX.classAndIDmatch == SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) && (packetAck.classAndIDmatch == SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_NOT_VALID))
                    {
                        if (_printDebug == true)
                        {
                            _debugSerial.Write("waitForACKResponse: INVALID data and INVALID ACK received after ");
                            //_debugSerial.Write(Zedf9p.Utils.millis() - startTime);
                            //_debugSerial.WriteLine(" msec"));
                        }
                        return (SfeUbloxStatus.SFE_UBLOX_STATUS_FAIL); //We received invalid data and an invalid ACK!
                    }

                    // If the outgoingUBX.classAndIDmatch is VALID and the packetAck.classAndIDmatch is NOT_DEFINED
                    // then the ACK has not yet been received and we should keep waiting for it
                    else if ((outgoingUBX.classAndIDmatch == SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED))
                    {
                        if (_printDebug == true)
                        {
                            _debugSerial.Write("waitForACKResponse: valid data after ");
                            //_debugSerial.Write(Zedf9p.Utils.millis() - startTime);
                            //_debugSerial.WriteLine(" msec. Waiting for ACK."));
                        }
                    }

                } //checkUbloxInternal == true

                Thread.Sleep(1);
                //delayMicroseconds(500);
            } //while (Zedf9p.Utils.millis() - startTime < maxTime)

            // We have timed out...
            // If the outgoingUBX.classAndIDmatch is VALID then we can take a gamble and return DATA_RECEIVED
            // even though we did not get an ACK
            if ((outgoingUBX.classAndIDmatch == SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (outgoingUBX.valid == SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX.cls == requestedClass) && (outgoingUBX.id == requestedID))
            {
                if (_printDebug == true)
                {
                    _debugSerial.Write("waitForACKResponse: TIMEOUT with valid data after ");
                    //_debugSerial.Write(Zedf9p.Utils.millis() - startTime);
                    _debugSerial.WriteLine(" msec. ");
                }
                return (SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_RECEIVED); //We received valid data... But no ACK!
            }

            if (_printDebug == true)
            {
                _debugSerial.Write("waitForACKResponse: TIMEOUT after ");
                _debugSerial.Write((Zedf9p.Utils.millis() - startTime).ToString());
                _debugSerial.WriteLine(" msec.");
            }

            return SfeUbloxStatus.SFE_UBLOX_STATUS_TIMEOUT;
        }

        //For non-CFG queries no ACK is sent so we use this function
        //Returns SFE_UBLOX_STATUS_DATA_RECEIVED if we got a config packet full of response data that has CLS/ID match to our query packet
        //Returns SFE_UBLOX_STATUS_CRC_FAIL if we got a corrupt config packet that has CLS/ID match to our query packet
        //Returns SFE_UBLOX_STATUS_TIMEOUT if we timed out
        //Returns SFE_UBLOX_STATUS_DATA_OVERWRITTEN if we got an a valid packetCfg but that the packetCfg has been
        // or is currently being overwritten (remember that Serial data can arrive very slowly)
        public async Task<SfeUbloxStatus> waitForNoACKResponse(ubxPacket outgoingUBX, byte requestedClass, byte requestedID, ushort maxTime = Constants.DefaultMaxWait)
        {
            outgoingUBX.valid = SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; //This will go VALID (or NOT_VALID) when we receive a response to the packet we sent
            packetAck.valid = SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
            packetBuf.valid = SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
            outgoingUBX.classAndIDmatch = SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or NOT_VALID) when we receive a packet that matches the requested class and ID
            packetAck.classAndIDmatch = SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
            packetBuf.classAndIDmatch = SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;

            long startTime = Zedf9p.Utils.millis();
            while (Zedf9p.Utils.millis() - startTime < maxTime)
            {
                if (await checkUbloxInternal(outgoingUBX, requestedClass, requestedID) == true) //See if new data is available. Process bytes as they come in.
                {

                    // If outgoingUBX.classAndIDmatch is VALID
                    // and outgoingUBX.valid is _still_ VALID and the class and ID _still_ match
                    // then we can be confident that the data in outgoingUBX is valid
                    if ((outgoingUBX.classAndIDmatch == SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX.valid == SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX.cls == requestedClass) && (outgoingUBX.id == requestedID))
                    {
                        if (_printDebug == true)
                        {
                            _debugSerial.Write("waitForNoACKResponse: valid data with CLS/ID match after ");
                            //_debugSerial.Write((Zedf9p.Utils.millis() - startTime);
                            //_debugSerial.WriteLine" msec"));
                        }
                        return (SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_RECEIVED); //We received valid data!
                    }

                    // If the outgoingUBX.classAndIDmatch is VALID
                    // but the outgoingUBX.cls or ID no longer match then we can be confident that we had
                    // valid data but it has been or is currently being overwritten by another packet (e.g. PVT).
                    // If (e.g.) a PVT packet is _being_ received: outgoingUBX.valid will be NOT_DEFINED
                    // If (e.g.) a PVT packet _has been_ received: outgoingUBX.valid will be VALID (or just possibly NOT_VALID)
                    // So we cannot use outgoingUBX.valid as part of this check.
                    // Note: the addition of packetBuf should make this check redundant!
                    else if ((outgoingUBX.classAndIDmatch == SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_VALID) && ((outgoingUBX.cls != requestedClass) || (outgoingUBX.id != requestedID)))
                    {
                        if (_printDebug == true)
                        {
                            //_debugSerial.Write("waitForNoACKResponse: data being OVERWRITTEN after "));
                            //_debugSerial.Write((Zedf9p.Utils.millis() - startTime);
                            //_debugSerial.WriteLine" msec"));
                        }
                        return (SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_OVERWRITTEN); // Data was valid but has been or is being overwritten
                    }

                    // If outgoingUBX.classAndIDmatch is NOT_DEFINED
                    // and outgoingUBX.valid is VALID then this must be (e.g.) a PVT packet
                    else if ((outgoingUBX.classAndIDmatch == SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (outgoingUBX.valid == SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_VALID))
                    {
                        // if (_printDebug == true)
                        // {
                        //   _debugSerial.Write("waitForNoACKResponse: valid but UNWANTED data after "));
                        //   _debugSerial.Write((Zedf9p.Utils.millis() - startTime);
                        //   _debugSerial.Write(" msec. Class: "));
                        //   _debugSerial.Write((outgoingUBX.cls);
                        //   _debugSerial.Write(" ID: "));
                        //   _debugSerial.Write((outgoingUBX.id);
                        // }
                    }

                    // If the outgoingUBX.classAndIDmatch is NOT_VALID then we return CRC failure
                    else if (outgoingUBX.classAndIDmatch == SfeUbloxPacketValidity.SFE_UBLOX_PACKET_VALIDITY_NOT_VALID)
                    {
                        if (_printDebug == true)
                        {
                            //_debugSerial.Write("waitForNoACKResponse: CLS/ID match but failed CRC after "));
                            //_debugSerial.Write((Zedf9p.Utils.millis() - startTime);
                            //_debugSerial.WriteLine" msec"));
                        }
                        return (SfeUbloxStatus.SFE_UBLOX_STATUS_CRC_FAIL); //We received invalid data
                    }
                }

                Thread.Sleep(1);
                //delayMicroseconds(500);
            }

            if (_printDebug == true)
            {
                //_debugSerial.Write("waitForNoACKResponse: TIMEOUT after "));
                //_debugSerial.Write((Zedf9p.Utils.millis() - startTime);
                //_debugSerial.WriteLine" msec. No packet received."));
            }

            return (SfeUbloxStatus.SFE_UBLOX_STATUS_TIMEOUT);
        }

        //Save current configuration to flash and BBR (battery backed RAM)
        //This still works but it is the old way of configuring ublox modules. See getVal and setVal for the new methods
        public async Task<bool> saveConfiguration(ushort maxWait = Constants.DefaultMaxWait)
        {
            packetCfg.cls = Constants.UBX_CLASS_CFG;
            packetCfg.id = Constants.UBX_CLASS_CFG;
            packetCfg.len = 12;
            packetCfg.startingSpot = 0;

            //Clear packet payload
            for (byte x = 0; x < packetCfg.len; x++)
                packetCfg.payload[x] = 0;

            packetCfg.payload[4] = 0xFF; //Set any bit in the saveMask field to save current config to Flash and BBR
            packetCfg.payload[5] = 0xFF;

            return await sendCommand(packetCfg, maxWait) == SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_SENT; // We are only expecting an ACK
        }

        //Save the selected configuration sub-sections to flash and BBR (battery backed RAM)
        //This still works but it is the old way of configuring ublox modules. See getVal and setVal for the new methods
        public async Task<bool> saveConfigSelective(uint configMask, ushort maxWait = Constants.DefaultMaxWait)
        {
            packetCfg.cls = Constants.UBX_CLASS_CFG;
            packetCfg.id = Constants.UBX_CLASS_CFG;
            packetCfg.len = 12;
            packetCfg.startingSpot = 0;

            //Clear packet payload
            for (byte x = 0; x < packetCfg.len; x++)
                packetCfg.payload[x] = 0;

            packetCfg.payload[4] = (byte)(configMask & 0xFF); //Set the appropriate bits in the saveMask field to save current config to Flash and BBR
            packetCfg.payload[5] = (byte)((configMask >> 8) & 0xFF);
            packetCfg.payload[6] = (byte)((configMask >> 16) & 0xFF);
            packetCfg.payload[7] = (byte)((configMask >> 24) & 0xFF);

            return await sendCommand(packetCfg, maxWait) == SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_SENT; // We are only expecting an ACK
        }

        //Reset module to factory defaults
        //This still works but it is the old way of configuring ublox modules. See getVal and setVal for the new methods
        public async Task<bool> factoryDefault(ushort maxWait = Constants.DefaultMaxWait)
        {
            packetCfg.cls = Constants.UBX_CLASS_CFG;
            packetCfg.id = Constants.UBX_CLASS_CFG;
            packetCfg.len = 12;
            packetCfg.startingSpot = 0;

            //Clear packet payload
            for (byte x = 0; x < packetCfg.len; x++)
                packetCfg.payload[x] = 0;

            packetCfg.payload[0] = 0xFF; //Set any bit in the clearMask field to clear saved config
            packetCfg.payload[1] = 0xFF;
            packetCfg.payload[8] = 0xFF; //Set any bit in the loadMask field to discard current config and rebuild from lower non-volatile memory layers
            packetCfg.payload[9] = 0xFF;

            return await sendCommand(packetCfg, maxWait) == SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_SENT; // We are only expecting an ACK
        }

        //Given a key, load the payload with data that can then be extracted to 8, 16, or 32 bits
        //This function takes a full 32-bit key
        //Default layer is RAM
        //Configuration of modern u-blox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
        public async Task<SfeUbloxStatus> getVal(uint key, byte layer, ushort maxWait = 250)
        {
            packetCfg.cls = Constants.UBX_CLASS_CFG;
            packetCfg.id = Constants.UBX_CFG_VALGET;
            packetCfg.len = 4 + 4 * 1; //While multiple keys are allowed, we will send only one key at a time
            packetCfg.startingSpot = 0;

            //Clear packet payload
            for (byte x = 0; x < packetCfg.len; x++)
                packetCfg.payload[x] = 0;

            //VALGET uses different memory layer definitions to VALSET
            //because it can only return the value for one layer.
            //So we need to fiddle the layer here.
            //And just to complicate things further, the ZED-F9P only responds
            //correctly to layer 0 (RAM) and layer 7 (Default)!
            byte getLayer = 7;                         // 7 is the "Default Layer"
            if ((layer & Constants.VAL_LAYER_RAM) == Constants.VAL_LAYER_RAM) // Did the user request the RAM layer?
            {
                getLayer = 0; // Layer 0 is RAM
            }

            payloadCfg[0] = 0;        //Message Version - set to 0
            payloadCfg[1] = getLayer; //Layer

            //Load key into outgoing payload
            payloadCfg[4] = (byte)(key >> 8 * 0); //Key LSB
            payloadCfg[5] = (byte)(key >> 8 * 1);
            payloadCfg[6] = (byte)(key >> 8 * 2);
            payloadCfg[7] = (byte)(key >> 8 * 3);

            if (_printDebug == true)
            {
                //_debugSerial.Write("key: 0x"));
                //_debugSerial.Write((key, HEX);
                //_debugSerial.WriteLine();
            }

            //Send VALGET command with this key

            SfeUbloxStatus retVal = await sendCommand(packetCfg, maxWait);
            if (_printDebug == true)
            {
                //_debugSerial.Write("getVal: sendCommand returned: "));
                _debugSerial.WriteLine(statusString(retVal));
            }

            //Verify the response is the correct length as compared to what the user called (did the module respond with 8-bits but the user called getVal32?)
            //Response is 8 bytes plus cfg data
            //if(packet.len > 8+1)

            //The response is now sitting in payload, ready for extraction
            return (retVal);
        }

        //Given a key, return its value
        //This function takes a full 32-bit key
        //Default layer is RAM
        //Configuration of modern Ublox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
        public async Task<byte> getVal8(uint key, byte layer, ushort maxWait = 250)
        {
            if (await getVal(key, layer, maxWait) != SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_RECEIVED)
                return 0;

            return extractByte(8);
        }
        public async Task<ushort> getVal16(uint key, byte layer, ushort maxWait = 250)
        {
            if (await getVal(key, layer, maxWait) != SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_RECEIVED)
                return 0;

            return extractInt(8);
        }
        public async Task<uint> getVal32(uint key, byte layer, ushort maxWait = 250)
        {
            if (await getVal(key, layer, maxWait) != SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_RECEIVED)
                return 0;

            return extractLong(8);
        }

        //Form 32-bit key from group/id/size
        public uint createKey(ushort group, ushort id, byte size)
        {
            uint key = 0;
            key |= (uint)id;
            key |= (uint)group << 16;
            key |= (uint)size << 28;
            return key;
        }

        //Given a group, ID and size, return the value of this config spot
        //The 32-bit key is put together from group/ID/size. See other getVal to send key directly.
        //Configuration of modern Ublox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
        public async Task<byte> getVal8(ushort group, ushort id, byte size, byte layer, ushort maxWait = 250)
        {
            uint key = createKey(group, id, size);
            return await getVal8(key, layer, maxWait);
        }
        public async Task<ushort> getVal16(ushort group, ushort id, byte size, byte layer, ushort maxWait = 250)
        {
            uint key = createKey(group, id, size);
            return await getVal16(key, layer, maxWait);
        }
        public async Task<uint> getVal32(ushort group, ushort id, byte size, byte layer, ushort maxWait = 250)
        {
            uint key = createKey(group, id, size);
            return await getVal32(key, layer, maxWait);
        }

        //Given a key, set a 16-bit value
        //This function takes a full 32-bit key
        //Default layer is all: RAM+BBR+Flash
        //Configuration of modern Ublox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
        public async Task<byte> setVal(uint key, ushort value, byte layer, ushort maxWait = 250)
        {
            return await setVal16(key, value, layer, maxWait) ? (byte)1 : (byte)0;
        }

        //Given a key, set a 16-bit value
        //This function takes a full 32-bit key
        //Default layer is all: RAM+BBR+Flash
        //Configuration of modern Ublox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
        public async Task<bool> setVal16(uint key, ushort value, byte layer, ushort maxWait = 250)
        {
            packetCfg.cls = Constants.UBX_CLASS_CFG;
            packetCfg.id = Constants.UBX_CFG_VALSET;
            packetCfg.len = 4 + 4 + 2; //4 byte header, 4 byte key ID, 2 bytes of value
            packetCfg.startingSpot = 0;

            //Clear packet payload
            for (ushort x = 0; x < packetCfg.len; x++)
                packetCfg.payload[x] = 0;

            payloadCfg[0] = 0;     //Message Version - set to 0
            payloadCfg[1] = layer; //By default we ask for the BBR layer

            //Load key into outgoing payload
            payloadCfg[4] = (byte)(key >> 8 * 0); //Key LSB
            payloadCfg[5] = (byte)(key >> 8 * 1);
            payloadCfg[6] = (byte)(key >> 8 * 2);
            payloadCfg[7] = (byte)(key >> 8 * 3);

            //Load user's value
            payloadCfg[8] = (byte)(value >> 8 * 0); //Value LSB
            payloadCfg[9] = (byte)(value >> 8 * 1);

            //Send VALSET command with this key and value
            return await sendCommand(packetCfg, maxWait) == SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_SENT; // We are only expecting an ACK
        }

        //Given a key, set an 8-bit value
        //This function takes a full 32-bit key
        //Default layer is all: RAM+BBR+Flash
        //Configuration of modern Ublox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
        public async Task<bool> setVal8(uint key, byte value, byte layer, ushort maxWait = 250)
        {
            packetCfg.cls = Constants.UBX_CLASS_CFG;
            packetCfg.id = Constants.UBX_CFG_VALSET;
            packetCfg.len = 4 + 4 + 1; //4 byte header, 4 byte key ID, 1 byte value
            packetCfg.startingSpot = 0;

            //Clear packet payload
            for (ushort x = 0; x < packetCfg.len; x++)
                packetCfg.payload[x] = 0;

            payloadCfg[0] = 0;     //Message Version - set to 0
            payloadCfg[1] = layer; //By default we ask for the BBR layer

            //Load key into outgoing payload
            payloadCfg[4] = (byte)(key >> 8 * 0); //Key LSB
            payloadCfg[5] = (byte)(key >> 8 * 1);
            payloadCfg[6] = (byte)(key >> 8 * 2);
            payloadCfg[7] = (byte)(key >> 8 * 3);

            //Load user's value
            payloadCfg[8] = value; //Value

            //Send VALSET command with this key and value
            return await sendCommand(packetCfg, maxWait) == SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_SENT; // We are only expecting an ACK
        }

        //Given a key, set a 32-bit value
        //This function takes a full 32-bit key
        //Default layer is all: RAM+BBR+Flash
        //Configuration of modern Ublox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
        public async Task<bool> setVal32(uint key, uint value, byte layer, ushort maxWait = 250)
        {
            packetCfg.cls = Constants.UBX_CLASS_CFG;
            packetCfg.id = Constants.UBX_CFG_VALSET;
            packetCfg.len = 4 + 4 + 4; //4 byte header, 4 byte key ID, 4 bytes of value
            packetCfg.startingSpot = 0;

            //Clear packet payload
            for (ushort x = 0; x < packetCfg.len; x++)
                packetCfg.payload[x] = 0;

            payloadCfg[0] = 0;     //Message Version - set to 0
            payloadCfg[1] = layer; //By default we ask for the BBR layer

            //Load key into outgoing payload
            payloadCfg[4] = (byte)(key >> 8 * 0); //Key LSB
            payloadCfg[5] = (byte)(key >> 8 * 1);
            payloadCfg[6] = (byte)(key >> 8 * 2);
            payloadCfg[7] = (byte)(key >> 8 * 3);

            //Load user's value
            payloadCfg[8] = (byte)(value >> 8 * 0); //Value LSB
            payloadCfg[9] = (byte)(value >> 8 * 1);
            payloadCfg[10] = (byte)(value >> 8 * 2);
            payloadCfg[11] = (byte)(value >> 8 * 3);

            //Send VALSET command with this key and value
            return await sendCommand(packetCfg, maxWait) == SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_SENT; // We are only expecting an ACK
        }

        //Start defining a new UBX-CFG-VALSET ubxPacket
        //This function takes a full 32-bit key and 32-bit value
        //Default layer is BBR
        //Configuration of modern u-blox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
        public bool newCfgValset32(uint key, uint value, byte layer)
        {
            packetCfg.cls = Constants.UBX_CLASS_CFG;
            packetCfg.id = Constants.UBX_CFG_VALSET;
            packetCfg.len = 4 + 4 + 4; //4 byte header, 4 byte key ID, 4 bytes of value
            packetCfg.startingSpot = 0;

            //Clear packet payload
            for (ushort x = 0; x < Constants.MAX_PAYLOAD_SIZE; x++)
                packetCfg.payload[x] = 0;

            payloadCfg[0] = 0;     //Message Version - set to 0
            payloadCfg[1] = layer; //By default we ask for the BBR layer

            //Load key into outgoing payload
            payloadCfg[4] = (byte)(key >> 8 * 0); //Key LSB
            payloadCfg[5] = (byte)(key >> 8 * 1);
            payloadCfg[6] = (byte)(key >> 8 * 2);
            payloadCfg[7] = (byte)(key >> 8 * 3);

            //Load user's value
            payloadCfg[8] = (byte)(value >> 8 * 0); //Value LSB
            payloadCfg[9] = (byte)(value >> 8 * 1);
            payloadCfg[10] = (byte)(value >> 8 * 2);
            payloadCfg[11] = (byte)(value >> 8 * 3);

            //All done
            return true;
        }

        //Start defining a new UBX-CFG-VALSET ubxPacket
        //This function takes a full 32-bit key and 16-bit value
        //Default layer is BBR
        //Configuration of modern u-blox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
        public async Task<bool> newCfgValset16(uint key, ushort value, byte layer)
        {
            packetCfg.cls = Constants.UBX_CLASS_CFG;
            packetCfg.id = Constants.UBX_CFG_VALSET;
            packetCfg.len = 4 + 4 + 2; //4 byte header, 4 byte key ID, 2 bytes of value
            packetCfg.startingSpot = 0;

            //Clear packet payload
            for (ushort x = 0; x < Constants.MAX_PAYLOAD_SIZE; x++)
                packetCfg.payload[x] = 0;

            payloadCfg[0] = 0;     //Message Version - set to 0
            payloadCfg[1] = layer; //By default we ask for the BBR layer

            //Load key into outgoing payload
            payloadCfg[4] = (byte)(key >> 8 * 0); //Key LSB
            payloadCfg[5] = (byte)(key >> 8 * 1);
            payloadCfg[6] = (byte)(key >> 8 * 2);
            payloadCfg[7] = (byte)(key >> 8 * 3);

            //Load user's value
            payloadCfg[8] = (byte)(value >> 8 * 0); //Value LSB
            payloadCfg[9] = (byte)(value >> 8 * 1);

            //All done
            return true;
        }

        //Start defining a new UBX-CFG-VALSET ubxPacket
        //This function takes a full 32-bit key and 8-bit value
        //Default layer is BBR
        //Configuration of modern u-blox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
        public async Task<bool> newCfgValset8(uint key, byte value, byte layer)
        {
            packetCfg.cls = Constants.UBX_CLASS_CFG;
            packetCfg.id = Constants.UBX_CFG_VALSET;
            packetCfg.len = 4 + 4 + 1; //4 byte header, 4 byte key ID, 1 byte value
            packetCfg.startingSpot = 0;

            //Clear packet payload
            for (ushort x = 0; x < Constants.MAX_PAYLOAD_SIZE; x++)
                packetCfg.payload[x] = 0;

            payloadCfg[0] = 0;     //Message Version - set to 0
            payloadCfg[1] = layer; //By default we ask for the BBR layer

            //Load key into outgoing payload
            payloadCfg[4] = (byte)(key >> 8 * 0); //Key LSB
            payloadCfg[5] = (byte)(key >> 8 * 1);
            payloadCfg[6] = (byte)(key >> 8 * 2);
            payloadCfg[7] = (byte)(key >> 8 * 3);

            //Load user's value
            payloadCfg[8] = value; //Value

            //All done
            return true;
        }

        //Add another keyID and value to an existing UBX-CFG-VALSET ubxPacket
        //This function takes a full 32-bit key and 32-bit value
        public async Task<bool> addCfgValset32(uint key, uint value)
        {
            //Load key into outgoing payload
            payloadCfg[packetCfg.len + 0] = (byte)(key >> 8 * 0); //Key LSB
            payloadCfg[packetCfg.len + 1] = (byte)(key >> 8 * 1);
            payloadCfg[packetCfg.len + 2] = (byte)(key >> 8 * 2);
            payloadCfg[packetCfg.len + 3] = (byte)(key >> 8 * 3);

            //Load user's value
            payloadCfg[packetCfg.len + 4] = (byte)(value >> 8 * 0); //Value LSB
            payloadCfg[packetCfg.len + 5] = (byte)(value >> 8 * 1);
            payloadCfg[packetCfg.len + 6] = (byte)(value >> 8 * 2);
            payloadCfg[packetCfg.len + 7] = (byte)(value >> 8 * 3);

            //Update packet length: 4 byte key ID, 4 bytes of value
            packetCfg.len = (ushort)(packetCfg.len + 4 + 4);

            //All done
            return true;
        }

        //Add another keyID and value to an existing UBX-CFG-VALSET ubxPacket
        //This function takes a full 32-bit key and 16-bit value
        public bool addCfgValset16(uint key, ushort value)
        {
            //Load key into outgoing payload
            payloadCfg[packetCfg.len + 0] = (byte)(key >> 8 * 0); //Key LSB
            payloadCfg[packetCfg.len + 1] = (byte)(key >> 8 * 1);
            payloadCfg[packetCfg.len + 2] = (byte)(key >> 8 * 2);
            payloadCfg[packetCfg.len + 3] = (byte)(key >> 8 * 3);

            //Load user's value
            payloadCfg[packetCfg.len + 4] = (byte)(value >> 8 * 0); //Value LSB
            payloadCfg[packetCfg.len + 5] = (byte)(value >> 8 * 1);

            //Update packet length: 4 byte key ID, 2 bytes of value
            packetCfg.len = (ushort)(packetCfg.len + 4 + 2);

            //All done
            return true;
        }

        //Add another keyID and value to an existing UBX-CFG-VALSET ubxPacket
        //This function takes a full 32-bit key and 8-bit value
        public bool addCfgValset8(uint key, byte value)
        {
            //Load key into outgoing payload
            payloadCfg[packetCfg.len + 0] = (byte)(key >> 8 * 0); //Key LSB
            payloadCfg[packetCfg.len + 1] = (byte)(key >> 8 * 1);
            payloadCfg[packetCfg.len + 2] = (byte)(key >> 8 * 2);
            payloadCfg[packetCfg.len + 3] = (byte)(key >> 8 * 3);

            //Load user's value
            payloadCfg[packetCfg.len + 4] = value; //Value

            //Update packet length: 4 byte key ID, 1 byte value
            packetCfg.len = (ushort)(packetCfg.len + 4 + 1);

            //All done
            return true;
        }

        //Add a final keyID and value to an existing UBX-CFG-VALSET ubxPacket and send it
        //This function takes a full 32-bit key and 32-bit value
        public async Task<bool> sendCfgValset32(uint key, uint value, ushort maxWait = 250)
        {
            //Load keyID and value into outgoing payload
            await addCfgValset32(key, value);

            //Send VALSET command with this key and value
            return await sendCommand(packetCfg, maxWait) == SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_SENT; // We are only expecting an ACK
        }

        //Add a final keyID and value to an existing UBX-CFG-VALSET ubxPacket and send it
        //This function takes a full 32-bit key and 16-bit value
        public async Task<bool> sendCfgValset16(uint key, ushort value, ushort maxWait = 250)
        {
            //Load keyID and value into outgoing payload
            addCfgValset16(key, value);

            //Send VALSET command with this key and value
            return await sendCommand(packetCfg, maxWait) == SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_SENT; // We are only expecting an ACK
        }

        //Add a final keyID and value to an existing UBX-CFG-VALSET ubxPacket and send it
        //This function takes a full 32-bit key and 8-bit value
        public async Task<bool> sendCfgValset8(uint key, byte value, ushort maxWait = 250)
        {
            //Load keyID and value into outgoing payload
            addCfgValset8(key, value);

            //Send VALSET command with this key and value
            return await sendCommand(packetCfg, maxWait) == SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_SENT; // We are only expecting an ACK
        }

        //Get the current TimeMode3 settings - these contain survey in statuses
        public async Task<SurveyMode> getSurveyMode(ushort maxWait = 250)
        {
            packetCfg.cls = Constants.UBX_CLASS_CFG;
            packetCfg.id = Constants.UBX_CFG_TMODE3;
            packetCfg.len = 0;
            packetCfg.startingSpot = 0;

            return await sendCommand(packetCfg, maxWait) == SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_RECEIVED // We are expecting data and an ACK
            ? new SurveyMode() { payload = payloadCfg.Take(40).ToArray() }//length of date in this packet is 40 bytes
            : null;
        }

        //Control Survey-In for NEO-M8P
        public async Task<bool> setSurveyMode(byte mode, ushort observationTime, float requiredAccuracy, ushort maxWait = 250)
        {
            if (await getSurveyMode(maxWait) == null) //Ask module for the current TimeMode3 settings. Loads into payloadCfg.
                return false;

            packetCfg.cls = Constants.UBX_CLASS_CFG;
            packetCfg.id = Constants.UBX_CFG_TMODE3;
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
            payloadCfg[28] = (byte)(svinAccLimit & 0xFF);                          //svinAccLimit in 0.1mm increments
            payloadCfg[29] = (byte)(svinAccLimit >> 8);
            payloadCfg[30] = (byte)(svinAccLimit >> 16);
            payloadCfg[31] = (byte)(svinAccLimit >> 24);
            //BitConverter.GetBytes(svinAccLimit).CopyTo(payloadCfg, 28);

            return await sendCommand(packetCfg, maxWait) == SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_SENT; // We are only expecting an ACK
        }
        
        //Control Fixed Mode for ZED-F9p
        public async Task<bool> setFixedMode(int latitude, int longitude, int altitude, ushort maxWait = 250)
        {
            if (await getSurveyMode(maxWait) == null) //Ask module for the current TimeMode3 settings. Loads into payloadCfg.
                return false;

            packetCfg.cls = Constants.UBX_CLASS_CFG;
            packetCfg.id = Constants.UBX_CFG_TMODE3;
            packetCfg.len = 40;
            packetCfg.startingSpot = 0;

            //Clear packet payload
            for (byte x = 0; x < packetCfg.len; x++) packetCfg.payload[x] = 0;

            //payloadCfg should be loaded with poll response. Now modify only the bits we care about
            payloadCfg[2] = Constants.SVIN_MODE_FIXED; //Set mode. Survey-In and Disabled are most common. Use ECEF (not LAT/LON/ALT).

            //set latitude
            BitConverter.GetBytes(latitude).CopyTo(payloadCfg, 8);
            payloadCfg[4] = (byte)(latitude & 0xFF);
            payloadCfg[5] = (byte)(latitude >> 8);
            payloadCfg[6] = (byte)(latitude >> 16);
            payloadCfg[7] = (byte)(latitude >> 24);

            //set longitude
            //BitConverter.GetBytes(longitude).CopyTo(payloadCfg, 8);
            //payloadCfg[4] = (byte)(latitude & 0xFF);
            //payloadCfg[5] = (byte)(latitude >> 8);
            //payloadCfg[6] = (byte)(latitude >> 16);
            //payloadCfg[7] = (byte)(latitude >> 24);

            //set altitude
            BitConverter.GetBytes(altitude).CopyTo(payloadCfg, 12);

            //svinMinDur is U4 (uint) but we'll only use a ushort (waiting more than 65535 seconds seems excessive!)
            //payloadCfg[24] = (byte)(observationTime & 0xFF); //svinMinDur in seconds
            //payloadCfg[25] = (byte)(observationTime >> 8);   //svinMinDur in seconds
            //payloadCfg[26] = 0;                      //Truncate to 16 bits
            //payloadCfg[27] = 0;                      //Truncate to 16 bits

            //svinAccLimit is U4 (uint) in 0.1mm.
            //uint svinAccLimit = (uint)(requiredAccuracy * 10000.0); //Convert m to 0.1mm
            //payloadCfg[28] = (byte)(svinAccLimit & 0xFF);                          //svinAccLimit in 0.1mm increments
            //payloadCfg[29] = (byte)(svinAccLimit >> 8);
            //payloadCfg[30] = (byte)(svinAccLimit >> 16);
            //payloadCfg[31] = (byte)(svinAccLimit >> 24);
            //BitConverter.GetBytes(svinAccLimit).CopyTo(payloadCfg, 28);

            return await sendCommand(packetCfg, maxWait) == SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_SENT; // We are only expecting an ACK
        }

        //Begin Survey-In for NEO-M8P
        public async Task<bool> enableSurveyMode(ushort observationTime, float requiredAccuracy, ushort maxWait = 250)
        {
            return await setSurveyMode(Constants.SVIN_MODE_ENABLE, observationTime, requiredAccuracy, maxWait);
        }

        //Stop Survey-In for NEO-M8P
        public async Task<bool> disableSurveyMode(ushort maxWait = 250)
        {
            return await setSurveyMode(Constants.SVIN_MODE_DISABLE, 0, 0, maxWait);
        }

        //Reads survey in status and sets the global variables
        //for status, position valid, observation time, and mean 3D StdDev
        //Returns true if commands was successful
        public async Task<bool> getSurveyStatus(ushort maxWait)
        {
            //Reset variables
            svin.active = false;
            svin.valid = false;
            svin.observationTime = 0;
            svin.meanAccuracy = 0;

            packetCfg.cls = Constants.UBX_CLASS_NAV;
            packetCfg.id = Constants.UBX_NAV_SVIN;
            packetCfg.len = 0;
            packetCfg.startingSpot = 0;

            if(await sendCommand(packetCfg, maxWait) != SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
                return false;                                                         //If command send fails then bail

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
            svin.meanAccuracy = ((float)tempFloat) / 10000.0f; //Convert 0.1mm to m

            svin.valid = payloadCfg[36] > 0;  //1 if survey-in position is valid, 0 otherwise
            svin.active = payloadCfg[37] > 0; //1 if survey-in in progress, 0 otherwise

            return true;
        }

        //Loads the payloadCfg array with the current protocol bits located the UBX-CFG-PRT register for a given port
        public async Task<bool> getPortSettings(byte portID, ushort maxWait = Constants.DefaultMaxWait)
        {
            packetCfg.cls = Constants.UBX_CLASS_CFG;
            packetCfg.id = Constants.UBX_CFG_PRT;
            packetCfg.len = 1;
            packetCfg.startingSpot = 0;

            payloadCfg[0] = portID;

            return await sendCommand(packetCfg, maxWait) == SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_RECEIVED; // We are expecting data and an ACK
        }

        //Configure a given port to output UBX, NMEA, RTCM3 or a combination thereof
        //Port 0=I2c, 1=UART1, 2=UART2, 3=USB, 4=SPI
        //Bit:0 = UBX, :1=NMEA, :5=RTCM3
        public async Task<bool> setPortOutput(byte portID, byte outStreamSettings, ushort maxWait = Constants.DefaultMaxWait)
        {
            //Get the current config values for this port ID
            if (await getPortSettings(portID, maxWait) == false)
                return false; //Something went wrong. Bail.

            packetCfg.cls = Constants.UBX_CLASS_CFG;
            packetCfg.id = Constants.UBX_CFG_PRT;
            packetCfg.len = 20;
            packetCfg.startingSpot = 0;

            //payloadCfg is now loaded with current bytes. Change only the ones we need to
            payloadCfg[14] = outStreamSettings; //OutProtocolMask LSB - Set outStream bits

            return await sendCommand(packetCfg, maxWait) == SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_SENT; // We are only expecting an ACK
        }

        //Configure a given port to input UBX, NMEA, RTCM3 or a combination thereof
        //Port 0=I2c, 1=UART1, 2=UART2, 3=USB, 4=SPI
        //Bit:0 = UBX, :1=NMEA, :5=RTCM3
        public async Task<bool> setPortInput(byte portID, byte inStreamSettings, ushort maxWait = Constants.DefaultMaxWait)
        {
            //Get the current config values for this port ID
            //This will load the payloadCfg array with current port settings
            if (await getPortSettings(portID, maxWait) == false)
                return false; //Something went wrong. Bail.

            packetCfg.cls = Constants.UBX_CLASS_CFG;
            packetCfg.id = Constants.UBX_CFG_PRT;
            packetCfg.len = 20;
            packetCfg.startingSpot = 0;

            //payloadCfg is now loaded with current bytes. Change only the ones we need to
            payloadCfg[12] = inStreamSettings; //InProtocolMask LSB - Set inStream bits

            return await sendCommand(packetCfg, maxWait) == SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_SENT; // We are only expecting an ACK
        }

        //Configure a port to output UBX, NMEA, RTCM3 or a combination thereof
        public async Task<bool> setI2COutput(byte comSettings, ushort maxWait = 250)
        {
            return await setPortOutput(Constants.COM_PORT_I2C, comSettings, maxWait);
        }
        public async Task<bool> setUART1Output(byte comSettings, ushort maxWait = Constants.DefaultMaxWait)
        {
            return await setPortOutput(Constants.COM_PORT_UART1, comSettings, maxWait);
        }
        public async Task<bool> setUART2Output(byte comSettings, ushort maxWait = Constants.DefaultMaxWait)
        {
            return await setPortOutput(Constants.COM_PORT_UART2, comSettings, maxWait);
        }
        public async Task<bool> setUSBOutput(byte comSettings, ushort maxWait = 250)
        {
            return await setPortOutput(Constants.COM_PORT_USB, comSettings, maxWait);
        }
        public async Task<bool> setSPIOutput(byte comSettings, ushort maxWait = 250)
        {
            return await setPortOutput(Constants.COM_PORT_SPI, comSettings, maxWait);
        }

        //Set the rate at which the module will give us an updated navigation solution
        //Expects a number that is the updates per second. For example 1 = 1Hz, 2 = 2Hz, etc.
        //Max is 40Hz(?!)
        public async Task<bool> setNavigationFrequency(byte navFreq, ushort maxWait = Constants.DefaultMaxWait)
        {
            //if(updateRate > 40) updateRate = 40; //Not needed: module will correct out of bounds values

            //Adjust the I2C polling timeout based on update rate
            i2cPollingWait = (byte)(1000 / (navFreq * 4)); //This is the number of ms to wait between checks for new I2C data

            //Query the module for the latest lat/long
            packetCfg.cls = Constants.UBX_CLASS_CFG;
            packetCfg.id = Constants.UBX_CFG_RATE;
            packetCfg.len = 0;
            packetCfg.startingSpot = 0;

            //This will load the payloadCfg array with current settings of the given register
            if(await sendCommand(packetCfg, maxWait) != SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
                return false;                                                       //If command send fails then bail

            ushort measurementRate = (ushort)(1000 / navFreq);

            //payloadCfg is now loaded with current bytes. Change only the ones we need to
            payloadCfg[0] = (byte)(measurementRate & 0xFF); //measRate LSB
            payloadCfg[1] = (byte)(measurementRate >> 8);   //measRate MSB

            return await sendCommand(packetCfg, maxWait) == SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_SENT; // We are only expecting an ACK
        }

        //Get the rate at which the module is outputting nav solutions
        public async Task<byte> getNavigationFrequency(ushort maxWait = Constants.DefaultMaxWait)
        {
            //Query the module for the latest lat/long
            packetCfg.cls = Constants.UBX_CLASS_CFG;
            packetCfg.id = Constants.UBX_CFG_RATE;
            packetCfg.len = 0;
            packetCfg.startingSpot = 0;

            //This will load the payloadCfg array with current settings of the given register
            if (await sendCommand(packetCfg, maxWait) != SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
                return 0;                                                       //If command send fails then bail

            ushort measurementRate = 0;

            //payloadCfg is now loaded with current bytes. Get what we need
            measurementRate = extractInt(0); //Pull from payloadCfg at measRate LSB

            measurementRate = (ushort)(1000 / measurementRate); //This may return an int when it's a float, but I'd rather not return 4 bytes
            return (byte)measurementRate;
        }

        //In case no config access to the GPS is possible and PVT is send cyclically already
        //set config to suitable parameters
        public bool assumeAutoPVT(bool enabled, bool implicitUpdate)
        {
            bool changes = autoPVT != enabled || autoPVTImplicitUpdate != implicitUpdate;
            if (changes)
            {
                autoPVT = enabled;
                autoPVTImplicitUpdate = implicitUpdate;
            }
            return changes;
        }

        //Enable or disable automatic navigation message generation by the GPS. This changes the way getPVT
        //works.
        public async Task<bool> setAutoPVT(bool enable, ushort maxWait = Constants.DefaultMaxWait)
        {
            return await setAutoPVT(enable, true, maxWait);
        }

        //Enable or disable automatic navigation message generation by the GPS. This changes the way getPVT
        //works.
        public async Task<bool> setAutoPVT(bool enable, bool implicitUpdate, ushort maxWait = Constants.DefaultMaxWait)
        {
            packetCfg.cls = Constants.UBX_CLASS_CFG;
            packetCfg.id = Constants.UBX_CFG_MSG;
            packetCfg.len = 3;
            packetCfg.startingSpot = 0;
            payloadCfg[0] = Constants.UBX_CLASS_NAV;
            payloadCfg[1] = Constants.UBX_NAV_PVT;
            payloadCfg[2] = enable ? (byte)1 : (byte)0; // rate relative to navigation freq.

            bool ok = await sendCommand(packetCfg, maxWait) == SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_SENT; // We are only expecting an ACK
            if (ok)
            {
                autoPVT = enable;
                autoPVTImplicitUpdate = implicitUpdate;
            }
            moduleQueried.all = 0;
            return ok;
        }

        //In case no config access to the GPS is possible and HPPOSLLH is send cyclically already
        //set config to suitable parameters
        public bool assumeAutoHPPOSLLH(bool enabled, bool implicitUpdate)
        {
            bool changes = autoHPPOSLLH != enabled || autoHPPOSLLHImplicitUpdate != implicitUpdate;
            if (changes)
            {
                autoHPPOSLLH = enabled;
                autoHPPOSLLHImplicitUpdate = implicitUpdate;
            }
            return changes;
        }

        //Enable or disable automatic navigation message generation by the GPS. This changes the way getHPPOSLLH
        //works.
        public async Task<bool> setAutoHPPOSLLH(bool enable, ushort maxWait = Constants.DefaultMaxWait)
        {
            return await setAutoHPPOSLLH(enable, true, maxWait);
        }

        //Enable or disable automatic navigation message generation by the GPS. This changes the way getHPPOSLLH
        //works.
        public async Task<bool> setAutoHPPOSLLH(bool enable, bool implicitUpdate, ushort maxWait = Constants.DefaultMaxWait)
        {
            packetCfg.cls = Constants.UBX_CLASS_CFG;
            packetCfg.id = Constants.UBX_CFG_MSG;
            packetCfg.len = 3;
            packetCfg.startingSpot = 0;
            payloadCfg[0] = Constants.UBX_CLASS_NAV;
            payloadCfg[1] = Constants.UBX_NAV_HPPOSLLH;
            payloadCfg[2] = enable ? (byte)1 : (byte)0; // rate relative to navigation freq.

            bool ok = await sendCommand(packetCfg, maxWait) == SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_SENT; // We are only expecting an ACK
            if (ok)
            {
                autoHPPOSLLH = enable;
                autoHPPOSLLHImplicitUpdate = implicitUpdate;
            }
            highResModuleQueried.all = 0;
            return ok;
        }

        //Configure a given message type for a given port (UART1, I2C, SPI, etc)
        public async Task<bool> configureMessage(byte msgClass, byte msgID, byte portID, byte sendRate, ushort maxWait = Constants.DefaultMaxWait)
        {
            //Poll for the current settings for a given message
            packetCfg.cls = Constants.UBX_CLASS_CFG;
            packetCfg.id = Constants.UBX_CFG_MSG;
            packetCfg.len = 2;
            packetCfg.startingSpot = 0;

            payloadCfg[0] = msgClass;
            payloadCfg[1] = msgID;

            //This will load the payloadCfg array with current settings of the given register
            if (await sendCommand(packetCfg, maxWait) != SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
                return false;                                                       //If command send fails then bail

            //Now send it back with new mods
            packetCfg.len = 8;

            //payloadCfg is now loaded with current bytes. Change only the ones we need to
            payloadCfg[2 + portID] = sendRate; //Send rate is relative to the event a message is registered on. For example, if the rate of a navigation message is set to 2, the message is sent every 2nd navigation solution.

            return await sendCommand(packetCfg, maxWait) == SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_SENT; // We are only expecting an ACK
        }

        //Enable a given message type, default of 1 per update rate (usually 1 per second)
        public async Task<bool> enableMessage(byte msgClass, byte msgID, byte portID, byte rate, ushort maxWait = Constants.DefaultMaxWait)
        {
            return await configureMessage(msgClass, msgID, portID, rate, maxWait);
        }
        //Disable a given message type on a given port
        public async Task<bool> disableMessage(byte msgClass, byte msgID, byte portID, ushort maxWait = Constants.DefaultMaxWait)
        {
            return await configureMessage(msgClass, msgID, portID, 0, maxWait);
        }

        public async Task<bool> enableNMEAMessage(byte msgID, byte portID, byte rate, ushort maxWait = Constants.DefaultMaxWait)
        {
            return await configureMessage(Constants.UBX_CLASS_NMEA, msgID, portID, rate, maxWait);
        }
        public async Task<bool> disableNMEAMessage(byte msgID, byte portID, ushort maxWait = Constants.DefaultMaxWait)
        {
            return await enableNMEAMessage(msgID, portID, 0, maxWait);
        }

        //Given a message number turns on a message ID for output over a given portID (UART, I2C, SPI, USB, etc)
        //To disable a message, set secondsBetween messages to 0
        //Note: This function will return false if the message is already enabled
        //For base station RTK output we need to enable various sentences

        //NEO-M8P has four:
        //1005 = 0xF5 0x05 - Stationary RTK reference ARP
        //1077 = 0xF5 0x4D - GPS MSM7
        //1087 = 0xF5 0x57 - GLONASS MSM7
        //1230 = 0xF5 0xE6 - GLONASS code-phase biases, set to once every 10 seconds

        //ZED-F9P has six:
        //1005, 1074, 1084, 1094, 1124, 1230

        //Much of this configuration is not documented and instead discerned from u-center binary console
        public async Task<bool> enableRTCMmessage(byte messageNumber, byte portID, byte sendRate, ushort maxWait = Constants.DefaultMaxWait)
        {
            return await configureMessage(Constants.UBX_RTCM_MSB, messageNumber, portID, sendRate, maxWait);
        }

        //Disable a given message on a given port by setting secondsBetweenMessages to zero
        public async Task<bool> disableRTCMmessage(byte messageNumber, byte portID, ushort maxWait = Constants.DefaultMaxWait)
        {
            return await enableRTCMmessage(messageNumber, portID, 0, maxWait);
        }

        //Add a new geofence using UBX-CFG-GEOFENCE
        public async Task<bool> addGeofence(int latitude, int longitude, uint radius, byte confidence, byte pinPolarity, byte pin, ushort maxWait = 1100)
        {
            if (currentGeofenceParams.numFences >= 4)
                return false; // Quit if we already have four geofences defined

            // Store the new geofence parameters
            currentGeofenceParams.lats[currentGeofenceParams.numFences] = latitude;
            currentGeofenceParams.longs[currentGeofenceParams.numFences] = longitude;
            currentGeofenceParams.rads[currentGeofenceParams.numFences] = radius;
            currentGeofenceParams.numFences = (byte)(currentGeofenceParams.numFences + 1); // Increment the number of fences

            packetCfg.cls = Constants.UBX_CLASS_CFG;
            packetCfg.id = Constants.UBX_CFG_GEOFENCE;
            packetCfg.len = (ushort)((currentGeofenceParams.numFences * 12) + 8);
            packetCfg.startingSpot = 0;

            payloadCfg[0] = 0;                               // Message version = 0x00
            payloadCfg[1] = currentGeofenceParams.numFences; // numFences
            payloadCfg[2] = confidence;                      // confLvl = Confidence level 0-4 (none, 68%, 95%, 99.7%, 99.99%)
            payloadCfg[3] = 0;                               // reserved1
            if (pin > 0)
            {
                payloadCfg[4] = 1; // enable PIO combined fence state
            }
            else
            {
                payloadCfg[4] = 0; // disable PIO combined fence state
            }
            payloadCfg[5] = pinPolarity; // PIO pin polarity (0 = low means inside, 1 = low means outside (or unknown))
            payloadCfg[6] = pin;         // PIO pin
            payloadCfg[7] = 0;           //reserved2
            payloadCfg[8] = (byte)(currentGeofenceParams.lats[0] & 0xFF);
            payloadCfg[9] = (byte)(currentGeofenceParams.lats[0] >> 8);
            payloadCfg[10] = (byte)(currentGeofenceParams.lats[0] >> 16);
            payloadCfg[11] = (byte)(currentGeofenceParams.lats[0] >> 24);
            payloadCfg[12] = (byte)(currentGeofenceParams.longs[0] & 0xFF);
            payloadCfg[13] = (byte)(currentGeofenceParams.longs[0] >> 8);
            payloadCfg[14] = (byte)(currentGeofenceParams.longs[0] >> 16);
            payloadCfg[15] = (byte)(currentGeofenceParams.longs[0] >> 24);
            payloadCfg[16] = (byte)(currentGeofenceParams.rads[0] & 0xFF);
            payloadCfg[17] = (byte)(currentGeofenceParams.rads[0] >> 8);
            payloadCfg[18] = (byte)(currentGeofenceParams.rads[0] >> 16);
            payloadCfg[19] = (byte)(currentGeofenceParams.rads[0] >> 24);
            if (currentGeofenceParams.numFences >= 2)
            {
                payloadCfg[20] = (byte)(currentGeofenceParams.lats[1] & 0xFF);
                payloadCfg[21] = (byte)(currentGeofenceParams.lats[1] >> 8);
                payloadCfg[22] = (byte)(currentGeofenceParams.lats[1] >> 16);
                payloadCfg[23] = (byte)(currentGeofenceParams.lats[1] >> 24);
                payloadCfg[24] = (byte)(currentGeofenceParams.longs[1] & 0xFF);
                payloadCfg[25] = (byte)(currentGeofenceParams.longs[1] >> 8);
                payloadCfg[26] = (byte)(currentGeofenceParams.longs[1] >> 16);
                payloadCfg[27] = (byte)(currentGeofenceParams.longs[1] >> 24);
                payloadCfg[28] = (byte)(currentGeofenceParams.rads[1] & 0xFF);
                payloadCfg[29] = (byte)(currentGeofenceParams.rads[1] >> 8);
                payloadCfg[30] = (byte)(currentGeofenceParams.rads[1] >> 16);
                payloadCfg[31] = (byte)(currentGeofenceParams.rads[1] >> 24);
            }
            if (currentGeofenceParams.numFences >= 3)
            {
                payloadCfg[32] = (byte)(currentGeofenceParams.lats[2] & 0xFF);
                payloadCfg[33] = (byte)(currentGeofenceParams.lats[2] >> 8);
                payloadCfg[34] = (byte)(currentGeofenceParams.lats[2] >> 16);
                payloadCfg[35] = (byte)(currentGeofenceParams.lats[2] >> 24);
                payloadCfg[36] = (byte)(currentGeofenceParams.longs[2] & 0xFF);
                payloadCfg[37] = (byte)(currentGeofenceParams.longs[2] >> 8);
                payloadCfg[38] = (byte)(currentGeofenceParams.longs[2] >> 16);
                payloadCfg[39] = (byte)(currentGeofenceParams.longs[2] >> 24);
                payloadCfg[40] = (byte)(currentGeofenceParams.rads[2] & 0xFF);
                payloadCfg[41] = (byte)(currentGeofenceParams.rads[2] >> 8);
                payloadCfg[42] = (byte)(currentGeofenceParams.rads[2] >> 16);
                payloadCfg[43] = (byte)(currentGeofenceParams.rads[2] >> 24);
            }
            if (currentGeofenceParams.numFences >= 4)
            {
                payloadCfg[44] = (byte)(currentGeofenceParams.lats[3] & 0xFF);
                payloadCfg[45] = (byte)(currentGeofenceParams.lats[3] >> 8);
                payloadCfg[46] = (byte)(currentGeofenceParams.lats[3] >> 16);
                payloadCfg[47] = (byte)(currentGeofenceParams.lats[3] >> 24);
                payloadCfg[48] = (byte)(currentGeofenceParams.longs[3] & 0xFF);
                payloadCfg[49] = (byte)(currentGeofenceParams.longs[3] >> 8);
                payloadCfg[50] = (byte)(currentGeofenceParams.longs[3] >> 16);
                payloadCfg[51] = (byte)(currentGeofenceParams.longs[3] >> 24);
                payloadCfg[52] = (byte)(currentGeofenceParams.rads[3] & 0xFF);
                payloadCfg[53] = (byte)(currentGeofenceParams.rads[3] >> 8);
                payloadCfg[54] = (byte)(currentGeofenceParams.rads[3] >> 16);
                payloadCfg[55] = (byte)(currentGeofenceParams.rads[3] >> 24);
            }
            return await sendCommand(packetCfg, maxWait) == SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_SENT; // We are only expecting an ACK
        }

        //Clear all geofences using UBX-CFG-GEOFENCE
        public async Task<bool> clearGeofences(ushort maxWait = 1100)
        {
            packetCfg.cls = Constants.UBX_CLASS_CFG;
            packetCfg.id = Constants.UBX_CFG_GEOFENCE;
            packetCfg.len = 8;
            packetCfg.startingSpot = 0;

            payloadCfg[0] = 0; // Message version = 0x00
            payloadCfg[1] = 0; // numFences
            payloadCfg[2] = 0; // confLvl
            payloadCfg[3] = 0; // reserved1
            payloadCfg[4] = 0; // disable PIO combined fence state
            payloadCfg[5] = 0; // PIO pin polarity (0 = low means inside, 1 = low means outside (or unknown))
            payloadCfg[6] = 0; // PIO pin
            payloadCfg[7] = 0; //reserved2

            currentGeofenceParams.numFences = 0; // Zero the number of geofences currently in use

            return await sendCommand(packetCfg, maxWait) == SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_SENT; // We are only expecting an ACK
        }

        //Clear the antenna control settings using UBX-CFG-ANT
        //This function is hopefully redundant but may be needed to release
        //any PIO pins pre-allocated for antenna functions
        public async Task<bool> clearAntPIO(ushort maxWait = 1100)
        {
            packetCfg.cls = Constants.UBX_CLASS_CFG;
            packetCfg.id = Constants.UBX_CFG_ANT;
            packetCfg.len = 4;
            packetCfg.startingSpot = 0;

            payloadCfg[0] = 0x10; // Antenna flag mask: set the recovery bit
            payloadCfg[1] = 0;
            payloadCfg[2] = 0xFF; // Antenna pin configuration: set pinSwitch and pinSCD to 31
            payloadCfg[3] = 0xFF; // Antenna pin configuration: set pinOCD to 31, set reconfig bit

            return await sendCommand(packetCfg, maxWait) == SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_SENT; // We are only expecting an ACK
        }

        //Returns the combined geofence state using UBX-NAV-GEOFENCE
        public async Task<bool> getGeofenceState(geofenceState currentGeofenceState, ushort maxWait = 1100)
        {
            packetCfg.cls = Constants.UBX_CLASS_NAV;
            packetCfg.id = Constants.UBX_NAV_GEOFENCE;
            packetCfg.len = 0;
            packetCfg.startingSpot = 0;

            //Ask module for the geofence status. Loads into payloadCfg.
            if (await sendCommand(packetCfg, maxWait) != SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
                return false;

            currentGeofenceState.status = payloadCfg[5];    // Extract the status
            currentGeofenceState.numFences = payloadCfg[6]; // Extract the number of geofences
            currentGeofenceState.combState = payloadCfg[7]; // Extract the combined state of all geofences
            if (currentGeofenceState.numFences > 0)
                currentGeofenceState.states[0] = payloadCfg[8]; // Extract geofence 1 state
            if (currentGeofenceState.numFences > 1)
                currentGeofenceState.states[1] = payloadCfg[10]; // Extract geofence 2 state
            if (currentGeofenceState.numFences > 2)
                currentGeofenceState.states[2] = payloadCfg[12]; // Extract geofence 3 state
            if (currentGeofenceState.numFences > 3)
                currentGeofenceState.states[3] = payloadCfg[14]; // Extract geofence 4 state

            return true;
        }

        //Power Save Mode
        //Enables/Disables Low Power Mode using UBX-CFG-RXM
        public async Task<bool> powerSaveMode(bool power_save, ushort maxWait = 1100)
        {
            // Let's begin by checking the Protocol Version as UBX_CFG_RXM is not supported on the ZED (protocol >= 27)
            byte protVer = await getProtocolVersionHigh(maxWait);
            /*
            if (_printDebug == true)
            {
              _debugSerial.Write("Protocol version is "));
              _debugSerial.WriteLine(protVer);
            }
            */
            if (protVer >= 27)
            {
                if (_printDebug == true)
                {
                    _debugSerial.WriteLine("powerSaveMode (UBX-CFG-RXM) is not supported by this protocol version");
                }
                return false;
            }

            // Now let's change the power setting using UBX-CFG-RXM
            packetCfg.cls = Constants.UBX_CLASS_CFG;
            packetCfg.id = Constants.UBX_CFG_RXM;
            packetCfg.len = 0;
            packetCfg.startingSpot = 0;

            //Ask module for the current power management settings. Loads into payloadCfg.
            if (await sendCommand(packetCfg, maxWait) != SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
                return false;

            if (power_save)
            {
                payloadCfg[1] = 1; // Power Save Mode
            }
            else
            {
                payloadCfg[1] = 0; // Continuous Mode
            }

            packetCfg.len = 2;
            packetCfg.startingSpot = 0;

            return await sendCommand(packetCfg, maxWait) == SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_SENT; // We are only expecting an ACK
        }

        // Get Power Save Mode
        // Returns the current Low Power Mode using UBX-CFG-RXM
        // Returns 255 if the sendCommand fails
        public async Task<byte> getPowerSaveMode(ushort maxWait = 1100)
        {
            // Let's begin by checking the Protocol Version as UBX_CFG_RXM is not supported on the ZED (protocol >= 27)
            byte protVer = await getProtocolVersionHigh(maxWait);
            /*
            if (_printDebug == true)
            {
              _debugSerial.Write("Protocol version is "));
              _debugSerial.WriteLine(protVer);
            }
            */
            if (protVer >= 27)
            {
                if (_printDebug == true)
                {
                    _debugSerial.WriteLine("powerSaveMode (UBX-CFG-RXM) is not supported by this protocol version");
                }
                return (255);
            }

            // Now let's read the power setting using UBX-CFG-RXM
            packetCfg.cls = Constants.UBX_CLASS_CFG;
            packetCfg.id = Constants.UBX_CFG_RXM;
            packetCfg.len = 0;
            packetCfg.startingSpot = 0;

            //Ask module for the current power management settings. Loads into payloadCfg.
            if (await sendCommand(packetCfg, maxWait) != SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
                return 255;

            return payloadCfg[1]; // Return the low power mode
        }

        // Powers off the GPS device for a given duration to reduce power consumption.
        // NOTE: Querying the device before the duration is complete, for example by "getLatitude()" will wake it up!
        // Returns true if command has not been not acknowledged.
        // Returns false if command has not been acknowledged or maxWait = 0.
        public async Task<bool> powerOff(uint durationInMs, ushort maxWait = 1100)
        {
            // use durationInMs = 0 for infinite duration
            if (_printDebug == true)
            {
                //_debugSerial.Write("Powering off for "));
                //_debugSerial.Write(durationInMs);
                _debugSerial.WriteLine(" ms");
            }

            // Power off device using UBX-RXM-PMREQ
            packetCfg.cls = Constants.UBX_CLASS_RXM; // 0x02
            packetCfg.id = Constants.UBX_RXM_PMREQ;  // 0x41
            packetCfg.len = 8;
            packetCfg.startingSpot = 0;

            // duration
            // big endian to little endian, switch byte order
            payloadCfg[0] = (byte)((durationInMs >> (8 * 0)) & 0xff);
            payloadCfg[1] = (byte)((durationInMs >> (8 * 1)) & 0xff);
            payloadCfg[2] = (byte)((durationInMs >> (8 * 2)) & 0xff);
            payloadCfg[3] = (byte)((durationInMs >> (8 * 3)) & 0xff);

            payloadCfg[4] = 0x02; //Flags : set the backup bit
            payloadCfg[5] = 0x00; //Flags
            payloadCfg[6] = 0x00; //Flags
            payloadCfg[7] = 0x00; //Flags

            if (maxWait != 0)
            {
                // check for "not acknowledged" command
                return await sendCommand(packetCfg, maxWait) != SfeUbloxStatus.SFE_UBLOX_STATUS_COMMAND_NACK;
            }
            else
            {
                await sendCommand(packetCfg, maxWait);
                return false; // can't tell if command not acknowledged if maxWait = 0
            }
        }

        // Powers off the GPS device for a given duration to reduce power consumption.
        // While powered off it can be woken up by creating a falling or rising voltage edge on the specified pin.
        // NOTE: The GPS seems to be sensitve to signals on the pins while powered off. Works best when Microcontroller is in deepsleep.
        // NOTE: Querying the device before the duration is complete, for example by "getLatitude()" will wake it up!
        // Returns true if command has not been not acknowledged.
        // Returns false if command has not been acknowledged or maxWait = 0.
        public async Task<bool> powerOffWithInterrupt(uint durationInMs, uint wakeupSources, bool forceWhileUsb, ushort maxWait = 1100)
        {
            // use durationInMs = 0 for infinite duration
            if (_printDebug == true)
            {
                _debugSerial.Write("Powering off for ");
                _debugSerial.Write(durationInMs.ToString());
                _debugSerial.WriteLine(" ms");
            }

            // Power off device using UBX-RXM-PMREQ
            packetCfg.cls = Constants.UBX_CLASS_RXM; // 0x02
            packetCfg.id = Constants.UBX_RXM_PMREQ;  // 0x41
            packetCfg.len = 16;
            packetCfg.startingSpot = 0;

            payloadCfg[0] = 0x00; // message version

            // bytes 1-3 are reserved - and must be set to zero
            payloadCfg[1] = 0x00;
            payloadCfg[2] = 0x00;
            payloadCfg[3] = 0x00;

            // duration
            // big endian to little endian, switch byte order
            payloadCfg[4] = (byte)((durationInMs >> (8 * 0)) & 0xff);
            payloadCfg[5] = (byte)((durationInMs >> (8 * 1)) & 0xff);
            payloadCfg[6] = (byte)((durationInMs >> (8 * 2)) & 0xff);
            payloadCfg[7] = (byte)((durationInMs >> (8 * 3)) & 0xff);

            // flags

            // disables USB interface when powering off, defaults to true
            if (forceWhileUsb)
            {
                payloadCfg[8] = 0x06; // force | backup
            }
            else
            {
                payloadCfg[8] = 0x02; // backup only (leave the force bit clear - module will stay on if USB is connected)
            }

            payloadCfg[9] = 0x00;
            payloadCfg[10] = 0x00;
            payloadCfg[11] = 0x00;

            // wakeUpSources

            // wakeupPin mapping, defaults to VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT0

            // Possible values are:
            // VAL_RXM_PMREQ_WAKEUPSOURCE_UARTRX
            // VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT0
            // VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT1
            // VAL_RXM_PMREQ_WAKEUPSOURCE_SPICS

            payloadCfg[12] = (byte)((wakeupSources >> (8 * 0)) & 0xff);
            payloadCfg[13] = (byte)((wakeupSources >> (8 * 1)) & 0xff);
            payloadCfg[14] = (byte)((wakeupSources >> (8 * 2)) & 0xff);
            payloadCfg[15] = (byte)((wakeupSources >> (8 * 3)) & 0xff);

            if (maxWait != 0)
            {
                // check for "not acknowledged" command
                return await sendCommand(packetCfg, maxWait) != SfeUbloxStatus.SFE_UBLOX_STATUS_COMMAND_NACK;
            }
            else
            {
                await sendCommand(packetCfg, maxWait);
                return false; // can't tell if command not acknowledged if maxWait = 0
            }
        }

        //Change the dynamic platform model using UBX-CFG-NAV5
        //Possible values are:
        //PORTABLE,STATIONARY,PEDESTRIAN,AUTOMOTIVE,SEA,
        //AIRBORNE1g,AIRBORNE2g,AIRBORNE4g,WRIST,BIKE
        //WRIST is not supported in protocol versions less than 18
        //BIKE is supported in protocol versions 19.2
        public async Task<bool> setDynamicModel(DynModel newDynamicModel, ushort maxWait = 1100)
        {
            packetCfg.cls = Constants.UBX_CLASS_CFG;
            packetCfg.id = Constants.UBX_CFG_NAV5;
            packetCfg.len = 0;
            packetCfg.startingSpot = 0;

            //Ask module for the current navigation model settings. Loads into payloadCfg.
            if(await sendCommand(packetCfg, maxWait) != SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
                return false;

            payloadCfg[0] = 0x01;            // mask: set only the dyn bit (0)
            payloadCfg[1] = 0x00;            // mask
            payloadCfg[2] = (byte)newDynamicModel; // dynModel

            packetCfg.len = 36;
            packetCfg.startingSpot = 0;

            return await sendCommand(packetCfg, maxWait) == SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_SENT; // We are only expecting an ACK
        }

        //Get the dynamic platform model using UBX-CFG-NAV5
        //Returns 255 if the sendCommand fails
        public async Task<byte> getDynamicModel(ushort maxWait = 1100)
        {
            packetCfg.cls = Constants.UBX_CLASS_CFG;
            packetCfg.id = Constants.UBX_CFG_NAV5;
            packetCfg.len = 0;
            packetCfg.startingSpot = 0;

            //Ask module for the current navigation model settings. Loads into payloadCfg.
            if (await sendCommand(packetCfg, maxWait) != SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
                return 255;

            return payloadCfg[2]; // Return the dynamic model
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
            val |= (ushort)(payloadCfg[spotToStart + 0] << 8 * 0);
            val |= (ushort)(payloadCfg[spotToStart + 1] << 8 * 1);
            return val;
        }

        //Given a spot, extract a byte from the payload
        byte extractByte(byte spotToStart)
        {
            return payloadCfg[spotToStart];
        }

        //Given a spot, extract a signed 8-bit value from the payload
        byte extractSignedChar(byte spotToStart)
        {
            return payloadCfg[spotToStart];
        }

        //Get the current year
        public async Task<ushort> getYear(ushort maxWait = Constants.getPVTmaxWait)
        {
            if (moduleQueried.gpsYear == 0)
                await getPVT(maxWait);
            moduleQueried.gpsYear = 0; //Since we are about to give this to user, mark this data as stale
            return (gpsYear);
        }

        //Get the current month
        public async Task<byte> getMonth(ushort maxWait = Constants.getPVTmaxWait)
        {
            if (moduleQueried.gpsMonth == 0)
                await getPVT(maxWait);
            moduleQueried.gpsMonth = 0; //Since we are about to give this to user, mark this data as stale
            return (gpsMonth);
        }

        //Get the current day
        public async Task<byte> getDay(ushort maxWait = Constants.getPVTmaxWait)
        {
            if (moduleQueried.gpsDay == 0)
                await getPVT(maxWait);
            moduleQueried.gpsDay = 0; //Since we are about to give this to user, mark this data as stale
            return (gpsDay);
        }

        //Get the current hour
        public async Task<byte> getHour(ushort maxWait = Constants.getPVTmaxWait)
        {
            if (moduleQueried.gpsHour == 0)
                await getPVT(maxWait);
            moduleQueried.gpsHour = 0; //Since we are about to give this to user, mark this data as stale
            return gpsHour;
        }

        //Get the current minute
        public async Task<byte> getMinute(ushort maxWait = Constants.getPVTmaxWait)
        {
            if (moduleQueried.gpsMinute == 0)
                await getPVT(maxWait);
            moduleQueried.gpsMinute = 0; //Since we are about to give this to user, mark this data as stale
            return gpsMinute;
        }

        //Get the current second
        public async Task<byte> getSecond(ushort maxWait = Constants.getPVTmaxWait)
        {
            if (moduleQueried.gpsSecond == 0)
                await getPVT(maxWait);
            moduleQueried.gpsSecond = 0; //Since we are about to give this to user, mark this data as stale
            return gpsSecond;
        }

        //Get the current date validity
        public async Task<bool> getDateValid(ushort maxWait = Constants.getPVTmaxWait)
        {
            if (moduleQueried.gpsDateValid == 0)
                await getPVT(maxWait);
            moduleQueried.gpsDateValid = 0; //Since we are about to give this to user, mark this data as stale
            return gpsDateValid;
        }

        //Get the current time validity
        public async Task<bool> getTimeValid(ushort maxWait = Constants.getPVTmaxWait)
        {
            if (moduleQueried.gpsTimeValid == 0)
                await getPVT(maxWait);
            moduleQueried.gpsTimeValid = 0; //Since we are about to give this to user, mark this data as stale
            return gpsTimeValid;
        }

        //Get the current millisecond
        public async Task<ushort> getMillisecond(ushort maxWait = Constants.getPVTmaxWait)
        {
            if (moduleQueried.gpsiTOW == 0)
                await getPVT(maxWait);
            moduleQueried.gpsiTOW = 0; //Since we are about to give this to user, mark this data as stale
            return gpsMillisecond;
        }

        //Get the current nanoseconds - includes milliseconds
        public async Task<int> getNanosecond(ushort maxWait = Constants.getPVTmaxWait)
        {
            if (moduleQueried.gpsNanosecond == 0)
                await getPVT(maxWait);
            moduleQueried.gpsNanosecond = 0; //Since we are about to give this to user, mark this data as stale
            return gpsNanosecond;
        }

        //Get the latest Position/Velocity/Time solution and fill all global variables
        public async Task<bool> getPVT(ushort maxWait = Constants.getPVTmaxWait)
        {
            if (autoPVT && autoPVTImplicitUpdate)
            {
                //The GPS is automatically reporting, we just check whether we got unread data
                if (_printDebug == true)
                {
                    _debugSerial.WriteLine("getPVT: Autoreporting");
                }
                await checkUbloxInternal(packetCfg, Constants.UBX_CLASS_NAV, Constants.UBX_NAV_PVT);
                return moduleQueried.all > 0;
            }
            else if (autoPVT && !autoPVTImplicitUpdate)
            {
                //Someone else has to call checkUblox for us...
                if (_printDebug == true)
                {
                    _debugSerial.WriteLine("getPVT: Exit immediately");
                }
                return false;
            }
            else
            {
                if (_printDebug == true)
                {
                    _debugSerial.WriteLine("getPVT: Polling");
                }

                //The GPS is not automatically reporting navigation position so we have to poll explicitly
                packetCfg.cls = Constants.UBX_CLASS_NAV;
                packetCfg.id = Constants.UBX_NAV_PVT;
                packetCfg.len = 0;
                //packetCfg.startingSpot = 20; //Begin listening at spot 20 so we can record up to 20+Constants.MAX_PAYLOAD_SIZE = 84 bytes Note:now hard-coded in processUBX

                //The data is parsed as part of processing the response
                SfeUbloxStatus retVal = await sendCommand(packetCfg, maxWait);

                if (retVal == SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_RECEIVED)
                    return true;

                if ((retVal == SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_OVERWRITTEN) && (packetCfg.id == Constants.UBX_NAV_HPPOSLLH))
                {
                    if (_printDebug == true)
                    {
                        _debugSerial.WriteLine("getPVT: data was OVERWRITTEN by HPPOSLLH (but that's OK)");
                    }
                    return true;
                }

                if (_printDebug == true)
                {
                    _debugSerial.Write("getPVT retVal: ");
                    _debugSerial.WriteLine(statusString(retVal));
                }
                return false;
            }
        }

        public async Task<uint> getTimeOfWeek(ushort maxWait = Constants.getPVTmaxWait)
        {
            if (moduleQueried.gpsiTOW == 0)
                await getPVT(maxWait);
            moduleQueried.gpsiTOW = 0; //Since we are about to give this to user, mark this data as stale
            return timeOfWeek;
        }

        public async Task<int> getHighResLatitude(ushort maxWait = Constants.getHPPOSLLHmaxWait)
        {
            if (highResModuleQueried.highResLatitude == 0)
                await getHPPOSLLH(maxWait);
            highResModuleQueried.highResLatitude = 0; //Since we are about to give this to user, mark this data as stale
            highResModuleQueried.all = 0;

            return highResLatitude;
        }

        public async Task<byte> getHighResLatitudeHp(ushort maxWait = Constants.getHPPOSLLHmaxWait)
        {
            if (highResModuleQueried.highResLatitudeHp == 0)
                await getHPPOSLLH(maxWait);
            highResModuleQueried.highResLatitudeHp = 0; //Since we are about to give this to user, mark this data as stale
            highResModuleQueried.all = 0;

            return highResLatitudeHp;
        }

        public async Task<int> getHighResLongitude(ushort maxWait = Constants.getHPPOSLLHmaxWait)
        {
            if (highResModuleQueried.highResLongitude == 0)
                await getHPPOSLLH(maxWait);
            highResModuleQueried.highResLongitude = 0; //Since we are about to give this to user, mark this data as stale
            highResModuleQueried.all = 0;

            return highResLongitude;
        }

        public async Task<byte> getHighResLongitudeHp(ushort maxWait = Constants.getHPPOSLLHmaxWait)
        {
            if (highResModuleQueried.highResLongitudeHp == 0)
                await getHPPOSLLH(maxWait);
            highResModuleQueried.highResLongitudeHp = 0; //Since we are about to give this to user, mark this data as stale
            highResModuleQueried.all = 0;

            return (highResLongitudeHp);
        }

        public async Task<int> getElipsoid(ushort maxWait = Constants.getHPPOSLLHmaxWait)
        {
            if (highResModuleQueried.elipsoid == 0)
                await getHPPOSLLH(maxWait);
            highResModuleQueried.elipsoid = 0; //Since we are about to give this to user, mark this data as stale
            highResModuleQueried.all = 0;

            return (elipsoid);
        }

        public async Task<byte> getElipsoidHp(ushort maxWait = Constants.getHPPOSLLHmaxWait)
        {
            if (highResModuleQueried.elipsoidHp == 0)
                await getHPPOSLLH(maxWait);
            highResModuleQueried.elipsoidHp = 0; //Since we are about to give this to user, mark this data as stale
            highResModuleQueried.all = 0;

            return (elipsoidHp);
        }

        public async Task<int> getMeanSeaLevel(ushort maxWait = Constants.getHPPOSLLHmaxWait)
        {
            if (highResModuleQueried.meanSeaLevel == 0)
                await getHPPOSLLH(maxWait);
            highResModuleQueried.meanSeaLevel = 0; //Since we are about to give this to user, mark this data as stale
            highResModuleQueried.all = 0;

            return (meanSeaLevel);
        }

        public async Task<byte> getMeanSeaLevelHp(ushort maxWait = Constants.getHPPOSLLHmaxWait)
        {
            if (highResModuleQueried.meanSeaLevelHp == 0)
                await getHPPOSLLH(maxWait);
            highResModuleQueried.meanSeaLevelHp = 0; //Since we are about to give this to user, mark this data as stale
            highResModuleQueried.all = 0;

            return (meanSeaLevelHp);
        }

        // getGeoidSeparation is currently redundant. The geoid separation seems to only be provided in NMEA GGA and GNS messages.
        public async Task<int> getGeoidSeparation(ushort maxWait = Constants.getHPPOSLLHmaxWait)
        {
            if (highResModuleQueried.geoidSeparation == 0)
                await getHPPOSLLH(maxWait);
            highResModuleQueried.geoidSeparation = 0; //Since we are about to give this to user, mark this data as stale
            highResModuleQueried.all = 0;

            return (geoidSeparation);
        }

        public async Task<uint> getHorizontalAccuracy(ushort maxWait = Constants.getHPPOSLLHmaxWait)
        {
            if (highResModuleQueried.horizontalAccuracy == 0)
                await getHPPOSLLH(maxWait);
            highResModuleQueried.horizontalAccuracy = 0; //Since we are about to give this to user, mark this data as stale
            highResModuleQueried.all = 0;

            return (horizontalAccuracy);
        }

        public async Task<uint> getVerticalAccuracy(ushort maxWait = Constants.getHPPOSLLHmaxWait)
        {
            if (highResModuleQueried.verticalAccuracy == 0)
                await getHPPOSLLH(maxWait);
            highResModuleQueried.verticalAccuracy = 0; //Since we are about to give this to user, mark this data as stale
            highResModuleQueried.all = 0;

            return (verticalAccuracy);
        }

        public async Task<bool> getHPPOSLLH(ushort maxWait)
        {
            if (autoHPPOSLLH && autoHPPOSLLHImplicitUpdate)
            {
                //The GPS is automatically reporting, we just check whether we got unread data
                if (_printDebug == true)
                {
                    _debugSerial.WriteLine("getHPPOSLLH: Autoreporting");
                }
                await checkUbloxInternal(packetCfg, Constants.UBX_CLASS_NAV, Constants.UBX_NAV_HPPOSLLH);
                return highResModuleQueried.all > 0 ? true : false;
            }
            else if (autoHPPOSLLH && !autoHPPOSLLHImplicitUpdate)
            {
                //Someone else has to call checkUblox for us...
                if (_printDebug == true)
                {
                    _debugSerial.WriteLine("getHPPOSLLH: Exit immediately");
                }
                return false;
            }
            else
            {
                if (_printDebug == true)
                {
                    _debugSerial.WriteLine("getHPPOSLLH: Polling");
                }

                //The GPS is not automatically reporting navigation position so we have to poll explicitly
                packetCfg.cls = Constants.UBX_CLASS_NAV;
                packetCfg.id = Constants.UBX_NAV_HPPOSLLH;
                packetCfg.len = 0;

                //The data is parsed as part of processing the response
                SfeUbloxStatus retVal = await sendCommand(packetCfg, maxWait);

                if (retVal == SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_RECEIVED)
                    return true;

                if ((retVal == SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_OVERWRITTEN) && (packetCfg.id == Constants.UBX_NAV_PVT))
                {
                    if (_printDebug == true)
                    {
                        _debugSerial.WriteLine("getHPPOSLLH: data was OVERWRITTEN by PVT (but that's OK)");
                    }
                    return true;
                }

                if (_printDebug == true)
                {
                    _debugSerial.Write("getHPPOSLLH retVal: ");
                    _debugSerial.WriteLine(statusString(retVal));
                }
                return false;
            }
        }

        //Get the current 3D high precision positional accuracy - a fun thing to watch
        //Returns a long representing the 3D accuracy in millimeters
        public async Task<uint> getPositionAccuracy(ushort maxWait = 1100)
        {
            packetCfg.cls = Constants.UBX_CLASS_NAV;
            packetCfg.id = Constants.UBX_NAV_HPPOSECEF;
            packetCfg.len = 0;
            packetCfg.startingSpot = 0;

            if (await sendCommand(packetCfg, maxWait) != SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_RECEIVED) // We are only expecting data (no ACK)
                return 0;                                                           //If command send fails then bail

            uint tempAccuracy = extractLong(24); //We got a response, now extract a long beginning at a given position

            if ((tempAccuracy % 10) >= 5)
                tempAccuracy += 5; //Round fraction of mm up to next mm if .5 or above
            tempAccuracy /= 10;  //Convert 0.1mm units to mm

            return (tempAccuracy);
        }

        //Get the current latitude in degrees
        //Returns a long representing the number of degrees *10^-7
        public async Task<int> getLatitude(ushort maxWait = Constants.getPVTmaxWait)
        {
            if (moduleQueried.latitude == 0) await getPVT(maxWait);
            moduleQueried.latitude = 0; //Since we are about to give this to user, mark this data as stale
            moduleQueried.all = 0;

            return latitude;
        }

        //Get the current longitude in degrees
        //Returns a long representing the number of degrees *10^-7
        public async Task<int> getLongitude(ushort maxWait = Constants.getPVTmaxWait)
        {
            if (moduleQueried.longitude == 0) await getPVT(maxWait);
            moduleQueried.longitude = 0; //Since we are about to give this to user, mark this data as stale
            moduleQueried.all = 0;

            return longitude;
        }

        //Get the current altitude in mm according to ellipsoid model
        public async Task<int> getAltitude(ushort maxWait = Constants.getPVTmaxWait)
        {
            if (moduleQueried.altitude == 0)
                await getPVT(maxWait);
            moduleQueried.altitude = 0; //Since we are about to give this to user, mark this data as stale
            moduleQueried.all = 0;

            return altitude;
        }

        //Get the current altitude in mm according to mean sea level
        //Ellipsoid model: https://www.esri.com/news/arcuser/0703/geoid1of3.html
        //Difference between Ellipsoid Model and Mean Sea Level: https://eos-gnss.com/elevation-for-beginners/
        public async Task<int> getAltitudeMSL(ushort maxWait = Constants.getPVTmaxWait)
        {
            if (moduleQueried.altitudeMSL == 0)
                await getPVT(maxWait);
            moduleQueried.altitudeMSL = 0; //Since we are about to give this to user, mark this data as stale
            moduleQueried.all = 0;

            return altitudeMSL;
        }

        //Get the number of satellites used in fix
        public async Task<byte> getSIV(ushort maxWait = Constants.getPVTmaxWait)
        {
            if (moduleQueried.SIV == 0)
                await getPVT(maxWait);
            moduleQueried.SIV = 0; //Since we are about to give this to user, mark this data as stale
            moduleQueried.all = 0;

            return SIV;
        }

        //Get the current fix type
        //0=no fix, 1=dead reckoning, 2=2D, 3=3D, 4=GNSS, 5=Time fix
        public async Task<byte> getFixType(ushort maxWait = Constants.getPVTmaxWait)
        {
            if (moduleQueried.fixType == 0)
            {
                await getPVT(maxWait);
            }
            moduleQueried.fixType = 0; //Since we are about to give this to user, mark this data as stale
            moduleQueried.all = 0;

            return (fixType);
        }

        //Get the carrier phase range solution status
        //Useful when querying module to see if it has high-precision RTK fix
        //0=No solution, 1=Float solution, 2=Fixed solution
        public async Task<byte> getCarrierSolutionType(ushort maxWait = Constants.getPVTmaxWait)
        {
            if (moduleQueried.carrierSolution == 0)
                await getPVT(maxWait);
            moduleQueried.carrierSolution = 0; //Since we are about to give this to user, mark this data as stale
            moduleQueried.all = 0;

            return (carrierSolution);
        }

        //Get the ground speed in mm/s
        public async Task<int> getGroundSpeed(ushort maxWait = Constants.getPVTmaxWait)
        {
            if (moduleQueried.groundSpeed == 0)
                await getPVT(maxWait);
            moduleQueried.groundSpeed = 0; //Since we are about to give this to user, mark this data as stale
            moduleQueried.all = 0;

            return groundSpeed;
        }

        //Get the heading of motion (as opposed to heading of car) in degrees * 10^-5
        public async Task<int> getHeading(ushort maxWait = Constants.getPVTmaxWait)
        {
            if (moduleQueried.headingOfMotion == 0)
                await getPVT(maxWait);
            moduleQueried.headingOfMotion = 0; //Since we are about to give this to user, mark this data as stale
            moduleQueried.all = 0;

            return (headingOfMotion);
        }

        //Get the positional dillution of precision * 10^-2 (dimensionless)
        public async Task<ushort> getPDOP(ushort maxWait = Constants.getPVTmaxWait)
        {
            if (moduleQueried.pDOP == 0)
                await getPVT(maxWait);
            moduleQueried.pDOP = 0; //Since we are about to give this to user, mark this data as stale
            moduleQueried.all = 0;

            return (pDOP);
        }

        //Get the current protocol version of the u-blox module we're communicating with
        //This is helpful when deciding if we should call the high-precision Lat/Long (HPPOSLLH) or the regular (POSLLH)
        public async Task<byte> getProtocolVersionHigh(ushort maxWait = 500)
        {
            if (moduleQueried.versionNumber == 0)
                await getProtocolVersion(maxWait);
            return versionHigh;
        }

        //Get the current protocol version of the u-blox module we're communicating with
        //This is helpful when deciding if we should call the high-precision Lat/Long (HPPOSLLH) or the regular (POSLLH)
        public async Task<byte> getProtocolVersionLow(ushort maxWait = 500)
        {
            if (moduleQueried.versionNumber == 0)
                await getProtocolVersion(maxWait);
            return versionLow;
        }

        //Get the current protocol version of the u-blox module we're communicating with
        //This is helpful when deciding if we should call the high-precision Lat/Long (HPPOSLLH) or the regular (POSLLH)
        public async Task<bool> getProtocolVersion(ushort maxWait = 500)
        {
            //Send packet with only CLS and ID, length of zero. This will cause the module to respond with the contents of that CLS/ID.
            packetCfg.cls = Constants.UBX_CLASS_MON;
            packetCfg.id = Constants.UBX_MON_VER;

            packetCfg.len = 0;
            packetCfg.startingSpot = 40; //Start at first "extended software information" string

            if (await sendCommand(packetCfg, maxWait) != SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_RECEIVED) // We are only expecting data (no ACK)
                return false;                                                       //If command send fails then bail

            //Payload should now contain ~220 characters (depends on module type)

            if (_printDebug == true)
            {
                //_debugSerial.Write("MON VER Payload:"));
                //for (int location = 0; location < packetCfg.len; location++)
                //{
                //    if (location % 30 == 0)
                //        _debugSerial.WriteLine();
                //    _debugSerial.write(payloadCfg[location]);
                //}
                //_debugSerial.WriteLine();
            }

            //We will step through the payload looking at each extension field of 30 bytes
            for (byte extensionNumber = 0; extensionNumber < 10; extensionNumber++)
            {
                //Now we need to find "PROTVER=18.00" in the incoming byte stream
                if (payloadCfg[(30 * extensionNumber) + 0] == 'P' && payloadCfg[(30 * extensionNumber) + 6] == 'R')
                {
                    versionHigh = (byte)((payloadCfg[(30 * extensionNumber) + 8] - '0') * 10 + (payloadCfg[(30 * extensionNumber) + 9] - '0'));  //Convert '18' to 18
                    versionLow = (byte)((payloadCfg[(30 * extensionNumber) + 11] - '0') * 10 + (payloadCfg[(30 * extensionNumber) + 12] - '0')); //Convert '00' to 00
                    moduleQueried.versionNumber = 1;                                                                                  //Mark this data as new

                    if (_printDebug == true)
                    {
                        //_debugSerial.Write("Protocol version: ");
                        //_debugSerial.Write((versionHigh);
                        //_debugSerial.Write("."));
                        //_debugSerial.WriteLine(versionLow);
                    }
                    return true; //Success!
                }
            }

            return false; //We failed
        }

        //Mark all the PVT data as read/stale. This is handy to get data alignment after CRC failure
        public void flushPVT()
        {
            //Mark all datums as stale (read before)
            moduleQueried.gpsiTOW = 0;
            moduleQueried.gpsYear = 0;
            moduleQueried.gpsMonth = 0;
            moduleQueried.gpsDay = 0;
            moduleQueried.gpsHour = 0;
            moduleQueried.gpsMinute = 0;
            moduleQueried.gpsSecond = 0;
            moduleQueried.gpsDateValid = 0;
            moduleQueried.gpsTimeValid = 0;
            moduleQueried.gpsNanosecond = 0;

            moduleQueried.all = 0;
            moduleQueried.longitude = 0;
            moduleQueried.latitude = 0;
            moduleQueried.altitude = 0;
            moduleQueried.altitudeMSL = 0;
            moduleQueried.SIV = 0;
            moduleQueried.fixType = 0;
            moduleQueried.carrierSolution = 0;
            moduleQueried.groundSpeed = 0;
            moduleQueried.headingOfMotion = 0;
            moduleQueried.pDOP = 0;
        }

        //Mark all the HPPOSLLH data as read/stale. This is handy to get data alignment after CRC failure
        public void flushHPPOSLLH()
        {
            //Mark all datums as stale (read before)
            highResModuleQueried.all = 0;
            highResModuleQueried.highResLatitude = 0;
            highResModuleQueried.highResLatitudeHp = 0;
            highResModuleQueried.highResLongitude = 0;
            highResModuleQueried.highResLongitudeHp = 0;
            highResModuleQueried.elipsoid = 0;
            highResModuleQueried.elipsoidHp = 0;
            highResModuleQueried.meanSeaLevel = 0;
            highResModuleQueried.meanSeaLevelHp = 0;
            highResModuleQueried.geoidSeparation = 0;
            highResModuleQueried.horizontalAccuracy = 0;
            highResModuleQueried.verticalAccuracy = 0;
            //moduleQueried.gpsiTOW = false; // this can arrive via HPPOS too.
        }

        //Relative Positioning Information in NED frame
        //Returns true if commands was successful
        public async Task<bool> getRELPOSNED(ushort maxWait = 1100)
        {
            packetCfg.cls = Constants.UBX_CLASS_NAV;
            packetCfg.id = Constants.UBX_NAV_RELPOSNED;
            packetCfg.len = 0;
            packetCfg.startingSpot = 0;

            if (await sendCommand(packetCfg, maxWait) != SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_RECEIVED) // We are only expecting data (no ACK)
                return false;                                                       //If command send fails then bail

            //We got a response, now parse the bits

            ushort refStationID = extractInt(2);
            //_debugSerial.Write("refStationID: "));
            //_debugSerial.WriteLine(refStationID));

            int tempRelPos;

            tempRelPos = (int)extractLong(8);
            relPosInfo.relPosN = tempRelPos / 100.0F; //Convert cm to m

            tempRelPos = (int)extractLong(12);
            relPosInfo.relPosE = tempRelPos / 100.0F; //Convert cm to m

            tempRelPos = (int)extractLong(16);
            relPosInfo.relPosD = tempRelPos / 100.0F; //Convert cm to m

            relPosInfo.relPosLength = extractLong(20);
            relPosInfo.relPosHeading = extractLong(24);

            relPosInfo.relPosHPN = payloadCfg[32];
            relPosInfo.relPosHPE = payloadCfg[33];
            relPosInfo.relPosHPD = payloadCfg[34];
            relPosInfo.relPosHPLength = payloadCfg[35];

            uint tempAcc;

            tempAcc = extractLong(36);
            relPosInfo.accN = tempAcc / 10000.0F; //Convert 0.1 mm to m

            tempAcc = extractLong(40);
            relPosInfo.accE = tempAcc / 10000.0F; //Convert 0.1 mm to m

            tempAcc = extractLong(44);
            relPosInfo.accD = tempAcc / 10000.0F; //Convert 0.1 mm to m

            byte flags = payloadCfg[60];

            relPosInfo.gnssFixOk = (flags & (1 << 0)) > 0;
            relPosInfo.diffSoln = (flags & (1 << 1)) > 0;
            relPosInfo.relPosValid = (flags & (1 << 2)) > 0;
            relPosInfo.carrSoln = (byte)((flags & (0b11 << 3)) >> 3);
            relPosInfo.isMoving = (flags & (1 << 5)) > 0;
            relPosInfo.refPosMiss = (flags & (1 << 6)) > 0;
            relPosInfo.refObsMiss = (flags & (1 << 7)) > 0;

            return true;
        }
        public async Task<bool> getEsfInfo(ushort maxWait = 1100)
        {
            // Requesting Data from the receiver
            packetCfg.cls = Constants.UBX_CLASS_ESF;
            packetCfg.id = Constants.UBX_ESF_STATUS;
            packetCfg.len = 0;
            packetCfg.startingSpot = 0;

            if (await sendCommand(packetCfg, maxWait) != SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_RECEIVED)
                return false; //If command send fails then bail

            await checkUblox();

            // payload should be loaded.
            imuMeas.version = extractByte(4);
            imuMeas.fusionMode = extractByte(12);
            ubloxSen.numSens = extractByte(15);

            // Individual Status Sensor in different function
            return true;
        }

        //
        public async Task<bool> getEsfIns(ushort maxWait = 1100)
        {
            packetCfg.cls = Constants.UBX_CLASS_ESF;
            packetCfg.id = Constants.UBX_ESF_INS;
            packetCfg.len = 0;
            packetCfg.startingSpot = 0;

            if (await sendCommand(packetCfg, maxWait) != SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_RECEIVED)
                return false; //If command send fails then bail

            await checkUblox();

            // Validity of each sensor value below
            uint validity = extractLong(0);

            imuMeas.xAngRateVald = (byte)((validity & 0x0080) >> 8);
            imuMeas.yAngRateVald = (byte)((validity & 0x0100) >> 9);
            imuMeas.zAngRateVald = (byte)((validity & 0x0200) >> 10);
            imuMeas.xAccelVald = (byte)((validity & 0x0400) >> 11);
            imuMeas.yAccelVald = (byte)((validity & 0x0800) >> 12);
            imuMeas.zAccelVald = (byte)((validity & 0x1000) >> 13);

            imuMeas.xAngRate = (int)extractLong(12); // deg/s
            imuMeas.yAngRate = (int)extractLong(16); // deg/s
            imuMeas.zAngRate = (int)extractLong(20); // deg/s

            imuMeas.xAccel = (int)extractLong(24); // m/s
            imuMeas.yAccel = (int)extractLong(28); // m/s
            imuMeas.zAccel = (int)extractLong(32); // m/s

            return true;
        }

        //
        public async Task<bool> getEsfDataInfo(ushort maxWait = 1100)
        {

            packetCfg.cls = Constants.UBX_CLASS_ESF;
            packetCfg.id = Constants.UBX_ESF_MEAS;
            packetCfg.len = 0;
            packetCfg.startingSpot = 0;

            if (await sendCommand(packetCfg, maxWait) != SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_RECEIVED)
                return false; //If command send fails then bail

            await checkUblox();

            uint timeStamp = extractLong(0);
            uint flags = extractInt(4);

            byte timeSent = (byte)((flags & 0x01) >> 1);
            byte timeEdge = (byte)((flags & 0x02) >> 2);
            byte tagValid = (byte)((flags & 0x04) >> 3);
            byte numMeas = (byte)((flags & 0x1000) >> 15);

            if (numMeas > Constants.DEF_NUM_SENS)
                numMeas = Constants.DEF_NUM_SENS;

            byte byteOffset = 4;

            for (byte i = 0; i < numMeas; i++)
            {

                uint bitField = extractLong((byte)(4 + byteOffset * i));
                imuMeas.dataType[i] = (bitField & 0xFF000000) >> 23;
                imuMeas.data[i] = (bitField & 0xFFFFFF);
                imuMeas.dataTStamp[i] = extractLong((byte)(8 + byteOffset * i));
            }

            return true;
        }

        public async Task<bool> getEsfRawDataInfo(ushort maxWait = 1100)
        {

            // Need to know the number of sensor to get the correct data
            // Rate selected in UBX-CFG-MSG is not respected
            packetCfg.cls = Constants.UBX_CLASS_ESF;
            packetCfg.id = Constants.UBX_ESF_RAW;
            packetCfg.len = 0;
            packetCfg.startingSpot = 0;

            if (await sendCommand(packetCfg, maxWait) != SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_RECEIVED)
                return false; //If command send fails then bail

            await checkUblox();

            uint bitField = extractLong(4);
            imuMeas.rawDataType = (bitField & 0xFF000000) >> 23;
            imuMeas.rawData = (bitField & 0xFFFFFF);
            imuMeas.rawTStamp = extractLong(8);

            return true;
        }

        public async Task<SfeUbloxStatus> getSensState(byte sensor, ushort maxWait = 1100)
        {

            packetCfg.cls = Constants.UBX_CLASS_ESF;
            packetCfg.id = Constants.UBX_ESF_STATUS;
            packetCfg.len = 0;
            packetCfg.startingSpot = 0;

            if (await sendCommand(packetCfg, maxWait) != SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_RECEIVED)
                return (SfeUbloxStatus.SFE_UBLOX_STATUS_FAIL); //If command send fails then bail

            ubloxSen.numSens = extractByte(15);

            if (sensor > ubloxSen.numSens)
                return (SfeUbloxStatus.SFE_UBLOX_STATUS_OUT_OF_RANGE);

            await checkUblox();

            byte offset = 4;

            // Only the last sensor value checked will remain.
            for (byte i = 0; i < sensor; i++)
            {

                byte sensorFieldOne = extractByte((byte)(16 + offset * i));
                byte sensorFieldTwo = extractByte((byte)(17 + offset * i));
                ubloxSen.freq = extractByte((byte)(18 + offset * i));
                byte sensorFieldThr = extractByte((byte)(19 + offset * i));

                ubloxSen.senType = (byte)((sensorFieldOne & 0x10) >> 5);
                ubloxSen.isUsed = ((sensorFieldOne & 0x20) >> 6) > 0;
                ubloxSen.isReady = (sensorFieldOne & 0x30) >> 7 > 0;

                ubloxSen.calibStatus = (byte)(sensorFieldTwo & 0x03);
                ubloxSen.timeStatus = (byte)((sensorFieldTwo & 0xC) >> 2);

                ubloxSen.badMeas = ((sensorFieldThr & 0x01)) > 0;
                ubloxSen.badTag = ((sensorFieldThr & 0x02) >> 1) > 0;
                ubloxSen.missMeas = ((sensorFieldThr & 0x04) >> 2) > 0;
                ubloxSen.noisyMeas = ((sensorFieldThr & 0x08) >> 3) > 0;
            }

            return (SfeUbloxStatus.SFE_UBLOX_STATUS_SUCCESS);
        }

        public async Task<bool> getVehAtt(ushort maxWait = 1100)
        {

            packetCfg.cls = Constants.UBX_CLASS_NAV;
            packetCfg.id = Constants.UBX_NAV_ATT;
            packetCfg.len = 0;
            packetCfg.startingSpot = 0;

            if (await sendCommand(packetCfg, maxWait) != SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_RECEIVED)
                //return (sfe_ublox_status_e.SFE_UBLOX_STATUS_FAIL); //If command send fails then bail
                return false;

            await checkUblox();

            vehAtt.roll = (int)extractLong(8);
            vehAtt.pitch = (int)extractLong(12);
            vehAtt.heading = (int)extractLong(16);

            vehAtt.accRoll = extractLong(20);
            vehAtt.accPitch = extractLong(24);
            vehAtt.accHeading = extractLong(28);

            return true;
        }

        //Set the ECEF or Lat/Long coordinates of a receiver
        //This imediately puts the receiver in TIME mode (fixed) and will begin outputting RTCM sentences if enabled
        //This is helpful once an antenna's position has been established. See this tutorial: https://learn.sparkfun.com/tutorials/how-to-build-a-diy-gnss-reference-station#gather-raw-gnss-data
        // For ECEF the units are: cm, 0.1mm, cm, 0.1mm, cm, 0.1mm
        // For Lat/Lon/Alt the units are: degrees^-7, degrees^-9, degrees^-7, degrees^-9, cm, 0.1mm
        public async Task<bool> setStaticPosition(int ecefXOrLat, byte ecefXOrLatHP, int ecefYOrLon, byte ecefYOrLonHP, int ecefZOrAlt, byte ecefZOrAltHP, bool latLong, ushort maxWait = 250)
        {
            packetCfg.cls = Constants.UBX_CLASS_CFG;
            packetCfg.id = Constants.UBX_CFG_TMODE3;
            packetCfg.len = 0;
            packetCfg.startingSpot = 0;

            //Ask module for the current TimeMode3 settings. Loads into payloadCfg.
            if (await sendCommand(packetCfg, maxWait) != SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_RECEIVED)
                return false;

            packetCfg.len = 40;

            //Clear packet payload
            for (byte x = 0; x < packetCfg.len; x++)
                payloadCfg[x] = 0;

            //customCfg should be loaded with poll response. Now modify only the bits we care about
            payloadCfg[2] = 2; //Set mode to fixed. Use ECEF (not LAT/LON/ALT).

            if (latLong == true)
                payloadCfg[3] = (byte)(1 << 0); //Set mode to fixed. Use LAT/LON/ALT.

            //Set ECEF X or Lat
            payloadCfg[4] = (byte)((ecefXOrLat >> 8 * 0) & 0xFF); //LSB
            payloadCfg[5] = (byte)((ecefXOrLat >> 8 * 1) & 0xFF);
            payloadCfg[6] = (byte)((ecefXOrLat >> 8 * 2) & 0xFF);
            payloadCfg[7] = (byte)((ecefXOrLat >> 8 * 3) & 0xFF); //MSB

            //Set ECEF Y or Long
            payloadCfg[8] = (byte)((ecefYOrLon >> 8 * 0) & 0xFF); //LSB
            payloadCfg[9] = (byte)((ecefYOrLon >> 8 * 1) & 0xFF);
            payloadCfg[10] = (byte)((ecefYOrLon >> 8 * 2) & 0xFF);
            payloadCfg[11] = (byte)((ecefYOrLon >> 8 * 3) & 0xFF); //MSB

            //Set ECEF Z or Altitude
            payloadCfg[12] = (byte)((ecefZOrAlt >> 8 * 0) & 0xFF); //LSB
            payloadCfg[13] = (byte)((ecefZOrAlt >> 8 * 1) & 0xFF);
            payloadCfg[14] = (byte)((ecefZOrAlt >> 8 * 2) & 0xFF);
            payloadCfg[15] = (byte)((ecefZOrAlt >> 8 * 3) & 0xFF); //MSB

            //Set high precision parts
            payloadCfg[16] = ecefXOrLatHP;
            payloadCfg[17] = ecefYOrLonHP;
            payloadCfg[18] = ecefZOrAltHP;

            return await sendCommand(packetCfg, maxWait) == SfeUbloxStatus.SFE_UBLOX_STATUS_DATA_SENT; // We are only expecting an ACK
        }

        public async Task<bool> setStaticPosition(int ecefXOrLat, int ecefYOrLon, int ecefZOrAlt, bool latlong, ushort maxWait = 250)
        {
            return await setStaticPosition(ecefXOrLat, 0, ecefYOrLon, 0, ecefZOrAlt, 0, latlong, maxWait);
        }

        //Attach handler function to NMEA bytes received
        public void attachNMEAHandler(Func<char, Task> handler) => _nmeaHandler = handler;

        //Attach handler function to RTCM bytes received
        public void attachRTCMHandler(Func<byte, Task> handler) => _rtcmHandler = handler;

        //Write data into GPS device
        public void send(byte[] data, int len)
        {
            _serialPort.Write(data, 0, len);
        }
    }
}