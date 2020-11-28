/*

to publish application for linux/raspberry PI environment:
dotnet publish -c Release --self-contained -r linux-arm

Params
-port [ComPort]
-debug              enables debug output
-ntrip-server
-ntrip-port
-ntrip-mountpoint
-ntrip-password
-rtcm-accuracy-req          minimum accuracy to complete survey (float) in meters default 3.000F
-rtcm-survey-time           minimum time require to complete survey

To launch this on pi run the following
/home/pi/f9p/Zedf9p -server -com-port /dev/ttyACM0 -ntrip-server rtk2go.com -ntrip-password 6n9c2TxqKwuc -ntrip-port 2101 -ntrip-mountpoint Wexford -rtcm-accuracy-req 3.000 -rtcm-survey-time 60
/home/pi/f9p/Zedf9p -client -com-port /dev/ttyACM0 -ntrip-server rtk2go.com -ntrip-password 6n9c2TxqKwuc -ntrip-port 2101 -ntrip-mountpoint Wexford

 */

using System;
using System.IO;
using System.IO.Ports;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Zedf9p.Communication;
using Zedf9p.Enums;
using Zedf9p.Exceptions;
using Zedf9p.Models;

namespace Zedf9p.Core
{
    class Driver
    {
        const int RTCM_OUTGOING_BUFFER_SIZE = 500; //The size of the RTCM correction data varies but in general it is approximately 2000 bytes every second (~2500 bytes every 10th second when 1230 is transmitted).
        const int NTRIP_INCOMING_BUFFER_SIZE = 10000; //Size of incoming buffer for the connection with NTRIP Caster
        const int SYNC_INCOMING_BUFFER_SIZE = 10000; //Size of incoming buffer for the connection with NTRIP Caster
        const int NMEA_OUTGOING_STRING_MAXLEN = 300; //Size of incoming buffer for the connection with NTRIP Caster

        const int NAV_FREQUENCY = 20;                 //Sets how often GPS module will spit out data

        //parameters        
        bool _debug = false;     //debug flag

        
        string _nmeaDataSocketLocation = "/tmp/zed-f9p-nmea-data.sock"; //location of the interprocess socket(used to communicate with nodejs server)
        string _rtcmDataSocketLocation = "/tmp/zed-f9p-rtcm-data.sock"; //location of the interprocess socket(used to communicate with nodejs server)
        string _syncDataSocketLocation = "/tmp/zed-f9p-sync-data.sock"; //location of the interprocess socket(used to communicate with nodejs server)

        //Ublox f9p gps module
        UBLOX.SFE_UBLOX_GPS _myGPS;

        //Gps Module Settings       
        string _portName = "";           //port the module is connected to (COM#) or ttyACM#
        SerialPort _serialPort;           //instance of serial port for communication with module

        //NTRIP Caster settings
        Socket _ntripCasterSocket;   //socket for connection with NTRIP caster
        byte[] _ntripOutBuffer = new byte[RTCM_OUTGOING_BUFFER_SIZE];
        byte[] _ntripInBuffer = new byte[NTRIP_INCOMING_BUFFER_SIZE];
        int _rtcmFrame = 0;           //number of currect rtcm frame (for incoming rtcm data buffer control)
        int _ntripPort;              //caster port
        string _ntripServer;         //caster server        
        string _ntripMountpoint;     //caster mountpoint
        string _ntripPassword;       //caster password
        float _rtcmAccuracy;         //Minimum accuracy to be accepted before survey completes in meters (3.000F)/float
        int _rtcmSurveyTime;         //minimum time required for the survey to complete

        // f9p socket for interapplication communication with node express server
        AsyncSocket _nmeaDataSocket;
        AsyncSocket _rtcmDataSocket;
        AsyncSocket _syncDataSocket;

        //initialize buffer for sync channel
        byte[] _syncInBuffer = new byte[SYNC_INCOMING_BUFFER_SIZE];

        //initialize buffer for nmea channel
        string _nmeaOutBuffer = "";

        //Current operation mode
        OperationMode _mode;
        public void setMode(OperationMode mode) => _mode = mode;
        public OperationMode getMode() => _mode;





        /// <summary>
        /// Main constructor, takes in all the input params for the driver
        /// </summary>
        /// <param name="inputParams"></param>
        public Driver(InputParams inputParams)
        {
            _debug = inputParams.debug;
            _portName = inputParams.port;
            _ntripMountpoint = inputParams.ntripMountpoint;
            _ntripPassword = inputParams.ntripPassword;
            _ntripPort = inputParams.ntripPort;
            _ntripServer = inputParams.ntripServer;
            _rtcmAccuracy = inputParams.rtcmAccuracy;
            _rtcmSurveyTime = inputParams.rtcmSurveyTime;
        }

        /// <summary>
        /// Once module is configured for a specific operation mode this will keep it running
        /// </summary>
        /// <returns></returns>
        async public Task Run()
        {
            //Running server operations
            if (_mode == OperationMode.Server)
            {
                //Set USB output to UBX AND RTCM only, no NMEA Noise
                await _myGPS.setUSBOutput(UBLOX.Constants.COM_TYPE_UBX | UBLOX.Constants.COM_TYPE_RTCM3);

                //ATTACH NMEA HANDLER
                //_myGPS.attachNMEAHandler(processNmea_Server);

                //ATTACH RTCM HANDLER            
                _myGPS.attachRTCMHandler(processRtcm_Server);

                while (_myGPS != null)
                {
                    //if base station configures successfully start pulling data from it
                    await _myGPS.checkUblox(); //See if new data is available. Process bytes as they come in.

                    //Check sync socket periodically
                    await checkSyncDataSocket();

                    Thread.Sleep(10);
                }
            }
            // Running client operations
            else if (_mode == OperationMode.Client)
            {
                Console.WriteLine("attaching nmea handler...");

                //ATTACH NMEA HANDLER
                //_myGPS.attachNMEAHandler(processNmea_Client);        //this is not needed, GPS coords will be passed thru UBX        

                //increase nav message output frequency
                Console.WriteLine("Set nav frequency, " + (await _myGPS.setNavigationFrequency(NAV_FREQUENCY) ? "OK" : "Failed"));
                Console.WriteLine("Current update rate: " + await _myGPS.getNavigationFrequency());

                //configure for auto High-res messages
                //await _myGPS.setAutoHPPOSLLH(true);  //not sure what this even does now

                //Set USB output to UBX only, no NMEA Noise
                await _myGPS.setUSBOutput(UBLOX.Constants.COM_TYPE_UBX);

                //SEND RTCM to modyle when it becomes awailable
                new Task(() =>
                {
                    var lastRTCMdataReceived = Utils.millis();

                    while (_ntripCasterSocket != null && _syncDataSocket != null && _ntripCasterSocket.Connected && _syncDataSocket.isConnected())
                    {
                        if (_ntripCasterSocket.Available > 0)
                        {
                            var ntripRcvLen = _ntripCasterSocket.Receive(_ntripInBuffer);

                            if (ntripRcvLen > 0)
                            {
                                Console.WriteLine("Sending RTCM info to F9p module...");

                                _myGPS.send(_ntripInBuffer, ntripRcvLen);

                                lastRTCMdataReceived = Utils.millis();
                            }
                            else
                            {
                                //if no RTCM data received for longer than 10 minutes send error to UI
                                if (Utils.millis() > lastRTCMdataReceived + (60 * 10 * 1000))
                                {
                                    _syncDataSocket.SendLine("NTRIP_DATA_RCV_TIMEOUT");
                                }
                            }
                        }

                        Thread.Sleep(10);
                    }
                }).Start();


                //Output nav data to console every second
                while (_ntripCasterSocket != null && _syncDataSocket != null && _ntripCasterSocket.Connected && _syncDataSocket.isConnected())
                {

                    //GET HIGH RESOLUTION DATA
                    //int latitude = await _myGPS.getHighResLatitude();
                    double latitude = await _myGPS.getHighResLatitude() / 10000000D;
                    int latitudeHp = await _myGPS.getHighResLatitudeHp();
                    //int longitude = await _myGPS.getHighResLongitude();
                    double longitude = await _myGPS.getHighResLongitude() / 10000000D;
                    int longitudeHp = await _myGPS.getHighResLongitudeHp();
                    int ellipsoid = await _myGPS.getElipsoid();
                    int ellipsoidHp = await _myGPS.getElipsoidHp();
                    int msl = await _myGPS.getMeanSeaLevel();
                    int mslHp = await _myGPS.getMeanSeaLevelHp();
                    uint accuracy = await _myGPS.getHorizontalAccuracy();

                    double heading = await _myGPS.getHeading() / 100000D;

                    // Calculate the latitude and longitude integer and fractional parts
                    //var lat_int = latitude / 10000000; // Convert latitude from degrees * 10^-7 to Degrees
                    //var lat_frac = latitude - (lat_int * 10000000); // Calculate the fractional part of the latitude
                    //lat_frac = (lat_frac * 100) + latitudeHp; // Now add the high resolution component
                    //if (lat_frac < 0) // If the fractional part is negative, remove the minus sign
                    //{
                    //    lat_frac = 0 - lat_frac;
                    //}

                    //var lon_int = longitude / 10000000; // Convert latitude from degrees * 10^-7 to Degrees
                    //var lon_frac = longitude - (lon_int * 10000000); // Calculate the fractional part of the longitude
                    //lon_frac = (lon_frac * 100) + longitudeHp; // Now add the high resolution component
                    //if (lon_frac < 0) // If the fractional part is negative, remove the minus sign
                    //{
                    //    lon_frac = 0 - lon_frac;
                    //}

                    //Console.WriteLine("l:" + latitude + " lhp:" + latitudeHp + " hpp:" + lat_int + "." + lat_frac);

                    //var latitude = await _myGPS.getLatitude() / 10000000D;
                    //var longitude = await _myGPS.getLongitude() / 10000000D;
                    //var longitudeHR = await _myGPS.getHighResLongitude();
                    //var altitude = await _myGPS.getAltitudeMSL();
                    //var accuracy = await _myGPS.getPositionAccuracy();


                    //Console.Write("Latitude: " + latitude + " Longitude: " + longitude + " LongitudeHighRes: " + longitudeHR + " Altitude: " + altitude / 1000 + "m Accuracy: " + accuracy + "mm");

                    //_syncDataSocket.SendLine("LATITUDE:" + Double.Parse(lat_int + "." + lat_frac));
                    //_syncDataSocket.SendLine("LONGITUDE:" + Double.Parse(lon_int + "." + lon_frac));

                    _syncDataSocket.SendLine("LATITUDE:" + latitude);
                    _syncDataSocket.SendLine("LONGITUDE:" + longitude);
                    _syncDataSocket.SendLine("ACCURACY:" + accuracy);
                    _syncDataSocket.SendLine("HEADING:" + heading);

                    Thread.Sleep(1100/NAV_FREQUENCY);
                }
            }
            //Running server operations
            else if (_mode == OperationMode.Idle)
            {
                while (_myGPS != null)
                {
                    //if base station configures successfully start pulling data from it
                    await _myGPS.checkUblox(); //See if new data is available. Process bytes as they come in.

                    //Check sync socket periodically
                    await checkSyncDataSocket();

                    Thread.Sleep(10);
                }
            }
        }

        /// <summary>
        /// Destructof of class, basically disconnects all sockets
        /// </summary>
        /// <returns></returns>
        async public Task Cleanup()
        {
            // in the end, disconnect and close the socket to cleanup
            _rtcmDataSocket?.Disconnect();
            _nmeaDataSocket?.Disconnect();
            _syncDataSocket?.Disconnect();

            _ntripCasterSocket?.Disconnect(false);
            _ntripCasterSocket?.Close();
        }

        /// <summary>
        /// Initialize driver and connect sockets
        /// </summary>
        /// <param name="mode">Mode of operation</param>
        /// <returns></returns>
        async public Task Initialize(OperationMode mode)
        {
            //set current mode
            _mode = mode;

            //start interprocess communication
            connectDataSockets();
            
            if (_mode == OperationMode.Server)
            {
                //start NTRIP server (base station)
                await configureGpsAsNtripServer();
            }
            else if(_mode == OperationMode.Client)
            {
                //start NTRIP client (rover)
                await configureGpsAsNtripClient();
            }
        }

        //connect to f9p socket for interprocess communication
        void connectDataSockets()
        {
            try
            {
                Console.WriteLine("Connecting to interprocess sockets...");

                //Client socket for HTTP server running on Node                
                _syncDataSocket = new AsyncSocket(_syncDataSocketLocation).Connect();
                _nmeaDataSocket = new AsyncSocket(_nmeaDataSocketLocation).Connect();
                _rtcmDataSocket = new AsyncSocket(_rtcmDataSocketLocation).Connect();
            }
            catch (SocketException e)
            {
                //unable to connect to the socket, spit out error in a console
                Console.WriteLine("Socket connection error: " + e.Message);

                throw e;
            }
        }

        /// <summary>
        /// Configure f9p module as ntrip client (will consume data from NTRIP server and adjust coordinates)
        /// </summary>
        /// <returns></returns>
        async Task configureGpsAsNtripClient()
        {
            //open serial port
            _serialPort = new SerialPort(_portName, 115200, Parity.None, 8, StopBits.One);

            // Set the read/write timeouts
            _serialPort.ReadTimeout = 500;
            _serialPort.WriteTimeout = 500;

            Console.WriteLine("Trying to open port " + _portName);
            _serialPort.Open();

            //Console.WriteLine("Create serial port connection for f9p module...");
            _myGPS = new UBLOX.SFE_UBLOX_GPS();
            await _myGPS.begin(_serialPort);

            //setup for debugging(needs to be a different serial port)
            //_myGPS.enableDebugging(_serialPort, true);


            //Try reconnectin to NTRIP Caster 10 times every 10 seconds if connection fails 
            Utils.RetryHelper<NtripException>(() =>
            {
                //start sending data to ntrip caster
                ConnectNtripCasterSocket().GetAwaiter().GetResult();

                //send credentials to ntrip caster
                string ntripWelcomeMessage = "GET /" + _ntripMountpoint + " /HTTP/1.0\r\n";
                ntripWelcomeMessage += "User-Agent: NTRIP SOLVIT/1.0.1\r\n";
                //ntripWelcomeMessage += "Accept: */\r\n*";
                //ntripWelcomeMessage += "Connection: close \r\n";
                //ntripWelcomeMessage += "\r\n";

                Console.WriteLine("Trying to authenticate to NTRIP caster");



                //Send auth request
                _ntripCasterSocket.Send(Encoding.ASCII.GetBytes(ntripWelcomeMessage));

                //Receive respone with ICY 200 OK or ERROR - Bad Password
                var rcvLen = _ntripCasterSocket.Receive(_ntripInBuffer);
                var ntripResponseMessage = Encoding.ASCII.GetString(_ntripInBuffer, 0, rcvLen);

                if (ntripResponseMessage.Equals("ICY 200 OK\r\n"))
                {
                    Console.WriteLine("Authentication passed!");
                }
                else if (ntripResponseMessage.Contains("SOURCETABLE 200 OK\r\n"))
                {
                    throw new NtripException("Authentication passed. Sourcetable received but no RTCM data. base station is down?");
                }
                else
                {
                    //send error response back to sync socket
                    _syncDataSocket.SendLine("NTRIP_CONNECTION_ERROR");

                    Console.WriteLine("NTRIP Response: \"" + ntripResponseMessage.Trim() + "\"");
                    throw new NtripException("NTRIP Authentication error or station down");
                }

            }, 10, new TimeSpan(0, 0, 10), ()=>Console.WriteLine("trying to reconnect in 10 seconds..."));

        }

        /// <summary>
        /// Configure f9p module as ntrip server (will be sending out RTCM data to ntrip caster)
        /// </summary>
        /// <param name="rtcmAccuracy"></param>
        /// <param name="rtcmSurveyTime"></param>
        /// <returns></returns>
        async Task configureGpsAsNtripServer()
        {

            Console.WriteLine("Create serial port connection for f9p module...");

            //open serial port
            _serialPort = new SerialPort(_portName, 115200, Parity.None, 8, StopBits.One);

            // Set the read/write timeouts
            _serialPort.ReadTimeout = 500;
            _serialPort.WriteTimeout = 500;

            Console.WriteLine("Trying to open port " + _portName);
            _serialPort.Open();

            //Console.WriteLine("Create serial port connection for f9p module...");
            _myGPS = new UBLOX.SFE_UBLOX_GPS();
            await _myGPS.begin(_serialPort);

            //await startSurvey(_rtcmSurveyTime, _rtcmAccuracy);

            await startFixed(406028960, -800735223, 284000);


            //myGPS.setI2COutput(COM_TYPE_UBX | COM_TYPE_RTCM3); //Set the I2C port to output UBX and RTCM sentences (not really an option, turns on NMEA as well)

            //start sending data to ntrip caster
            await ConnectNtripCasterSocket();

            //send credentials to ntrip caster
            string ntripWelcomeMessage = "SOURCE " + _ntripPassword + " /" + _ntripMountpoint + "\r\n";
            ntripWelcomeMessage += string.Format("Source-Agent: %s/%s\r\n\r\n", "NTRIP NtripServerPOSIX", "$Revision: 9109 $");

            //Send auth request
            _ntripCasterSocket.Send(Encoding.ASCII.GetBytes(ntripWelcomeMessage));

            //Receive respone with ICY 200 OK or ERROR - Bad Password
            var rcvLen = _ntripCasterSocket.Receive(_ntripInBuffer);
            var ntripResponseMessage = Encoding.ASCII.GetString(_ntripInBuffer, 0, rcvLen); //take first 30 bytes from response 

            if (ntripResponseMessage.Equals("ICY 200 OK\r\n"))
            {
                Console.WriteLine("Authentication passed!");
            }
            else
            {
                Console.WriteLine("Authentiction error: " + ntripResponseMessage);
                throw new Exception("NTRIP Authentication error");
            }
        }

        /// <summary>
        /// Opens the connection to the NTRIP (creates a socket)
        /// </summary>
        /// <returns></returns>
        async Task ConnectNtripCasterSocket()
        {
            if (_ntripServer != null && _ntripMountpoint != null && _ntripPassword != null && _ntripPort != 0)
            {
                //parse host entry to get a IP address for connection
                var hostEntry = await Dns.GetHostEntryAsync(_ntripServer);

                //Connect to server
                _ntripCasterSocket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);

                Console.WriteLine("Trying to connect to NTRIP caster");

                _ntripCasterSocket.Connect(new IPEndPoint(hostEntry.AddressList[0], _ntripPort));
            }
            else
            {
                throw new Exception("NTRIP configuration or credentials not provided");
            }
        }

        //handler for NMEA data coming from the module when creating client (-client)
        async Task processNmea_Client(char incoming)
        {
            processNmea_Server(incoming);
        }

        /// <summary>
        /// handler for NMEA data coming from the module when creating server (-server)
        /// </summary>
        /// <param name="incoming">Incoming byte</param>
        /// <returns></returns>
        async Task processNmea_Server(char incoming)
        {
            _nmeaOutBuffer += incoming;

            //if (incoming == '\n') Console.WriteLine("NMEA NEWLINE RECEIVED");
            //if (_nmeaOutBuffer.Length >= NMEA_OUTGOING_STRING_MAXLEN) Console.WriteLine("NMEA MAXLEN ACHIEVED");


            if (incoming == '\n' || _nmeaOutBuffer.Length >= NMEA_OUTGOING_STRING_MAXLEN)
            {
                //send nmea info thru socket to the http server app
                if (_nmeaDataSocket.isConnected())
                {
                    _nmeaDataSocket.Send(_nmeaOutBuffer);
                }
                else
                {
                    Console.WriteLine("error: Nmea Socket not open...");
                }

                _nmeaOutBuffer = "";
            }
        }

        /// <summary>
        /// handler for RTCM data coming from the module when creating server (-server)
        /// </summary>
        /// <param name="incoming">Incoming byte</param>
        /// <returns></returns>
        async Task processRtcm_Server(byte incoming)
        {

            //send rtcm info thru socket to the http server app
            if (_rtcmDataSocket.isConnected())
            {
                _rtcmDataSocket.Send(incoming);
            }

            //push RTCM bute into the receive buffer until it reaches maximum defined size
            _ntripOutBuffer[_rtcmFrame++] = incoming;

            //once max buffer size is reached send rtcm data to the RTCM caster thru the socket via NTRIP protocol
            if (_rtcmFrame == RTCM_OUTGOING_BUFFER_SIZE)
            {
                _rtcmFrame = 0; //reset buffer

                //check if ntrip socket is defined and connected
                if (_ntripCasterSocket != null && _ntripCasterSocket.Connected)
                {
                    Console.WriteLine("Sending RTCM to NTRIP Caster");

                    _ntripCasterSocket.Send(_ntripOutBuffer);
                }
                else
                {
                    //check if socket was closed
                    Console.WriteLine("Ntrip Socket disconnected?");
                }
            }
        }

        /// <summary>
        /// Check incoming data thru the sync socket for commands coming from the frontend
        /// </summary>
        /// <returns></returns>
        async Task checkSyncDataSocket()
        {
            //check if socket is connected
            if (_syncDataSocket.isConnected())
            {
                //check if anything in available in a socket
                if (_syncDataSocket.Available > 0)
                {

                    var syncRcvLen = _syncDataSocket.Receive(_syncInBuffer);

                    if (syncRcvLen > 0)
                    {
                        var syncMessage = Encoding.ASCII.GetString(_syncInBuffer, 0, syncRcvLen);

                        Console.WriteLine("Sync Message Received: " + syncMessage);

                        try
                        {
                            var command = new SyncIncomingCommand(syncMessage);

                            await processSyncCommand(command);
                        }
                        catch (UnknownSyncCommandException e)
                        {
                            //spit out errors if anything dyring command parsing
                            Console.WriteLine(e.Message);
                        }
                    }
                    else
                    {
                        Console.WriteLine("Nothing received thru sync buffer");
                    }
                }
            } 
            else
            {
                throw new SyncSocketException("Sync Socket Disconnected, cannot continue, driver must shutdown...");
            }
        }

        async Task processSyncCommand(SyncIncomingCommand command) {

            if (SyncIncomingCommandType.RESTART_SURVEY == command.Type)
            {

                //Receive new accuracy from UI
                var newAccuracy = command.getValue<float>();
                var newTime = command.getValue<int>(1);

                //Disable receiver survey mode bewfore re-enabling with new settings
                if (await _myGPS.disableSurveyMode()) Console.WriteLine("Survey disabled");

                //Enable receiver with new settings
                await startSurvey(newTime, newAccuracy);

            }
            else if (SyncIncomingCommandType.RESTART_FIXED == command.Type)
            {

                //Receive new accuracy from UI
                var latitude = command.getValue<int>();
                var longitude = command.getValue<int>(1);
                var altitude = command.getValue<int>(2);

                //Disable receiver survey mode bewfore re-enabling with new settings
                if (await _myGPS.disableSurveyMode()) Console.WriteLine("Survey disabled");

                //Enable receiver with new settings
                await startFixed(latitude, longitude, altitude);

            }
            else if (SyncIncomingCommandType.START_SERVER == command.Type)
            {
                await configureGpsAsNtripServer();
            }
            else if (SyncIncomingCommandType.START_CLIENT == command.Type)
            {
                await configureGpsAsNtripClient();
            }
        }

        async Task startSurvey(int rtcmSurveyTime, float rtcmAccuracy) {

            if (_rtcmAccuracy == 0 || _rtcmSurveyTime == 0) throw new InvalidDataException("Survey accuracy or minimum time not provided, cannot continue...");

            Console.WriteLine("Enable Survey Mode, minimum " + rtcmSurveyTime + "sec and minimum accuracy " + rtcmAccuracy + " meters");

            var moduleResponse = await _myGPS.enableSurveyMode((ushort)rtcmSurveyTime, rtcmAccuracy); //Enable Survey in, 60 seconds, 5.0m

            Console.WriteLine("Enable RTCM Messaging on USB");

            moduleResponse &= await _myGPS.enableRTCMmessage(UBLOX.Constants.UBX_RTCM_1005, UBLOX.Constants.COM_PORT_USB, 1);
            moduleResponse &= await _myGPS.enableRTCMmessage(UBLOX.Constants.UBX_RTCM_1074, UBLOX.Constants.COM_PORT_USB, 1);
            moduleResponse &= await _myGPS.enableRTCMmessage(UBLOX.Constants.UBX_RTCM_1084, UBLOX.Constants.COM_PORT_USB, 1);
            moduleResponse &= await _myGPS.enableRTCMmessage(UBLOX.Constants.UBX_RTCM_1094, UBLOX.Constants.COM_PORT_USB, 1);
            moduleResponse &= await _myGPS.enableRTCMmessage(UBLOX.Constants.UBX_RTCM_1124, UBLOX.Constants.COM_PORT_USB, 1);
            moduleResponse &= await _myGPS.enableRTCMmessage(UBLOX.Constants.UBX_RTCM_1230, UBLOX.Constants.COM_PORT_USB, 10);

            if (!moduleResponse) throw new Exception("Unable to properly configure module");

            //Get receiver mode
            var surveyMode = await _myGPS.getSurveyMode();
            Console.WriteLine("Accuracy: " + surveyMode.getAccuracyLimit() + "m");
            _syncDataSocket.SendLine("SET_ACCURACY:" + surveyMode.getAccuracyLimit());
            Console.WriteLine("Mode: " + surveyMode.getMode().ToString());
            _syncDataSocket.SendLine("RECEIVER_MODE:" + surveyMode.getMode());


            Console.WriteLine("Start requestiong survey status...");

            //Begin waiting for survey to complete
            while (_myGPS.svin.valid == false)
            {

                //check sync data socket for updates
                await checkSyncDataSocket();

                //Query module for SVIN status with 2000ms timeout (req can take a long time)
                moduleResponse = await _myGPS.getSurveyStatus(2000);
                double latitude = await _myGPS.getHighResLatitude() / 10000000D;
                double longitude = await _myGPS.getHighResLongitude() / 10000000D;
                int altitude = await _myGPS.getAltitude();

                //if module response if always true
                if (moduleResponse)
                {
                    //send data back to UI
                    _syncDataSocket.SendLine("LATITUDE:" + latitude);
                    _syncDataSocket.SendLine("LONGITUDE:" + longitude);
                    _syncDataSocket.SendLine("ALTITUDE:" + altitude/1000); //altitude in meters
                    _syncDataSocket.SendLine("SURVEY_TIME:" + _myGPS.svin.observationTime.ToString());
                    _syncDataSocket.SendLine("ACCURACY:" + _myGPS.svin.meanAccuracy.ToString());
                    _syncDataSocket.SendLine("SURVEY_VALID:" + _myGPS.svin.valid.ToString());


                    //show output in a console
                    var output = "Time elapsed: " + _myGPS.svin.observationTime + "s";
                    output += " Accuracy: " + _myGPS.svin.meanAccuracy + "m";
                    output += " Is Valid?: " + _myGPS.svin.valid;
                    Console.WriteLine(output);
                }
                else
                {
                    Console.WriteLine("SVIN request failed");
                }

                await _myGPS.checkUblox(); //See if new data is available in COM PORT and consume it

                Thread.Sleep(100);
            }

            Console.WriteLine("Base survey complete! RTCM can now be broadcast");
        }

        async Task startFixed(int latitude, int longitude, int altitude) {

            if (latitude == 0 || longitude == 0 || altitude == 0) throw new InvalidDataException("Latitude, longitude or altitude not provided, cannot continue...");

            Console.WriteLine("Enable Fixed Mode, Lat: " + latitude + " Lon: " + longitude + " Alt: " + altitude);

            var moduleResponse = await _myGPS.setFixedMode(latitude, longitude, altitude); //Enable Survey in, 60 seconds, 5.0m

            Console.WriteLine("Enable RTCM Messaging on USB");

            moduleResponse &= await _myGPS.enableRTCMmessage(UBLOX.Constants.UBX_RTCM_1005, UBLOX.Constants.COM_PORT_USB, 1);
            moduleResponse &= await _myGPS.enableRTCMmessage(UBLOX.Constants.UBX_RTCM_1074, UBLOX.Constants.COM_PORT_USB, 1);
            moduleResponse &= await _myGPS.enableRTCMmessage(UBLOX.Constants.UBX_RTCM_1084, UBLOX.Constants.COM_PORT_USB, 1);
            moduleResponse &= await _myGPS.enableRTCMmessage(UBLOX.Constants.UBX_RTCM_1094, UBLOX.Constants.COM_PORT_USB, 1);
            moduleResponse &= await _myGPS.enableRTCMmessage(UBLOX.Constants.UBX_RTCM_1124, UBLOX.Constants.COM_PORT_USB, 1);
            moduleResponse &= await _myGPS.enableRTCMmessage(UBLOX.Constants.UBX_RTCM_1230, UBLOX.Constants.COM_PORT_USB, 10);

            if (!moduleResponse) throw new Exception("Unable to properly configure module");

            //Get receiver mode
            //var surveyMode = await _myGPS.getSurveyMode();
            //Console.WriteLine("Accuracy: " + surveyMode.getAccuracyLimit() + "m");
            //_syncDataSocket.SendLine("SET_ACCURACY:" + surveyMode.getAccuracyLimit());
            //Console.WriteLine("Mode: " + surveyMode.getMode().ToString());
            //_syncDataSocket.SendLine("RECEIVER_MODE:" + surveyMode.getMode());
            Console.WriteLine("Lat: " + await _myGPS.getLatitude());
            _syncDataSocket.SendLine("LATITUDE:" + await _myGPS.getLatitude());
            Console.WriteLine("Lon: " + await _myGPS.getLongitude());
            _syncDataSocket.SendLine("LONGITUDE:" + await _myGPS.getLongitude());


            Console.WriteLine("Base setting complete! RTCM can now be broadcast");
        }
    }
}
