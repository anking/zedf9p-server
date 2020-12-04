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
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;
using Zedf9p.Communication;
using Zedf9p.Enums;
using Zedf9p.Exceptions;
using Zedf9p.Models;

namespace Zedf9p.Core
{
    internal class Driver
    {
        const int RTCM_OUTGOING_BUFFER_SIZE = 500; //The size of the RTCM correction data varies but in general it is approximately 2000 bytes every second (~2500 bytes every 10th second when 1230 is transmitted).
        const int NTRIP_INCOMING_BUFFER_SIZE = 10000; //Size of incoming buffer for the connection with NTRIP Caster
        const int SYNC_INCOMING_BUFFER_SIZE = 10000; //Size of incoming buffer for the connection with NTRIP Caster
        const int NMEA_OUTGOING_STRING_MAXLEN = 300; //Size of incoming buffer for the connection with NTRIP Caster

        const int CLIENT_NAV_FREQUENCY = 20;                 //Sets how often GPS module will spit out data
        const int SERVER_NAV_FREQUENCY = 4;                 //Sets how often GPS module will spit out data

        const int NTRIP_RECEIVE_TIMEOUT = 30;                 //How many seconds we wait until we dicede there is an Ntrip receive timeout

        //TEST CONSTANTS
        const bool NTRIP_SERVER_SIMULATION = false;         //if set to true the driver will not connect to NTRIP
        const bool IGNORE_DISCONNECTED_SOCKET = false;          //true is a testvalue

        //parameters        
        private bool _debug = false;     //debug flag


        private const string NmeaDataSocketLocation = "/tmp/zed-f9p-nmea-data.sock"; //location of the interprocess socket(used to communicate with nodejs server)
        private const string RtcmDataSocketLocation = "/tmp/zed-f9p-rtcm-data.sock"; //location of the interprocess socket(used to communicate with nodejs server)
        private const string SyncDataSocketLocation = "/tmp/zed-f9p-sync-data.sock"; //location of the interprocess socket(used to communicate with nodejs server)

        //Ublox f9p gps module
        UBLOX.SFE_UBLOX_GPS _myGps;

        //Gps Module Settings       
        private readonly string _portName;           //port the module is connected to (COM#) or ttyACM#
        private SerialPort _serialPort;           //instance of serial port for communication with module

        //NTRIP Caster settings
        Socket _ntripCasterSocket;   //socket for connection with NTRIP caster
        readonly byte[] _ntripOutBuffer = new byte[RTCM_OUTGOING_BUFFER_SIZE];
        readonly byte[] _ntripInBuffer = new byte[NTRIP_INCOMING_BUFFER_SIZE];
        int _rtcmFrame = 0;           //number of currect rtcm frame (for incoming rtcm data buffer control)
        readonly int _ntripPort;              //caster port
        readonly string _ntripServer;         //caster server        
        readonly string _ntripMountpoint;     //caster mountpoint
        readonly string _ntripPassword;       //caster password

        //SurveyIn Mode vars
        float _surveyAccuracy;         //Minimum accuracy to be accepted before survey completes in meters (3.000F)/float
        int _surveyTime;         //minimum time required for the survey to complete

        //Fixed Mode vars
        int _latitude = 406028960;
        int _longitude = -800735223;
        int _altitude = 284000;

        // f9p socket for interapplication communication with node express server
        AsyncSocket _nmeaDataSocket;
        AsyncSocket _rtcmDataSocket;
        AsyncSocket _syncDataSocket;

        //initialize buffer for sync channel
        byte[] _syncInBuffer = new byte[SYNC_INCOMING_BUFFER_SIZE];

        //initialize buffer for nmea channel
        string _nmeaOutBuffer = "";

        //Current operation mode
        OperationMode _driverOperationMode;
        public void setMode(OperationMode mode) => _driverOperationMode = mode;
        public OperationMode getMode() => _driverOperationMode;

        //Current receiver mode (in server configuration)
        ReceiverMode _receiverMode = ReceiverMode.SurveyIn;

        //Reference to currently running main thread
        Task _mainRunningThread;

        //Cancellation token for main running thread
        CancellationTokenSource _cancellationTokenSource = new CancellationTokenSource();

        //Indicator of next NTRIP packet send status (cannot send them to often)
        long _nextNtripStatusSendTime;

        //Class that holds current state of errors
        private ErrorFlags _errorFlags;


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
            _surveyAccuracy = inputParams.rtcmAccuracy;
            _surveyTime = inputParams.rtcmSurveyTime;

            _errorFlags = new ErrorFlags(SendErrors);
        }

        /// <summary>
        /// Once module is configured for a specific operation mode this will keep it running
        /// </summary>
        /// <returns></returns>
        async Task Run()
        {
            Console.WriteLine("Starting 'Run' process as " + _driverOperationMode.ToString());

            //Clear any errors from setup
            _errorFlags.ClearErrors();

            //Running server operations
            if (_driverOperationMode == OperationMode.Server)
            {               
                //ATTACH NMEA HANDLER
                //_myGPS.attachNMEAHandler(processNmea_Server);

                //ATTACH RTCM HANDLER            
                //_myGPS.attachRTCMHandler(processRtcm_Server);

                if(await _myGps.getProtocolVersion()) Console.WriteLine("Ublox Protocol Version: " + await _myGps.getProtocolVersionHigh() + "." + await _myGps.getProtocolVersionLow());

                //var runThreadCancellationToken = RunThreads.Peek().tokenSource.Token;

                //Request receiver mode every 3 seconds
                var timer = new System.Timers.Timer(3000);
                timer.AutoReset = true; // the key is here so it repeats
                timer.Elapsed += async (sender, e) =>
                {
                    //Console.WriteLine("Timer run in thread " + Thread.CurrentThread.ManagedThreadId);

                    //Get receiver mode
                    //var tmode3 = await _myGPS.getTmode3();
                    //if (tmode3 != null)
                    //{
                    //    Console.WriteLine(tmode3.getMode());
                    //    _syncDataSocket.SendLine("RECEIVER_MODE:" + tmode3.getMode());
                    //}
                };
                timer.Start();

                //Receive rtcm updates and send them to proper channels
                while (_myGps != null)
                {
                    //if base station configures successfully start pulling data from it
                    await _myGps.checkUblox(); //See if new data is available. Process bytes as they come in.

                    Thread.Sleep(10);

                    if (_cancellationTokenSource.Token.IsCancellationRequested)
                    {
                        Console.WriteLine("Run thread " + Thread.CurrentThread.ManagedThreadId + " exiting...");
                        timer.Stop();
                        timer.Dispose();
                        break;
                    }
                }
            }
            // Running client operations
            else if (_driverOperationMode == OperationMode.Client)
            {
                //Console.WriteLine("attaching nmea handler...");

                //ATTACH NMEA HANDLER
                //_myGPS.attachNMEAHandler(processNmea_Client);        //this is not needed, GPS coords will be passed thru UBX        

                //increase nav message output frequency
                Console.WriteLine("Set nav frequency, " + (await _myGps.setNavigationFrequency(CLIENT_NAV_FREQUENCY) ? "OK" : "Failed"));
                Console.WriteLine("Current update rate: " + await _myGps.getNavigationFrequency());


                //Set USB output to UBX only, no NMEA Noise
                await _myGps.setUSBOutput(UBLOX.Constants.COM_TYPE_UBX);

                //SEND RTCM to module when it becomes available
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

                                _myGps.send(_ntripInBuffer, ntripRcvLen);

                                lastRTCMdataReceived = Utils.millis();
                            }

                            _errorFlags.Remove("NTRIP_DATA_RCV_TIMEOUT");
                        }

                        //if no RTCM data received for longer than 10 minutes send error to UI
                        if (Utils.millis() > lastRTCMdataReceived + NTRIP_RECEIVE_TIMEOUT * 1000)
                        {
                            _errorFlags.Add("NTRIP_DATA_RCV_TIMEOUT");
                        }

                        Thread.Sleep(10);
                    }

                    Console.WriteLine("Sockets needed for RTCM communication disconnected");
                }).Start();


                //Output nav data to console every nav cycle
                while (_syncDataSocket != null && _syncDataSocket.isConnected())
                {

                    //GET HIGH RESOLUTION DATA
                    double latitude = await _myGps.getHighResLatitude() / 10000000D;
                    int latitudeHp = await _myGps.getHighResLatitudeHp();
                    double longitude = await _myGps.getHighResLongitude() / 10000000D;
                    uint accuracy = await _myGps.getPositionAccuracy();
                    int altitude = await _myGps.getAltitude() / 1000;

                    double heading = await _myGps.getHeading() / 100000D;


                    _syncDataSocket.SendLine("LATITUDE:" + latitude);
                    _syncDataSocket.SendLine("LONGITUDE:" + longitude);
                    _syncDataSocket.SendLine("ALTITUDE:" + altitude);
                    _syncDataSocket.SendLine("ACCURACY:" + accuracy);
                    _syncDataSocket.SendLine("HEADING:" + heading);

                    Thread.Sleep(1100 / CLIENT_NAV_FREQUENCY);
                }

                Console.WriteLine("Driver Exited...");
            }
        }

        /// <summary>
        /// Send error flags to UI
        /// </summary>
        void SendErrors(ErrorFlags errorFlags)
        {
            if(_syncDataSocket.isConnected()) _syncDataSocket.SendLine("ERRORS:" + JsonSerializer.Serialize(errorFlags));
            else Console.WriteLine("Cannot send error output, sync socket is closed...");
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
            _driverOperationMode = mode;

            //start interprocess communication
            connectDataSockets();
            
            if (_driverOperationMode == OperationMode.Server)
            {
                //start NTRIP server (base station)
                _mainRunningThread = Task.Run(configureGpsAsNtripServer, _cancellationTokenSource.Token);
            }
            else if(_driverOperationMode == OperationMode.Client)
            {
                //start NTRIP client (rover)
                _mainRunningThread = Task.Run(configureGpsAsNtripClient, _cancellationTokenSource.Token);
            }

            await maintainRunningThreads();
        }

        async Task maintainRunningThreads()
        {
            while (true)
            {
                //Check sync socket periodically
                await checkSyncDataSocket();

                Thread.Sleep(100);
            }
        }

        //connect to f9p socket for interprocess communication
        void connectDataSockets()
        {
            try
            {
                Console.WriteLine("Connecting to interprocess sockets...");

                //Client socket for HTTP server running on Node                
                _syncDataSocket = new AsyncSocket(SyncDataSocketLocation).Connect();
                _nmeaDataSocket = new AsyncSocket(NmeaDataSocketLocation).Connect();
                _rtcmDataSocket = new AsyncSocket(RtcmDataSocketLocation).Connect();
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
            _myGps = new UBLOX.SFE_UBLOX_GPS();
            await _myGps.begin(_serialPort);

            //setup for debugging(needs to be a different serial port)
            //_myGPS.enableDebugging(_serialPort, true);

            //Try reconnecting to NTRIP Caster 10 times every 10 seconds if connection fails 
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

                if (ntripResponseMessage.Contains("ICY 200 OK\r\n"))
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
                    _errorFlags.Add("NTRIP_CONNECTION_ERROR");

                    Console.WriteLine("NTRIP Response: \"" + ntripResponseMessage.Trim() + "\"");
                    throw new NtripException("NTRIP Authentication error or station down");
                }

            }, 10, new TimeSpan(0, 0, 10), ()=>Console.WriteLine("trying to reconnect in 10 seconds..."));

            await Run();

        }

        /// <summary>
        /// Configure f9p module as ntrip server (will be sending out RTCM data to ntrip caster)
        /// </summary>
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
            _myGps = new UBLOX.SFE_UBLOX_GPS();
            await _myGps.begin(_serialPort);

            //Set USB output to UBX AND RTCM only, no NMEA Noise
            await _myGps.setUSBOutput(UBLOX.Constants.COM_TYPE_UBX | UBLOX.Constants.COM_TYPE_RTCM3);

            //engage proper receiver mode
            switch (_receiverMode)
            {
                case ReceiverMode.SurveyIn: await startSurvey(_surveyTime, _surveyAccuracy); break;
                case ReceiverMode.Fixed: await startFixed(_latitude, _longitude, _altitude); break;
            }           

            //connect to to ntrip caster
            await ConnectNtripCasterSocket();

            {
                //AUTHENTICATE ON NTRIP AS SERVER
                if (!NTRIP_SERVER_SIMULATION)
                {

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
            }

            await Run();

        }

        /// <summary>
        /// Opens the connection to the NTRIP (creates a socket)
        /// </summary>
        /// <returns></returns>
        async Task ConnectNtripCasterSocket()
        {
            if (NTRIP_SERVER_SIMULATION) return;

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

            //send rtcm info thru socket to the UI http server app
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

                if (NTRIP_SERVER_SIMULATION) {
                    if (Utils.millis() > _nextNtripStatusSendTime)
                    {
                        _syncDataSocket.SendLine("NTRIP_SENT:");
                        _nextNtripStatusSendTime = Utils.millis() + 1000;
                    }
                    Console.WriteLine("Sending RTCM to NTRIP Server"); 
                    Thread.Sleep(250); return; 
                }

                //check if ntrip socket is defined and connected
                if (_ntripCasterSocket != null && _ntripCasterSocket.Connected)
                {
                    Console.WriteLine("Sending RTCM to NTRIP Caster");

                    //Do not send this more than once a second to sync
                    if (Utils.millis() > _nextNtripStatusSendTime) {
                        _syncDataSocket.SendLine("NTRIP_SENT:");
                        _nextNtripStatusSendTime = Utils.millis() + 1000;
                    }

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
                if(!IGNORE_DISCONNECTED_SOCKET)
                throw new SyncSocketException("Sync Socket Disconnected, cannot continue, driver must shutdown...");
            }
        }

        async Task processSyncCommand(SyncIncomingCommand command) {

            if (SyncIncomingCommandType.RESTART_SURVEY == command.Type)
            {
                //Receive new accuracy from UI
                _surveyAccuracy = command.getValue<float>();
                _surveyTime = command.getValue<int>(1);

                //Disable receiver survey mode bewfore re-enabling with new settings
                if (await _myGps.disableReceiver()) Console.WriteLine("Receiver disabled");

                //stop main running thread
                _cancellationTokenSource.Cancel();

                //wait for running thread to stop
                _mainRunningThread.Wait();                

                //set current mode
                _receiverMode = ReceiverMode.SurveyIn;

                //Reset cancellation token
                _cancellationTokenSource = new CancellationTokenSource();

                //enable receiver with new settings
                _mainRunningThread = Task.Run(configureGpsAsNtripServer, _cancellationTokenSource.Token);

            }
            else if (SyncIncomingCommandType.RESTART_FIXED == command.Type)
            {

                //Receive new accuracy from UI
                _latitude = command.getValue<int>();
                _longitude = command.getValue<int>(1);
                _altitude = command.getValue<int>(2);

                //Disable receiver survey mode bewfore re-enabling with new settings
                if (await _myGps.disableReceiver()) Console.WriteLine("Receiver disabled");

                //stop main running thread
                _cancellationTokenSource.Cancel();

                //wait for running thread to stop
                _mainRunningThread.Wait();

                //set current mode
                _receiverMode = ReceiverMode.Fixed;

                //Reset cancellation token
                _cancellationTokenSource = new CancellationTokenSource();

                //enable receiver with new settings
                _mainRunningThread = Task.Run(configureGpsAsNtripServer, _cancellationTokenSource.Token);

            }
        }

        /// <summary>
        /// Enables Survey mode on f9p device
        /// </summary>
        /// <param name="rtcmSurveyTime"></param>
        /// <param name="rtcmAccuracy"></param>
        /// <returns></returns>
        async Task startSurvey(int rtcmSurveyTime, float rtcmAccuracy) {

            if (_surveyAccuracy == 0 || _surveyTime == 0) throw new InvalidDataException("Survey accuracy or minimum time not provided, cannot continue...");

            Console.WriteLine("Enable Survey Mode, minimum " + rtcmSurveyTime + "sec and minimum accuracy " + rtcmAccuracy + " meters");

            var moduleResponse = await _myGps.enableSurveyMode((ushort)rtcmSurveyTime, rtcmAccuracy); //Enable Survey in, 60 seconds, 5.0m

            if (!moduleResponse) throw new Exception("Unable to properly configure module");

            await disableRtcmMessagesOnUSB();
            //_myGPS.attachRTCMHandler(null);

            //Get receiver mode
            var surveyMode = await _myGps.getTmode3();
            Console.WriteLine("Accuracy: " + surveyMode.GetAccuracyLimit() + "m");
            _syncDataSocket.SendLine("SET_ACCURACY:" + surveyMode.GetAccuracyLimit());
            Console.WriteLine("Mode: " + surveyMode.GetMode().ToString());
            _syncDataSocket.SendLine("RECEIVER_MODE:" + surveyMode.GetMode());

            //Console.WriteLine("Position mode: " + surveyMode.GetPositionMode());
            //Console.WriteLine("Lat: " + surveyMode.GetLatitude());
            //Console.WriteLine("Lon: " + surveyMode.GetPositionMode());
            //Console.WriteLine("Alt: " + surveyMode.GetPositionMode());
            //Console.WriteLine("Accuracy: " + surveyMode.GetPositionMode());


            //set ubx message output configuration
            //await _myGPS.configureMessage(UBLOX.Constants.UBX_CLASS_NAV, UBLOX.Constants.UBX_NAV_PVT, UBLOX.Constants.COM_PORT_USB, 1);
            //await _myGPS.configureMessage(UBLOX.Constants.UBX_CLASS_NAV, UBLOX.Constants.UBX_NAV_HPPOSLLH, UBLOX.Constants.COM_PORT_USB, 1);

            Console.WriteLine("Start requestiong survey status...");

            //get current survey status
            await _myGps.getSurveyStatus(2000);

            //set proper navigation frequency for base
            await _myGps.setNavigationFrequency(4);

            //Begin waiting for survey to complete
            while (_myGps.svin.valid == false)
            {
                //exit if cancellation requested
                if (_cancellationTokenSource.Token.IsCancellationRequested) return;

                //check sync data socket for updates
                await checkSyncDataSocket();

                //Query module for SVIN status with 2000ms timeout (req can take a long time)
                moduleResponse = await _myGps.getSurveyStatus(2000);
                double latitude = await _myGps.getHighResLatitude() / 10000000D;
                double longitude = await _myGps.getHighResLongitude() / 10000000D;
                int altitude = await _myGps.getAltitude() / 1000;

                //if module response if always true
                if (moduleResponse)
                {
                    //send data back to UI
                    _syncDataSocket.SendLine("LATITUDE:" + latitude);
                    _syncDataSocket.SendLine("LONGITUDE:" + longitude);
                    _syncDataSocket.SendLine("ALTITUDE:" + altitude); //altitude in meters
                    _syncDataSocket.SendLine("ACCURACY:" + _myGps.svin.meanAccuracy.ToString());
                    _syncDataSocket.SendLine("SURVEY_TIME:" + _myGps.svin.observationTime.ToString());
                    _syncDataSocket.SendLine("SURVEY_VALID:" + _myGps.svin.valid.ToString());


                    //show output in a console
                    var output = "Time elapsed: " + _myGps.svin.observationTime + "s";
                    output += " Accuracy: " + _myGps.svin.meanAccuracy + "m";
                    output += " Is Valid?: " + _myGps.svin.valid;
                    Console.WriteLine(output);
                }
                else
                {
                    Console.WriteLine("SVIN request failed");
                }

                await _myGps.checkUblox(); //See if new data is available in COM PORT and consume it

                Thread.Sleep(100);
            }

            //Send location data again in case survey has started as valid already
            {
                //send data back to UI
                double latitude = await _myGps.getHighResLatitude() / 10000000D;
                double longitude = await _myGps.getHighResLongitude() / 10000000D;
                int altitude = await _myGps.getAltitude() / 1000;
                uint accuracy = await _myGps.getPositionAccuracy() / 1000;
                _syncDataSocket.SendLine("LATITUDE:" + latitude);
                _syncDataSocket.SendLine("LONGITUDE:" + longitude);
                _syncDataSocket.SendLine("ALTITUDE:" + altitude); //altitude in meters
                _syncDataSocket.SendLine("ACCURACY:" + accuracy);

                Console.WriteLine("Lat: " + latitude);
                Console.WriteLine("Lon: " + longitude);
                Console.WriteLine("Alt: " + altitude);
                Console.WriteLine("Accuracy: " + accuracy);
            }

            Console.WriteLine("Base survey complete! RTCM can now be broadcast");

            await enableRtcmMessagesOnUSB();
            //_myGPS.attachRTCMHandler(processRtcm_Server);
        }

        /// <summary>
        /// Enables Fixed mode on f9p device
        /// </summary>
        /// <param name="latitude"></param>
        /// <param name="longitude"></param>
        /// <param name="altitude"></param>
        /// <returns></returns>
        async Task startFixed(int latitude, int longitude, int altitude) {

            if (latitude == 0 || longitude == 0 || altitude == 0) throw new InvalidDataException("Latitude, longitude or altitude not provided, cannot continue...");

            await disableRtcmMessagesOnUSB();
            //_myGPS.attachRTCMHandler(null);

            Console.WriteLine("Enable Fixed Mode, Lat: " + latitude + " Lon: " + longitude + " Alt: " + altitude);

            var moduleResponse = await _myGps.enableFixedMode(latitude, longitude, altitude); //Enable Survey in, 60 seconds, 5.0m  

            var surveyMode = await _myGps.getTmode3();
            _syncDataSocket.SendLine("RECEIVER_MODE:" + surveyMode.GetMode());

            if (!moduleResponse) throw new Exception("Unable to properly configure module");

            //_syncDataSocket.SendLine("LATITUDE:" + await _myGPS.getHighResLatitude() / 10000000D);
            //_syncDataSocket.SendLine("LONGITUDE:" + await _myGPS.getHighResLongitude() / 10000000D);
            //_syncDataSocket.SendLine("ALTITUDE:" + await _myGPS.getAltitude() / 1000);

            _syncDataSocket.SendLine("LATITUDE:" + latitude / 10000000D);
            _syncDataSocket.SendLine("LONGITUDE:" + longitude / 10000000D);
            _syncDataSocket.SendLine("ALTITUDE:" + altitude / 1000);

            Console.WriteLine("Base setting complete! RTCM can now be broadcast");

            await enableRtcmMessagesOnUSB();
            //_myGPS.attachRTCMHandler(processRtcm_Server);
        }

        async Task enableRtcmMessagesOnUSB()
        {

            Console.WriteLine("Enable RTCM Messaging on USB");

            var moduleResponse = await _myGps.enableRTCMmessage(UBLOX.Constants.UBX_RTCM_1005, UBLOX.Constants.COM_PORT_USB, 1);
            moduleResponse &= await _myGps.enableRTCMmessage(UBLOX.Constants.UBX_RTCM_1074, UBLOX.Constants.COM_PORT_USB, 1);
            moduleResponse &= await _myGps.enableRTCMmessage(UBLOX.Constants.UBX_RTCM_1084, UBLOX.Constants.COM_PORT_USB, 1);
            moduleResponse &= await _myGps.enableRTCMmessage(UBLOX.Constants.UBX_RTCM_1094, UBLOX.Constants.COM_PORT_USB, 1);
            moduleResponse &= await _myGps.enableRTCMmessage(UBLOX.Constants.UBX_RTCM_1124, UBLOX.Constants.COM_PORT_USB, 1);
            moduleResponse &= await _myGps.enableRTCMmessage(UBLOX.Constants.UBX_RTCM_1230, UBLOX.Constants.COM_PORT_USB, 10);

            if (!moduleResponse) throw new Exception("Unable to properly configure module");

            _myGps.attachRTCMHandler(processRtcm_Server);
        }

        async Task disableRtcmMessagesOnUSB()
        {

            Console.WriteLine("Disable RTCM Messaging on USB");

            var moduleResponse = await _myGps.disableRTCMmessage(UBLOX.Constants.UBX_RTCM_1005, UBLOX.Constants.COM_PORT_USB);
            moduleResponse &= await _myGps.disableRTCMmessage(UBLOX.Constants.UBX_RTCM_1074, UBLOX.Constants.COM_PORT_USB);
            moduleResponse &= await _myGps.disableRTCMmessage(UBLOX.Constants.UBX_RTCM_1084, UBLOX.Constants.COM_PORT_USB);
            moduleResponse &= await _myGps.disableRTCMmessage(UBLOX.Constants.UBX_RTCM_1094, UBLOX.Constants.COM_PORT_USB);
            moduleResponse &= await _myGps.disableRTCMmessage(UBLOX.Constants.UBX_RTCM_1124, UBLOX.Constants.COM_PORT_USB);
            moduleResponse &= await _myGps.disableRTCMmessage(UBLOX.Constants.UBX_RTCM_1230, UBLOX.Constants.COM_PORT_USB);

            if (!moduleResponse) throw new Exception("Unable to properly configure module");

            _myGps.attachRTCMHandler(null);
        }
    }
}
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     