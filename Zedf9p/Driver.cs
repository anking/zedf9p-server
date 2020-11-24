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
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using UBLOX;
using Zedf9p.Communication;

namespace Zedf9p.Core
{
    class Driver
    {
        const int RTCM_BUFFER_SIZE = 500; //The size of the RTCM correction data varies but in general it is approximately 2000 bytes every second (~2500 bytes every 10th second when 1230 is transmitted).
        const int NTRIP_INCOMING_BUFFER_SIZE = 10000; //Size of incoming buffer for the connection with NTRIP Caster
        const int SYNC_INCOMING_BUFFER_SIZE = 10000; //Size of incoming buffer for the connection with NTRIP Caster

        //parameters        
        bool _debug = false;     //debug flag

        public enum OperationMode
        {
            Undefined = 0,
            Server = 1,         //if set to true program will configure gps as NTRIP server
            Client = 2         //if set to true program will configure gps as NTRIP client            
        }
        string _nmeaDataSocketLocation = "/tmp/zed-f9p-nmea-data.sock"; //location of the interprocess socket(used to communicate with nodejs server)
        string _rtcmDataSocketLocation = "/tmp/zed-f9p-rtcm-data.sock"; //location of the interprocess socket(used to communicate with nodejs server)
        string _syncDataSocketLocation = "/tmp/zed-f9p-sync-data.sock"; //location of the interprocess socket(used to communicate with nodejs server)

        //Ublox f9p gps module
        SFE_UBLOX_GPS _myGPS;

        //Gps Module Settings       
        string _port = "";           //port the module is connected to (COM#) or ttyACM#

        //NTRIP Caster settings
        Socket _ntripCasterSocket;   //socket for connection with NTRIP caster
        byte[] ntripOutgoingBuffer = new byte[RTCM_BUFFER_SIZE];
        byte[] ntripIncomingBuffer = new byte[NTRIP_INCOMING_BUFFER_SIZE];
        int rtcmFrame = 0;           //number of currect rtcm frame (for incoming rtcm data buffer control)
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
        byte[] syncIncomingBuffer = new byte[SYNC_INCOMING_BUFFER_SIZE];

        //Current operation mode
        OperationMode _mode;





        /// <summary>
        /// Main constructor, takes in all the input params for the driver
        /// </summary>
        /// <param name="inputParams"></param>
        public Driver(InputParams inputParams)
        {
            _debug = inputParams.debug;
            _port = inputParams.port;
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
                var lastRTCMdataReceived = Utils.millis();

                //configure for auto High-res messages
                await _myGPS.setAutoHPPOSLLH(true);

                while (_ntripCasterSocket != null && _syncDataSocket != null && _ntripCasterSocket.Connected && _syncDataSocket.isConnected())
                {
                    var ntripRcvLen = _ntripCasterSocket.Receive(ntripIncomingBuffer);

                    if (ntripRcvLen > 0)
                    {
                        await _myGPS.getPositionAccuracy();

                        

                        Console.WriteLine("Sending RTCM info to module. Last Accuracy: " + _myGPS.horizontalAccuracy + " / " + await _myGPS.getPositionAccuracy());

                        _myGPS.send(ntripIncomingBuffer, ntripRcvLen);

                        //if base station configures successfully start pulling data from it
                        await _myGPS.checkUblox(); //See if new data is available. Process bytes as they come in.

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
                await configureGpsAsNtripServer(_rtcmAccuracy, _rtcmSurveyTime);
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
            catch (SocketException ese)
            {
                //unable to connect to the socket, spit out error in a console
                Console.WriteLine("Socket connection error: " + ese.Message);
            }
        }

        /// <summary>
        /// Configure f9p module as ntrip client (will consume data from NTRIP server and adjust coordinates)
        /// </summary>
        /// <returns></returns>
        async Task configureGpsAsNtripClient()
        {
            //Console.WriteLine("Create serial port connection for f9p module...");
            _myGPS = new SFE_UBLOX_GPS(_port, _debug);

            //start sending data to ntrip caster
            await ConnectNtripCaster();

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
            var rcvLen = _ntripCasterSocket.Receive(ntripIncomingBuffer);
            var ntripResponseMessage = Encoding.ASCII.GetString(ntripIncomingBuffer, 0, rcvLen);

            if (ntripResponseMessage.Equals("ICY 200 OK\r\n"))
            {
                Console.WriteLine("Authentication passed!");
            }
            else
            {
                //send error response back to sync socket
                _syncDataSocket.SendLine("NTRIP_CONNECTION_ERROR");

                Console.WriteLine("NTRIP Response: " + ntripResponseMessage);                
                throw new Exception("NTRIP Authentication error or station down");                
            }

            //ATTACH NMEA HANDLER
            _myGPS.attachNMEAHandler(processNmea_Client);
        }

        /// <summary>
        /// Configure f9p module as ntrip server (will be sending out RTCM data to ntrip caster)
        /// </summary>
        /// <param name="rtcmAccuracy"></param>
        /// <param name="rtcmSurveyTime"></param>
        /// <returns></returns>
        async Task configureGpsAsNtripServer(float rtcmAccuracy, int rtcmSurveyTime)
        {
            if (rtcmAccuracy == 0 || rtcmSurveyTime == 0) throw new InvalidDataException("Survey accuracy or minimum time not provided, cannot continue...");

            Console.WriteLine("Create serial port connection for f9p module...");
            _myGPS = new SFE_UBLOX_GPS(_port, _debug);

            Console.WriteLine("Enable Survey Mode, minimum " + rtcmSurveyTime + "sec and minimum accuracy " + rtcmAccuracy + " meters");

            var moduleResponse = await _myGPS.enableSurveyMode((ushort)rtcmSurveyTime, rtcmAccuracy); //Enable Survey in, 60 seconds, 5.0m

            Console.WriteLine("Enable RTCM Messaging on USB");

            moduleResponse &= await _myGPS.enableRTCMmessage(SFE_UBLOX_GPS.UBX_RTCM_1005, SFE_UBLOX_GPS.COM_PORT_USB, 1);
            moduleResponse &= await _myGPS.enableRTCMmessage(SFE_UBLOX_GPS.UBX_RTCM_1074, SFE_UBLOX_GPS.COM_PORT_USB, 1);
            moduleResponse &= await _myGPS.enableRTCMmessage(SFE_UBLOX_GPS.UBX_RTCM_1084, SFE_UBLOX_GPS.COM_PORT_USB, 1);
            moduleResponse &= await _myGPS.enableRTCMmessage(SFE_UBLOX_GPS.UBX_RTCM_1094, SFE_UBLOX_GPS.COM_PORT_USB, 1);
            moduleResponse &= await _myGPS.enableRTCMmessage(SFE_UBLOX_GPS.UBX_RTCM_1124, SFE_UBLOX_GPS.COM_PORT_USB, 1);
            moduleResponse &= await _myGPS.enableRTCMmessage(SFE_UBLOX_GPS.UBX_RTCM_1230, SFE_UBLOX_GPS.COM_PORT_USB, 10);

            if (!moduleResponse) throw new Exception("Unable to properly configure module");

            //ATTACH NMEA HANDLER
            _myGPS.attachNMEAHandler(processNmea_Server);

            //Get receiver mode
            var receiverMode = await _myGPS.getReceiverMode();
            Console.WriteLine("Accuracy: " + receiverMode.getAccuracyLimit() + "m");
            _syncDataSocket.SendLine("SET_ACCURACY:" + receiverMode.getAccuracyLimit());
            Console.WriteLine("Mode: " + receiverMode.getMode().ToString());
            _syncDataSocket.SendLine("RECEIVER_MODE:" + receiverMode.getMode());


            Console.WriteLine("Start requestiong survey status...");

            //Begin waiting for survey to complete
            while (_myGPS.svin.valid == false)
            {

                //check sync data socket for updates
                await checkSyncDataSocket();

                //Query module for SVIN status with 2000ms timeout (req can take a long time)
                moduleResponse = await _myGPS.getSurveyStatus(2000);

                //if module response if always true
                if (moduleResponse)
                {
                    //send data back to UI
                    _syncDataSocket.SendLine("SURVEY_TIME:" + _myGPS.svin.observationTime.ToString());
                    _syncDataSocket.SendLine("ACCURACY:" + _myGPS.svin.meanAccuracy.ToString());
                    _syncDataSocket.SendLine("SURVEY_VALID:" + _myGPS.svin.valid.ToString());

                    //show output in a console
                    var output = "Time elapsed: " + _myGPS.svin.observationTime;
                    output += " Accuracy: " + _myGPS.svin.meanAccuracy;
                    output += " Is Valid?: " + _myGPS.svin.valid;
                    Console.WriteLine(output);
                }
                else
                {
                    Console.WriteLine("SVIN request failed");
                }

                await _myGPS.checkUblox(); //See if new data is available in COM PORT and consume it

                Thread.Sleep(1000);
            }

            Console.WriteLine("Base survey complete! RTCM can now be broadcast");

            //myGPS.setI2COutput(COM_TYPE_UBX | COM_TYPE_RTCM3); //Set the I2C port to output UBX and RTCM sentences (not really an option, turns on NMEA as well)


            //ATTACH RTCM HANDLER            
            _myGPS.attachRTCMHandler(processRtcm_Server);

            //start sending data to ntrip caster
            await ConnectNtripCaster();

            //send credentials to ntrip caster
            string ntripWelcomeMessage = "SOURCE " + _ntripPassword + " /" + _ntripMountpoint + "\r\n";
            ntripWelcomeMessage += string.Format("Source-Agent: %s/%s\r\n\r\n", "NTRIP NtripServerPOSIX", "$Revision: 9109 $");

            //Send auth request
            _ntripCasterSocket.Send(Encoding.ASCII.GetBytes(ntripWelcomeMessage));

            //Receive respone with ICY 200 OK or ERROR - Bad Password
            var rcvLen = _ntripCasterSocket.Receive(ntripIncomingBuffer);
            var ntripResponseMessage = Encoding.ASCII.GetString(ntripIncomingBuffer, 0, rcvLen); //take first 30 bytes from response 

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
        async Task ConnectNtripCaster()
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
        async Task processNmea_Client(byte incoming)
        {
            //send nmea info thru socket to the http server app
            if (_nmeaDataSocket.isConnected())
            {
                _nmeaDataSocket.Send(incoming);
            }
            else
            {
                Console.WriteLine("error: Nmea Socket not open...");
            }
        }

        /// <summary>
        /// handler for NMEA data coming from the module when creating server (-server)
        /// </summary>
        /// <param name="incoming">Incoming byte</param>
        /// <returns></returns>
        async Task processNmea_Server(byte incoming)
        {
            //send nmea info thru socket to the http server app
            if (_nmeaDataSocket.isConnected())
            {
                _nmeaDataSocket.Send(incoming);
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
            ntripOutgoingBuffer[rtcmFrame++] = incoming;

            //once max buffer size is reached send rtcm data to the RTCM caster thru the socket via NTRIP protocol
            if (rtcmFrame == RTCM_BUFFER_SIZE)
            {
                rtcmFrame = 0; //reset buffer

                //check if ntrip socket is defined and connected
                if (_ntripCasterSocket != null && _ntripCasterSocket.Connected)
                {
                    Console.WriteLine("Sending RTCM");

                    _ntripCasterSocket.Send(ntripOutgoingBuffer);
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
            if (_syncDataSocket.isConnected() && _syncDataSocket.Available > 0)
            {
                var syncRcvLen = _syncDataSocket.Receive(syncIncomingBuffer);

                if (syncRcvLen > 0)
                {
                    var syncMessage = Encoding.ASCII.GetString(syncIncomingBuffer, 0, syncRcvLen);

                    Console.WriteLine("Sync Message Received: " + syncMessage);

                    if (syncMessage.Contains("RESTART_SURVEY"))
                    {
                        //Receive new accuracy from UI
                        var newAccuracy = float.Parse(syncMessage.Split(':')[1]);

                        //Disable receiver survey mode bewfore re-enabling with new settings
                        if (await _myGPS.disableSurveyMode()) Console.WriteLine("Survey disabled");

                        //Enable receiver with new settings
                        if (await _myGPS.enableSurveyMode((ushort)_rtcmSurveyTime, newAccuracy)) Console.WriteLine("Enabling survey, new accuracy = " + newAccuracy + "...");

                        //Get new receiver mode
                        var receiverMode = await _myGPS.getReceiverMode();

                        //Send new settings back into UI
                        Console.WriteLine("Accuracy: " + receiverMode.getAccuracyLimit());
                        _syncDataSocket.SendLine("SET_ACCURACY:" + receiverMode.getAccuracyLimit());

                        //Send new settings back into UI
                        Console.WriteLine("Mode: " + receiverMode.getMode());
                        _syncDataSocket.SendLine("RECEIVER_MODE:" + receiverMode.getMode());
                    }
                    else
                    {
                        Console.WriteLine("Unknown message");
                    }
                }
                else
                {
                    Console.WriteLine("Nothing received thru sync buffer");
                }
            }
        }
    }
}
