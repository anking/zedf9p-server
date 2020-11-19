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

socket implementation description used for local sockets can be found here:
https://medium.com/@goelhardik/http-connection-to-unix-socket-from-dotnet-core-in-c-21d19ef08f8a

 */

using System;
using System.IO;
using System.IO.Pipes;
using System.IO.Ports;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using UBLOX;

namespace Zedf9p
{
    class Program
    {
        const int RTCM_BUFFER_SIZE = 500; //The size of the RTCM correction data varies but in general it is approximately 2000 bytes every second (~2500 bytes every 10th second when 1230 is transmitted).
        const int NTRIP_INCOMING_BUFFER_SIZE = 10000; //Size of incoming buffer for the connection with NTRIP Caster

        //parameters        
        static bool _debug = false;     //debug flag
        static bool _server = false;    //if set to true program will configure gps as NTRIP server
        static bool _client = false;    //if set to true program will configure gps as NTRIP client
        static string _nmeaDataSocketLocation = "/tmp/zed-f9p-nmea-data.sock"; //location of the interprocess socket(used to communicate with nodejs server)
        static string _rtcmDataSocketLocation = "/tmp/zed-f9p-rtcm-data.sock"; //location of the interprocess socket(used to communicate with nodejs server)
        static string _syncDataSocketLocation = "/tmp/zed-f9p-sync-data.sock"; //location of the interprocess socket(used to communicate with nodejs server)

        //Ublox f9p gps module
        static SFE_UBLOX_GPS _myGPS;

        //Gps Module and NTRIP Caster settings
        static string _port = "";           //port the module is connected to (COM#) or ttyACM#
        static Socket _ntripCasterSocket;   //socket for connection with NTRIP caster
        static byte[] ntripOutgoingBuffer = new byte[RTCM_BUFFER_SIZE];
        static int rtcmFrame = 0;           //number of currect rtcm frame (for incoming rtcm data buffer control)
        static byte[] ntripIncomingBuffer = new byte[NTRIP_INCOMING_BUFFER_SIZE];
        static string _ntripServer;         //caster server
        static int _ntripPort;              //caster port
        static string _ntripMountpoint;     //caster mountpoint
        static string _ntripPassword;       //caster password
        static float _rtcmAccuracy;         //Minimum accuracy to be accepted before survey completes in meters (3.000F)/float
        static int _rtcmSurveyTime;         //minimum time required for the survey to complete

        // f9p socket for interapplication communication with node express server
        static Socket _nmeaDataSocket;
        static Socket _rtcmDataSocket;
        static Socket _syncDataSocket;


        async static Task Main(string[] args)
        {
            try
            {
                parseParams(args);

                Console.WriteLine("Hello F9P!");

                //start interprocess communication
                connectDataSockets();

                //if command line parameter for server was passed
                if (_server)
                {
                    //start NTRIP server (base station)
                    await configureGpsAsNtripServer(_rtcmAccuracy, _rtcmSurveyTime);

                    while (_myGPS != null)
                    {
                        //if base station configures successfully start pulling data from it
                        _myGPS.checkUblox(); //See if new data is available. Process bytes as they come in.

                        Thread.Sleep(10);
                    }
                }
                else if (_client)
                {
                    //start NTRIP client (rover)
                    await configureGpsAsNtripClient();

                    while (_ntripCasterSocket != null && _ntripCasterSocket.Connected)
                    {
                        var rcvLen = _ntripCasterSocket.Receive(ntripIncomingBuffer);
                        //var ntripResponseMessage = Encoding.ASCII.GetString(ntripIncomingBuffer, 0, rcvLen);

                        Console.WriteLine("Sending RTCM info to module");

                        _myGPS.send(ntripIncomingBuffer, rcvLen);

                        //if base station configures successfully start pulling data from it
                        _myGPS.checkUblox(); //See if new data is available. Process bytes as they come in.

                        Thread.Sleep(10);
                    }
                }
                else 
                {
                    Console.WriteLine("No application mode selected, exiting.....");
                }

                Environment.Exit(0);
            }
            catch (UnauthorizedAccessException uae)
            {
                Console.WriteLine("UnauthorizedAccessException happened: " + uae.Message);
            }
            catch (TimeoutException te)
            {
                Console.WriteLine("Timeout exception happened: " + te.Message);
            }
            catch (ArgumentException ea)
            {
                Console.WriteLine(ea.Message);
            }
            catch (Exception e)
            {
                Console.WriteLine("Error: "+e.Source+" - "+e.Message+" - "+e.StackTrace);
            }
            finally
            {
                // in the end, disconnect and close the socket to cleanup
                if (_rtcmDataSocket != null)
                {
                    _rtcmDataSocket.Disconnect(false);
                    _rtcmDataSocket.Close();
                }
                
                if (_nmeaDataSocket != null)
                {
                    _nmeaDataSocket.Disconnect(false);
                    _nmeaDataSocket.Close();
                }
                
                if (_syncDataSocket != null)
                {
                    _syncDataSocket.Disconnect(false);
                    _syncDataSocket.Close();
                }

                if (_ntripCasterSocket != null)
                {
                    _ntripCasterSocket.Disconnect(false);
                    _ntripCasterSocket.Close();
                }

                Environment.Exit(0);
            }
        }

        //parse incoming parameters
        static void parseParams(string[] args)
        {
            for (int i = 0; i < args.Length; i++)
            {
                switch (args[i].ToLower())
                {
                    case "-debug": _debug = true; break;
                    case "-server": _server = true; break;
                    case "-client": _client = true; break;
                    case "-com-port": _port = args[++i]; break;
                    case "-ntrip-server": _ntripServer = args[++i]; break;
                    case "-ntrip-port": _ntripPort = int.Parse(args[++i]); break;
                    case "-ntrip-mountpoint": _ntripMountpoint = args[++i]; break;
                    case "-ntrip-password": _ntripPassword = args[++i]; break;
                    case "-rtcm-accuracy-req": _rtcmAccuracy = float.Parse(args[++i]); break;
                    case "-rtcm-survey-time": _rtcmSurveyTime = int.Parse(args[++i]); break;
                    default: throw new ArgumentException("Argument not identified: " + args[i]);
                }
            }
        }

        //connect to f9p socket for interprocess communication
        static void connectDataSockets()
        {
            try
            {
                Console.WriteLine("trying to connect to interpsrocess sockets");

                //Client socket for HTTP server running on Node
                if (File.Exists(_rtcmDataSocketLocation)) _rtcmDataSocket = new Socket(AddressFamily.Unix, SocketType.Stream, ProtocolType.IP);
                if (File.Exists(_syncDataSocketLocation)) _syncDataSocket = new Socket(AddressFamily.Unix, SocketType.Stream, ProtocolType.IP);
                if (File.Exists(_nmeaDataSocketLocation)) _nmeaDataSocket = new Socket(AddressFamily.Unix, SocketType.Stream, ProtocolType.IP);

                //Unix endpoint creates the address where the socket exists
                _rtcmDataSocket?.Connect(new UnixEndPoint(_rtcmDataSocketLocation)); // connect to the RTCM socket
                _syncDataSocket?.Connect(new UnixEndPoint(_syncDataSocketLocation)); // connect to the SYNC socket                 
                _nmeaDataSocket?.Connect(new UnixEndPoint(_nmeaDataSocketLocation)); // connect to the NMEA socket                 
            }
            catch (SocketException ese)
            {
                //unable to connect to the socket, spit out error in a console
                Console.WriteLine("Socket connection error: " + ese.Message);
            }
        }

        //Configure f9p module as ntrip server (will be sending out RTCM data to ntrip caster)
        static async Task configureGpsAsNtripClient()
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
                Console.WriteLine("Authentiction error: " + ntripResponseMessage);
                throw new Exception("NTRIP Authentication error or station down");
            }

            //ATTACH NMEA HANDLER
            _myGPS.attachNMEAHandler(processNmeaClient);
        }

        //Configure f9p module as ntrip server (will be sending out RTCM data to ntrip caster)
        static async Task configureGpsAsNtripServer(float rtcmAccuracy, int rtcmSurveyTime)
        {
            if (rtcmAccuracy == 0 || rtcmSurveyTime == 0) throw new InvalidDataException("Survey accuracy or minimum time not provided, cannot continue...");

            Console.WriteLine("Create serial port connection for f9p module...");
            _myGPS = new SFE_UBLOX_GPS(_port, _debug);

            Console.WriteLine("Enable Survey Mode, minimum " + rtcmSurveyTime + "sec and minimum accuracy " + rtcmAccuracy + " meters");

            var moduleResponse = _myGPS.enableSurveyMode((ushort)rtcmSurveyTime, rtcmAccuracy); //Enable Survey in, 60 seconds, 5.0m

            Console.WriteLine("Enable RTCM Messaging on USB");

            moduleResponse &= _myGPS.enableRTCMmessage(SFE_UBLOX_GPS.UBX_RTCM_1005, SFE_UBLOX_GPS.COM_PORT_USB, 1);
            moduleResponse &= _myGPS.enableRTCMmessage(SFE_UBLOX_GPS.UBX_RTCM_1074, SFE_UBLOX_GPS.COM_PORT_USB, 1);
            moduleResponse &= _myGPS.enableRTCMmessage(SFE_UBLOX_GPS.UBX_RTCM_1084, SFE_UBLOX_GPS.COM_PORT_USB, 1);
            moduleResponse &= _myGPS.enableRTCMmessage(SFE_UBLOX_GPS.UBX_RTCM_1094, SFE_UBLOX_GPS.COM_PORT_USB, 1);
            moduleResponse &= _myGPS.enableRTCMmessage(SFE_UBLOX_GPS.UBX_RTCM_1124, SFE_UBLOX_GPS.COM_PORT_USB, 1);
            moduleResponse &= _myGPS.enableRTCMmessage(SFE_UBLOX_GPS.UBX_RTCM_1230, SFE_UBLOX_GPS.COM_PORT_USB, 10);

            if (!moduleResponse) throw new Exception("Unable to properly configure module");

            //ATTACH NMEA HANDLER
            _myGPS.attachNMEAHandler(processNmeaServer);

            //Get receiver mode
            var receiverMode = _myGPS.getReceiverMode();
            Console.WriteLine("Accuracy: " + receiverMode.getAccuracyLimit());
            _syncDataSocket.Send(Encoding.ASCII.GetBytes("SET_ACCURACY:" + receiverMode.getAccuracyLimit() + "\r\n"));
            Console.WriteLine("Mode: " + receiverMode.getMode());
            _syncDataSocket.Send(Encoding.ASCII.GetBytes("RECEIVER_MODE:" + receiverMode.getMode() + "\r\n"));


            Console.WriteLine("Start requestiong survey status...");

            //Begin waiting for survey to complete
            while (_myGPS.svin.valid == false)
            {

                //Query module for SVIN status with 2000ms timeout (req can take a long time)
                moduleResponse = _myGPS.getSurveyStatus(2000);

                //if module response if always true
                if (moduleResponse)
                {
                    Console.Write("Time elapsed: ");
                    Console.Write(_myGPS.svin.observationTime);
                    _syncDataSocket.Send(Encoding.ASCII.GetBytes("SURVEY_TIME:" + _myGPS.svin.observationTime.ToString() + "\r\n"));

                    Console.Write(" Accuracy: ");
                    Console.Write(_myGPS.svin.meanAccuracy);
                    _syncDataSocket.Send(Encoding.ASCII.GetBytes("ACCURACY:"+_myGPS.svin.meanAccuracy.ToString()+"\r\n"));

                    Console.Write(" Is Valid?: ");
                    Console.WriteLine(_myGPS.svin.valid);
                    _syncDataSocket.Send(Encoding.ASCII.GetBytes("SURVEY_VALID:" + _myGPS.svin.valid.ToString() + "\r\n"));
                }
                else
                {
                    Console.WriteLine("SVIN request failed");
                }

                _myGPS.checkUblox(); //See if new data is available in COM PORT and consume it

                Thread.Sleep(1000);
            }

            Console.WriteLine("Base survey complete! RTCM now broadcasting.");

            //myGPS.setI2COutput(COM_TYPE_UBX | COM_TYPE_RTCM3); //Set the I2C port to output UBX and RTCM sentences (not really an option, turns on NMEA as well)


            //ATTACH RTCM HANDLER            
            _myGPS.attachRTCMHandler(processRtcmServer);

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

        //Opens the connection to the NTRIP (creates a socket)
        private static async Task ConnectNtripCaster()
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
        static async Task processNmeaClient(byte incoming)
        {
            //send nmea info thru socket to the http server app
            if (_nmeaDataSocket != null && _nmeaDataSocket.Connected)
            {
                _nmeaDataSocket.Send(new byte[1] { incoming });
            }
            else 
            {
                Console.WriteLine("error: Nmea Socket not open...");
            }
        }

        //handler for NMEA data coming from the module when creating server (-server)
        static async Task processNmeaServer(byte incoming)
        {
            //send nmea info thru socket to the http server app
            if (_nmeaDataSocket != null && _nmeaDataSocket.Connected) {
                _nmeaDataSocket.Send(new byte[1] { incoming });
            }
        }

        //handler for RTCM data coming from the module when creating server (-server)
        static async Task processRtcmServer(byte incoming)
        {

            //send rtcm info thru socket to the http server app
            if (_rtcmDataSocket != null && _rtcmDataSocket.Connected)
            {
                _rtcmDataSocket.Send(new byte[1] { incoming });
            }

            //check if ntrip socket is defined and connected
            if (_ntripCasterSocket != null && _ntripCasterSocket.Connected)
            {
                //push RTCM bute into the receive buffer until it reaches maximum defined size
                ntripOutgoingBuffer[rtcmFrame++] = incoming;

                //once max buffer size is reached send rtcm data to the RTCM caster thru the socket via NTRIP protocol
                if (rtcmFrame == RTCM_BUFFER_SIZE)
                {
                    rtcmFrame = 0;

                    Console.WriteLine("Sending RTCM");

                    _ntripCasterSocket.Send(ntripOutgoingBuffer);
                }
            } 
            else
            {
                //check if socket was closed
                Console.WriteLine("Socket disconnected?");
            }
        }        
    }
}
