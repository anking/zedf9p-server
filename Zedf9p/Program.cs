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
        const int NTRIP_INCOMING_BUFFER_SIZE = 1000; //Size of incoming buffer for the connection with NTRIP Caster

        //parameters        
        static bool _debug = false;     //debug flag
        static bool _server = false;    //if set to true program will configure gps as NTRIP server
        static bool _client = false;    //if set to true program will configure gps as NTRIP client

        //Ublox f9p gps module
        static SFE_UBLOX_GPS myGPS;

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
        static Socket _serverAppSocket;


        async static Task Main(string[] args)
        {
            try
            {
                parseParams(args);

                Console.WriteLine("Hello F9P!");

                //start interprocess communication
                connectF9pSocket();

                //if command line parameter for server was passed
                if (_server)
                {
                    //start NTRIP server (base station)
                    configureGpsAsNtripServer(_rtcmAccuracy, _rtcmSurveyTime);

                    //if base station configures successfully start pulling data from it
                    while (myGPS != null)
                    {
                        myGPS.checkUblox(); //See if new data is available. Process bytes as they come in.

                        Thread.Sleep(10);
                    }
                }

                if (_client)
                {
                    ////start NTRIP server (base station)
                    //configureGpsAsNtripServer(_rtcmAccuracy, _rtcmSurveyTime);

                    ////if base station configures successfully start pulling data from it
                    //while (myGPS != null)
                    //{
                    //    myGPS.checkUblox(); //See if new data is available. Process bytes as they come in.

                    //    Thread.Sleep(10);
                    //}
                }

                Environment.Exit(0);
            }
            catch (UnauthorizedAccessException uae)
            {
                Console.WriteLine("UnauthorizedAccessException happened");
            }
            catch (TimeoutException te)
            {
                Console.WriteLine("Timeout exception happened");
            }
            catch (ArgumentException ea)
            {
                Console.WriteLine(ea.Message);
            }
            catch (Exception e)
            {
                Console.WriteLine("Error: "+e.Message);
            }
            finally
            {
                // in the end, disconnect and close the socket to cleanup
                if (_serverAppSocket != null)
                {
                    _serverAppSocket.Disconnect(false);
                    _serverAppSocket.Close();
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
        static void connectF9pSocket()
        {
            try
            {

                //Client socket for HTTP server running on Node
                _serverAppSocket = new Socket(AddressFamily.Unix, SocketType.Stream, ProtocolType.IP);

                var unixEndpoint = new UnixEndPoint("/tmp/Zedf9p.sock"); // this address is where the socket exists

                _serverAppSocket.Connect(unixEndpoint); // connect to the socket                 
            }
            catch (SocketException ese)
            {
                //unable to connect to the socket, spit out error in a console
                Console.WriteLine(ese.Message);
            }
        }

        //Configure f9p module as ntrip server (will be sending out RTCM data to ntrip caster)
        static async void configureGpsAsNtripServer(float rtcmAccuracy, int rtcmSurveyTime)
        {
            if (rtcmAccuracy == 0 || rtcmSurveyTime == 0) throw new InvalidDataException("Survey accuracy or minimum time not provided, cannot continue...");

            Console.WriteLine("Create serial port connection for f9p module...");
            myGPS = new SFE_UBLOX_GPS(_port, _debug);

            Console.WriteLine("Enable Survey Mode, minimum " + rtcmSurveyTime + "sec and minimum accuracy " + rtcmAccuracy + " meters");

            var response = myGPS.enableSurveyMode((ushort)rtcmSurveyTime, rtcmAccuracy); //Enable Survey in, 60 seconds, 5.0m

            Console.WriteLine("Enable RTCM Messaging on USB");

            response &= myGPS.enableRTCMmessage(SFE_UBLOX_GPS.UBX_RTCM_1005, SFE_UBLOX_GPS.COM_PORT_USB, 1);
            response &= myGPS.enableRTCMmessage(SFE_UBLOX_GPS.UBX_RTCM_1074, SFE_UBLOX_GPS.COM_PORT_USB, 1);
            response &= myGPS.enableRTCMmessage(SFE_UBLOX_GPS.UBX_RTCM_1084, SFE_UBLOX_GPS.COM_PORT_USB, 1);
            response &= myGPS.enableRTCMmessage(SFE_UBLOX_GPS.UBX_RTCM_1094, SFE_UBLOX_GPS.COM_PORT_USB, 1);
            response &= myGPS.enableRTCMmessage(SFE_UBLOX_GPS.UBX_RTCM_1124, SFE_UBLOX_GPS.COM_PORT_USB, 1);
            response &= myGPS.enableRTCMmessage(SFE_UBLOX_GPS.UBX_RTCM_1230, SFE_UBLOX_GPS.COM_PORT_USB, 10);

            Console.WriteLine("Start requestiong survey status...");

            //Begin waiting for survey to complete
            while (myGPS.svin.valid == false)
            {
                response = myGPS.getSurveyStatus(2000); //Query module for SVIN status with 2000ms timeout (req can take a long time)
                if (response == true)
                {
                    Console.Write("Time elapsed: ");
                    Console.Write(myGPS.svin.observationTime);

                    Console.Write(" Accuracy: ");
                    Console.Write(myGPS.svin.meanAccuracy);

                    Console.Write(" Is Valid?: ");
                    Console.WriteLine(myGPS.svin.valid);
                }
                else
                {
                    Console.WriteLine("SVIN request failed");
                }

                Thread.Sleep(1000);
            }

            Console.WriteLine("Base survey complete! RTCM now broadcasting.");

            //myGPS.setI2COutput(COM_TYPE_UBX | COM_TYPE_RTCM3); //Set the I2C port to output UBX and RTCM sentences (not really an option, turns on NMEA as well)


            //ATTACH RTCM AND NMEA HANDLERS
            myGPS.attachNMEAHandler(processNMEA);
            myGPS.attachRTCMHandler(processRTCM);

            //start sending data to ntrip caster
            await StartNtripServer();
        }

        //Opens the connection to the NTRIP server and starts sending
        private static async Task StartNtripServer()
        {
            if (_ntripServer != null && _ntripMountpoint != null && _ntripPassword != null && _ntripPort != 0)
            {

                var hostEntry = await Dns.GetHostEntryAsync(_ntripServer);
                var BroadCasterIP = hostEntry.AddressList[0]; //Select correct Address
                var BroadCasterPort = _ntripPort; //Select correct port 

                //Connect to server
                _ntripCasterSocket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
                //sckt.Blocking = true;
                _ntripCasterSocket.Connect(new IPEndPoint(BroadCasterIP, BroadCasterPort));

                //send credentials to ntrip caster
                string tosend = "SOURCE " + _ntripPassword + " /" + _ntripMountpoint + "\r\n";
                tosend += string.Format("Source-Agent: %s/%s\r\n\r\n", "NTRIP NtripServerPOSIX", "$Revision: 9109 $");

                //Send auth request
                _ntripCasterSocket.Send(Encoding.ASCII.GetBytes(tosend));

                //Receive respone with ICY 200 OK or ERROR - Bad Password
                var rcvLen = _ntripCasterSocket.Receive(ntripIncomingBuffer);
                var response = Encoding.ASCII.GetString(ntripIncomingBuffer, 0, rcvLen);

                if (response.Trim().Equals("ICY 200 OK"))
                {
                    Console.WriteLine("Authentication passed!");
                }
                else
                {
                    Console.WriteLine("Authentiction error: " + response);
                    throw new Exception("NTRIP AUthentication error");
                }
            }
            else
            {
                throw new Exception("NTRIP Credentials not provided");
            }
        }     

        //handler for NMEA data coming from the module
        static async Task processNMEA(byte incoming)
        {
            //send nmea info thru the f9p socket to the http server app
            if (_serverAppSocket != null && _serverAppSocket.Connected) {
                _serverAppSocket.Send(new byte[1] { incoming });
            }
        }

        //handler for RTCM data coming from the module
        static async Task processRTCM(byte incoming)
        {
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
