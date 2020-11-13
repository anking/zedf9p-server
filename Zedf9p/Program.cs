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

To launch this on pi run the following
/home/pi/f9p/Zedf9p -com-port /dev/ttyACM0 -ntrip-server rtk2go.com -ntrip-password 6n9c2TxqKwuc -ntrip-port 2101 -ntrip-mountpoint Wexford

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
        const float MIN_ACCURACY_REQUIRED = 20.000F;
        const int MIN_SYRVEY_TIME_REQUIRED_SEC = 20;
        const int RTCM_BUFFER_SIZE = 500; //The size of the RTCM correction data varies but in general it is approximately 2000 bytes every second (~2500 bytes every 10th second when 1230 is transmitted).
        const int NTRIP_INCOMING_BUFFER_SIZE = 1000; //Size of incoming buffer for the connection with NTRIP Caster

        //parameters
        static string _port = "";
        static bool _debug = false;

        //ntrip socket
        static Socket _ntripSocket;
        static byte[] ntripOutgoingBuffer = new byte[RTCM_BUFFER_SIZE];
        static int rtcmFrame = 0;
        static byte[] ntripIncomingBuffer = new byte[NTRIP_INCOMING_BUFFER_SIZE];
        static string _ntripServer;
        static int _ntripPort;
        static string _ntripMountpoint;
        static string _ntripPassword;

        //Pipes for interapplication communication
        static NamedPipeClientStream clientPipeToNode;

        // f9p socket - local socket to communicate with node express server
        static Socket _f9pSocket;


        async static Task Main(string[] args)
        {
            try
            {
                parseParams(args);

                Console.WriteLine("Hello F9p!");
                Console.WriteLine("Create serial port");

                SFE_UBLOX_GPS myGPS = new SFE_UBLOX_GPS(_port, _debug);

                Console.WriteLine("Enable Survey Mode, minimum " + MIN_SYRVEY_TIME_REQUIRED_SEC + "sec and minimum accuracy " + MIN_ACCURACY_REQUIRED + " meters");

                var response = myGPS.enableSurveyMode(MIN_SYRVEY_TIME_REQUIRED_SEC, MIN_ACCURACY_REQUIRED); //Enable Survey in, 60 seconds, 5.0m

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

                await StartNTRIP();

                startPipes();

                while (true)
                {
                    myGPS.checkUblox(); //See if new data is available. Process bytes as they come in.

                    Thread.Sleep(10);
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
            finally
            {
                // in the end, disconnect and close the socket to cleanup
                _f9pSocket.Disconnect(false);
                _f9pSocket.Close();

                Environment.Exit(0);
            }
        }

        static void startPipes()
        {
            try
            {
                //Client Pipe for HTTP server running on Node
                //clientPipeToNode = new Socket(AddressFamily.InterNetwork, ProtocolType.Tcp, ProtocolType.);
                //clientPipeToNode.Connect();
                //StreamReader reader = new StreamReader(clientPipeToNode);
                //StreamWriter writer = new StreamWriter(clientPipeToNode);

                _f9pSocket = new Socket(AddressFamily.Unix, SocketType.Stream, ProtocolType.IP);
                var unixEndpoint = new UnixEndPoint("/tmp/Zedf9p.sock"); // this address is where the socket exists
                _f9pSocket.Connect(unixEndpoint); // connect to the socket

                //socket.Send(new byte[4] { 0, 1, 2, 3 });

                // assuming we want the server to answer to the /getMyData route with a query parameter; create a request like this in HTTP spec format
                //var request = $"GET /getMyData?id=testIdValue "
                //  + "HTTP/1.1\r\n"
                //  + "Host: localhost\r\n"
                //  + "Content-Length: 0\r\n"
                //  + "\r\n";
                //// convert the request into byte data
                //byte[] requestBytes = Encoding.ASCII.GetBytes(request);
                //_f9pSocket.Send(requestBytes);
                //// receive the response; assuming it's less than 1024 bytes. If it can be more, keep receiving data and keep appending it to a final response array
                //byte[] bytesReceived = new byte[1024];
                //int numBytes = _f9pSocket.Receive(bytesReceived);
                
            }
            catch (SocketException ese) {
                Console.WriteLine(ese.Message);
            }
        }

        static void parseParams(string[] args)
        {
            for (int i = 0; i < args.Length; i++)
            {
                switch (args[i].ToLower())
                {
                    case "-com-port": _port = args[++i]; break;
                    case "-debug": _debug = true; break;
                    case "-ntrip-server": _ntripServer = args[++i]; break;
                    case "-ntrip-port": _ntripPort = int.Parse(args[++i]); break;
                    case "-ntrip-mountpoint": _ntripMountpoint = args[++i]; break;
                    case "-ntrip-password": _ntripPassword = args[++i]; break;
                    default: throw new ArgumentException("Argument not identified: " + args[i]);
                }
            }
        }

        static async Task processNMEA(byte incoming)
        {
            //Console.Write((char)incoming);

            //if (clientPipeToNode.IsConnected && clientPipeToNode.CanWrite)
            //{
            //    //var data = new byte[4] { 0, 1, 2, 3 };
            //    clientPipeToNode.WriteByte(incoming);//WriteAsync(data, 0, data.Length);
            //}

            if (_f9pSocket != null && _f9pSocket.Connected) {
                //byte[] requestBytes = Encoding.ASCII.GetBytes(request);
                //byte[] requestBytes = Encoding.ASCII.GetBytes();

                _f9pSocket.Send(new byte[1] { incoming });
            }
        }

        static async Task processRTCM(byte incoming)
        {
            if (_ntripSocket != null && _ntripSocket.Connected)
            {

                ntripOutgoingBuffer[rtcmFrame++] = incoming;

                if (rtcmFrame == RTCM_BUFFER_SIZE)
                {
                    rtcmFrame = 0;

                    Console.WriteLine("Sending RTCM");

                    _ntripSocket.Send(ntripOutgoingBuffer);
                }
            }
        }

        /// <summary>
        /// Opens the connection to the NTRIP server and starts sending
        /// </summary>
        private static async Task StartNTRIP()
        {
            if (_ntripServer != null && _ntripMountpoint != null && _ntripPassword != null && _ntripPort != 0)
            {

                var hostEntry = await Dns.GetHostEntryAsync(_ntripServer);
                var BroadCasterIP = hostEntry.AddressList[0]; //Select correct Address
                var BroadCasterPort = _ntripPort; //Select correct port 

                //Connect to server
                _ntripSocket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
                //sckt.Blocking = true;
                _ntripSocket.Connect(new IPEndPoint(BroadCasterIP, BroadCasterPort));

                //send credentials to ntrip caster
                string tosend = "SOURCE " + _ntripPassword + " /" + _ntripMountpoint + "\r\n";
                tosend += string.Format("Source-Agent: %s/%s\r\n\r\n", "NTRIP NtripServerPOSIX", "$Revision: 9109 $");

                //Send auth request
                _ntripSocket.Send(Encoding.ASCII.GetBytes(tosend));

                //Receive respone with ICY 200 OK or ERROR - Bad Password
                var rcvLen = _ntripSocket.Receive(ntripIncomingBuffer);
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
    }
}
