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
 */

using System;
using System.IO;
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
        static Socket socket;
        static byte[] ntripOutgoingBuffer = new byte[RTCM_BUFFER_SIZE];
        static int rtcmFrame = 0;
        static byte[] ntripIncomingBuffer = new byte[NTRIP_INCOMING_BUFFER_SIZE];
        static string _ntripServer;
        static int _ntripPort;
        static string _ntripMountpoint;
        static string _ntripPassword;


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

                while (true)
                {
                    myGPS.checkUblox(); //See if new data is available. Process bytes as they come in.

                    Thread.Sleep(10); //delay(250); //Don't pound too hard on the I2C bus
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
                Environment.Exit(0);
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
            Console.Write((char)incoming);
        }

        static async Task processRTCM(byte incoming)
        {
            if (socket != null && socket.Connected)
            {

                ntripOutgoingBuffer[rtcmFrame++] = incoming;

                if (rtcmFrame == RTCM_BUFFER_SIZE)
                {
                    rtcmFrame = 0;

                    Console.WriteLine("Sending RTCM");

                    socket.Send(ntripOutgoingBuffer);
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
                socket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
                //sckt.Blocking = true;
                socket.Connect(new IPEndPoint(BroadCasterIP, BroadCasterPort));

                //send credentials to ntrip caster
                string tosend = "SOURCE " + _ntripPassword + " /" + _ntripMountpoint + "\r\n";
                tosend += string.Format("Source-Agent: %s/%s\r\n\r\n", "NTRIP NtripServerPOSIX", "$Revision: 9109 $");

                //Send auth request
                socket.Send(Encoding.ASCII.GetBytes(tosend));

                //Receive respone with ICY 200 OK or ERROR - Bad Password
                var rcvLen = socket.Receive(ntripIncomingBuffer);
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
