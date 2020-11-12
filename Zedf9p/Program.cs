/*
 to publish application for linux/raspberry PI environment:
dotnet publish -c Release --self-contained -r linux-arm

Params
-port [ComPort]
-debug              enables debug output
-ntrips [passwd@]addr[:port][/mntpnt[:str]]      //NTRIP Server
 */

using System;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;
using UBLOX;

namespace Zedf9p
{
    class Program
    {
        const float MIN_ACCURACY_REQUIRED = 20.000F;
        const int MIN_SYRVEY_TIME_REQUIRED_SEC = 20;

        //parameters
        static string _port = "";
        static bool _debug = false;


        async static Task Main(string[] args)
        {
            try
            {
                await StartNTRIP();

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

                StartNTRIP();

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
                    case "-port": _port = args[++i]; break;
                    case "-debug": _debug = true; break;
                    default: throw new ArgumentException("Argument not identified: " + args[i]);
                }
            }
        }

        static byte processNMEA(byte incoming)
        {
            Console.Write((char)incoming);

            return 0;
        }

        static byte processRTCM(byte incoming)
        {
            Console.Write((char)incoming);

            return 0;
        }

        /// <summary>
        /// Opens the connection to the NTRIP server and starts receiving
        /// </summary>
        private static async Task StartNTRIP()
        {
            var he = await Dns.GetHostEntryAsync("rtk2go.com");



            var BroadCasterIP = IPAddress.Parse("129.217.182.51"); //Select correct Address
            var BroadCasterPort = 80; //Select correct port (usually 80)
            var stream = "FLEN0"; //Insert the correct stream
            var username = "USERNAME"; //Insert your username!
            var password = "PASSWORD"; //Insert your password!

            //Connect to server
            var sckt = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
            sckt.Blocking = true;
            sckt.Connect(new IPEndPoint(BroadCasterIP, BroadCasterPort));

            //Build request message
            //string auth = ToBase64(username + ":" + password);
            //string msg = "GET /" + stream + " HTTP/1.1\r\n";
            //msg += "User-Agent: NTRIP iter.dk\r\n";
            //msg += "Authorization: Basic " + auth + "\r\n"; //This line can be removed if no authorization is needed
            //msg += "Accept: */*\r\nConnection: close\r\n";
            //msg += "\r\n";

            ////Send request
            //byte[] data = System.Text.Encoding.ASCII.GetBytes(msg);
            //sckt.Send(data);

            //byte[] returndata = new byte[256];

            //Thread.Sleep(100); //Wait for response
            //sckt.Receive(returndata); //Get response
            //string responseData = System.Text.Encoding.ASCII.GetString(returndata, 0, returndata.Length);
            //ShowResponse(responseData);
        }
    }
}
