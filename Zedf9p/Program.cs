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
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using UBLOX;
using Zedf9p.Communication;
using Zedf9p.Core;

namespace Zedf9p
{
    //application input params
    class InputParams
    {
        public string port = "";           //port the module is connected to (COM#) or ttyACM#
        public string ntripServer;         //caster server
        public int ntripPort;              //caster port
        public string ntripMountpoint;     //caster mountpoint
        public string ntripPassword;       //caster password
        public float rtcmAccuracy;         //Minimum accuracy to be accepted before survey completes in meters (3.000F)/float
        public int rtcmSurveyTime;         //minimum time required for the survey to complete
        public Driver.OperationMode mode;  //operation mode for the driver
        public bool debug;                 //debug flag
    }

    class Program
    {
        //Ublox f9p gps driver
        static Driver _driver;


        async static Task Main(string[] args)
        {
            try
            {
                var inputParams = parseParams(args);

                Console.WriteLine("Hello F9P!");

                //Create driver
                _driver = new Driver(inputParams);

                await _driver.Initialize(inputParams.mode);

                await _driver.Run();

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
                //for this to show line numbers you neede to include .pdb file
                Console.WriteLine("\r\nError: "+e.Source+"\r\n"+e.Message+"\r\n"+e.StackTrace);
            }
            finally
            {
                _driver.Cleanup();
            }
        }

        //parse incoming parameters
        static InputParams parseParams(string[] args)
        {
            InputParams inputParams = new InputParams();

            for (int i = 0; i < args.Length; i++)
            {
                switch (args[i].ToLower())
                {
                    case "-debug": inputParams.debug = true; break;
                    case "-server": inputParams.mode = Driver.OperationMode.Server; break;
                    case "-client": inputParams.mode = Driver.OperationMode.Client; break;
                    case "-com-port": inputParams.port = args[++i]; break;
                    case "-ntrip-server": inputParams.ntripServer = args[++i]; break;
                    case "-ntrip-port": inputParams.ntripPort = int.Parse(args[++i]); break;
                    case "-ntrip-mountpoint": inputParams.ntripMountpoint = args[++i]; break;
                    case "-ntrip-password": inputParams.ntripPassword = args[++i]; break;
                    case "-rtcm-accuracy-req": inputParams.rtcmAccuracy = float.Parse(args[++i]); break;
                    case "-rtcm-survey-time": inputParams.rtcmSurveyTime = int.Parse(args[++i]); break;
                    default: throw new ArgumentException("Argument not identified: " + args[i]);
                }
            }

            return inputParams;
        }
    }
}
