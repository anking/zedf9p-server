﻿/*

Application is self-contained .net app that does not need any external deps to run
To compile application for linux/raspberry PI environment 32/64 bit:
dotnet publish -c Release --self-contained -r linux-arm
dotnet publish -c Release --self-contained -r linux-arm64

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
/home/pi/gps-station/f9p/Zedf9p -p /dev/ttyACM0 -s rtk2go.com -t 2101 -m Wexford -w 6n9c2TxqKwuc -a 3.000 -y 60 -o Server -n /tmp/zed-f9p-nmea-data.sock -r /tmp/zed-f9p-rtcm-data.sock -x /tmp/zed-f9p-sync-data.sock -d
/home/pi/gps-station/f9p/Zedf9p --com-port /dev/ttyACM0 --ntrip-server rtk2go.com --ntrip-port 2101 --ntrip-mountpoint SmirnovRTK --ntrip-password arhG4oKZ --rtcm-accuracy-req 3.000 --rtcm-survey-time 60 --mode Server --nmea-socket /tmp/zed-f9p-nmea-data.sock --rtcm-socket /tmp/zed-f9p-rtcm-data.sock --sync-socket /tmp/zed-f9p-sync-data.sock --debug
 
*/

using System;
using System.Threading;
using System.Threading.Tasks;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Hosting;
using CommandLine;
using Zedf9p.Exceptions;
using Zedf9p.Enums;
using Zedf9p.F9p;
using Zedf9p.Sockets;
using Serilog;
using Zedf9p.SerialInterface;
using Serilog.Events;

namespace Zedf9p
{
    class Program
    {
        static async Task Main(string[] args)
        {
            try
            {
                // Create the host and run the application
                await CreateHostBuilder(args).RunConsoleAsync();
            }
            catch (Exception e)
            {
                // Handle any unhandled exceptions here
                Console.WriteLine($"Error: {e.Message}");
            }
        }

        // This configures the DI container and services
        public static IHostBuilder CreateHostBuilder(string[] args)
        {
            // Parse the command-line arguments into InputParams
            var inputParams = new InputParams();
            var result = Parser.Default.ParseArguments<InputParams>(args);

            result.WithParsed(parsedParams =>
            {
                inputParams = parsedParams;
                Console.WriteLine("Input params parsed successfully");
            })
            .WithNotParsed(errors =>
            {
                Console.WriteLine("Error parsing arguments. Please check your input.");
            });

            return Host.CreateDefaultBuilder(args)
                .UseSerilog((context, services, configuration) =>
                {
                    // Set log level based on the debug flag
                    var logLevel = inputParams.Debug ? LogEventLevel.Debug : LogEventLevel.Information;

                    configuration
                        .MinimumLevel.Is(logLevel)
                        .WriteTo.Console(
                            theme: Serilog.Sinks.SystemConsole.Themes.AnsiConsoleTheme.Literate // Apply color theme
                        )
                        .WriteTo.File("logs/log.txt", rollingInterval: RollingInterval.Day) // Optional: Write logs to a file
                        .Enrich.FromLogContext(); // Enrich log with context information
                })
                .ConfigureServices((hostContext, services) =>
                {


                    // Register the parsed InputParams as a singleton
                    services.AddSingleton(inputParams);

                    // Register the Driver service
                    services.AddSingleton<IF9pDriver, F9pDriver>();

                    // Register SerialPortHandler as a service
                    services.AddSingleton<ISerialPortHandler, SerialPortHandler>();

                    // Register SocketCommunications with the custom paths from input parameters
                    services.AddSingleton<ISocketCommunications>(provider =>
                    {
                        var logger = provider.GetRequiredService<ILogger>(); // Get the logger
                        return new SocketCommunications(
                            inputParams.NmeaSocketPath,
                            inputParams.RtcmSocketPath,
                            inputParams.SyncSocketPath,
                            logger
                        );
                    });

                    // Register the application entry point
                    services.AddHostedService<App>();
                });
        }
    }

    public class App(IF9pDriver driver, InputParams inputParams, ISocketCommunications socketCommunications, ILogger logger) : BackgroundService
    {
        private readonly IF9pDriver _driver = driver;
        private readonly ISocketCommunications _socketCommunications = socketCommunications;
        private readonly InputParams _inputParams = inputParams;
        private readonly ILogger _logger = logger;

        protected override async Task ExecuteAsync(CancellationToken cancellationToken)
        {
            try
            {
                _logger.Information("F9P Driver Starting...");

                // Start interprocess communication with Node.js
                _socketCommunications.ConnectDataSockets();

                // Create driver and initialize
                await _driver.Initialize(_inputParams.Mode, cancellationToken);
            }
            catch (NtripException e)
            {
                //if driver fails to connect to NTRIP server put it into idle mode(only listed to incoming messages and spit out GPS data if available
                _driver.SetMode(OperationMode.Idle);
                _logger.Warning($"Ntrip Exception {e.Message}");
                _logger.Information("Setting driver mode to Idle");
            }
            catch (Exception e)
            {
                _logger.Error($"Application Exception: {e.Message}", e);
            }
        }

        public override async Task StopAsync(CancellationToken cancellationToken)
        {
            // Clean up resources before stopping
            _logger.Information("Stopping F9P Driver...");
            _driver.Cleanup();
            await base.StopAsync(cancellationToken);
        }
    }
}