/*
This is a console application that connects to the u-blox ZED-F9P Precision GNSS Module via a serial interface.
The application configures the module to operate as either an NTRIP Client or Station.
When set as a Station, the module surveys (or uses a fixed position) and then streams RTCM corrections to the caster.
When set as a Client, it receives RTCM corrections from the caster, passes them to the module and reports the position.

To compile the application for Linux/Raspberry Pi environment (32/64-bit):
dotnet publish Zedf9p/Zedf9p.csproj -c Release --self-contained -r linux-arm
dotnet publish Zedf9p/Zedf9p.csproj -c Release --self-contained -r linux-arm64

See readme.md for the parameters.
*/

using System;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Hosting;
using CommandLine;
using Zedf9p.F9p;
using Zedf9p.Sockets;
using Serilog;
using Zedf9p.SerialInterface;
using Serilog.Events;

namespace Zedf9p
{
    class Program
    {
        private const int ExitInvalidArguments = 2;

        static async Task<int> Main(string[] args)
        {
            var parsed = Parser.Default.ParseArguments<InputParams>(args);

            if (parsed.Errors.Any())
            {
                // CommandLineParser has already printed the reason. Running on with empty settings would look healthy and do nothing.
                return parsed.Errors.All(e => e.Tag is ErrorType.HelpRequestedError or ErrorType.VersionRequestedError) ? 0 : ExitInvalidArguments;
            }

            var problems = parsed.Value.Validate();
            if (problems.Count > 0)
            {
                foreach (var problem in problems)
                {
                    Console.Error.WriteLine($"Invalid arguments: {problem}");
                }

                return ExitInvalidArguments;
            }

            try
            {
                // Create the host and run the application
                using var host = CreateHostBuilder(parsed.Value).Build();
                await host.RunAsync();
                return Environment.ExitCode;
            }
            catch (Exception e)
            {
                // Handle any unhandled exceptions here
                Console.Error.WriteLine($"Main Process Error: {e.Message}");
                return 1;
            }
        }

        // This configures the DI container and services
        public static IHostBuilder CreateHostBuilder(InputParams inputParams)
        {
            return Host.CreateDefaultBuilder()
                .UseSerilog((context, services, configuration) =>
                {
                    // Set log level based on the debug flag
                    var logLevel = inputParams.Debug ? LogEventLevel.Debug : LogEventLevel.Information;

                    configuration
                        .MinimumLevel.Is(logLevel)
                        .WriteTo.Console(
                            theme: Serilog.Sinks.SystemConsole.Themes.AnsiConsoleTheme.Literate // Apply color theme
                        )
                        // Bounded, because this runs for months on an SD card
                        .WriteTo.File("logs/log.txt", rollingInterval: RollingInterval.Day, fileSizeLimitBytes: 5 * 1024 * 1024, rollOnFileSizeLimit: true, retainedFileCountLimit: 7)
                        .Enrich.FromLogContext(); // Enrich log with context information
                })
                .ConfigureServices((hostContext, services) =>
                {
                    // Suppress default startup messages
                    services.Configure<ConsoleLifetimeOptions>(options => options.SuppressStatusMessages = true);

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

    public class App(IF9pDriver driver, InputParams inputParams, ISocketCommunications socketCommunications, ILogger logger, IHostApplicationLifetime lifetime) : BackgroundService
    {
        private readonly IF9pDriver _driver = driver;
        private readonly ISocketCommunications _socketCommunications = socketCommunications;
        private readonly InputParams _inputParams = inputParams;
        private readonly ILogger _logger = logger;
        private readonly IHostApplicationLifetime _lifetime = lifetime;

        protected override async Task ExecuteAsync(CancellationToken cancellationToken)
        {
            // Let the host finish starting before the socket retries can block this thread
            await Task.Yield();

            try
            {
                _logger.Information("F9P Driver Starting...");

                // Start interprocess communication with Node.js
                _socketCommunications.ConnectDataSockets();

                // Create driver and run until stopped
                await _driver.Initialize(_inputParams.Mode, cancellationToken);
            }
            catch (OperationCanceledException) when (cancellationToken.IsCancellationRequested)
            {
                _logger.Information("Stop requested");
            }
            catch (Exception e)
            {
                // The owning process restarts the driver when it exits with an error
                Environment.ExitCode = 1;
                _logger.Error(e, "Application Exception: {Message}", e.Message);
            }
            finally
            {
                // Clean up resources before stopping
                _logger.Information("Stopping F9P Driver...");
                _driver.Cleanup();
                _lifetime.StopApplication();
            }
        }
    }
}
