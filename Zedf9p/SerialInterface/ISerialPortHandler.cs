using System;
using System.IO.Ports;
using System.Threading.Tasks;

namespace Zedf9p.SerialInterface
{
    public interface ISerialPortHandler
    {
        /// <summary>
        /// Opens the serial port asynchronously.
        /// </summary>
        /// <param name="portName">The name of the serial port to open.</param>
        /// <param name="timeout">The timeout duration for opening the port.</param>
        /// <returns>A Task that represents the asynchronous operation. Returns the opened SerialPort if successful, otherwise null.</returns>
        Task<SerialPort> OpenPortAsync(
            string portName,
            TimeSpan timeout,
            int baudRate = 115200,
            Parity parity = Parity.None,
            int dataBits = 8,
            StopBits stopBits = StopBits.One);

        /// <summary>
        /// Closes the serial port if it's open.
        /// </summary>
        //void ClosePort();

        /// <summary>
        /// Checks if the serial port is currently open.
        /// </summary>
        /// <returns>True if the serial port is open, otherwise false.</returns>
        //bool IsPortOpen();
    }
}