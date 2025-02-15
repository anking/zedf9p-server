using System.Threading;
using System.Threading.Tasks;
using Zedf9p.Enums;

namespace Zedf9p.F9p
{
    public interface IF9pDriver
    {
        // Asynchronous method to start the driver
        Task RunAsync(CancellationToken cancellationToken);

        // Method to initialize the driver with a specified mode
        Task Initialize(OperationMode mode, CancellationToken cancellationToken);

        // Method to set the operational mode of the driver
        void SetMode(OperationMode mode);

        // Method to clean up resources used by the driver
        void Cleanup();
    }
}
