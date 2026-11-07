using System.Threading;
using System.Threading.Tasks;
using Zedf9p.Enums;

namespace Zedf9p.F9p
{
    public interface IF9pDriver
    {
        // Runs the driver in the given mode until cancelled or the owning process goes away
        Task Initialize(OperationMode mode, CancellationToken cancellationToken);

        // Method to clean up resources used by the driver
        void Cleanup();
    }
}
