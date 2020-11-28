using System;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;

namespace Zedf9p.Exceptions
{
    public class NtripException : Exception
    {
        public NtripException(string message) : base(message) { }
    }

    public class SyncSocketException : Exception
    {
        public SyncSocketException(string message) : base(message) { }
    }

    public class UnknownSyncCommandException : Exception
    {
        public UnknownSyncCommandException(string message) : base(message) { }
    }
}
