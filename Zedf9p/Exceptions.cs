using System;
using Zedf9p.Ntrip;

namespace Zedf9p.Exceptions
{
    internal class NtripException(string message, NtripResponseKind? response) : Exception(message)
    {
        /// <summary>What the caster answered, or null when it could not be reached at all.</summary>
        public NtripResponseKind? Response { get; } = response;
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
