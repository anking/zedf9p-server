using System;

namespace Zedf9p.Ntrip
{
    /// <summary>
    /// Doubling delay between caster connection attempts. Public casters such as RTK2go count failed
    /// connections per IP and ban the address, which would also take out anything else behind it.
    /// </summary>
    internal sealed class ReconnectBackoff(TimeSpan initial, TimeSpan maximum)
    {
        private TimeSpan _next = initial;

        public static ReconnectBackoff ForCaster() => new(TimeSpan.FromSeconds(15), TimeSpan.FromMinutes(15));

        /// <summary>How long to wait after the caster says this IP is banned.</summary>
        public static readonly TimeSpan BannedDelay = TimeSpan.FromHours(1);

        /// <summary>A connection must last this long before its failure counts as a fresh start.</summary>
        public static readonly TimeSpan HealthyConnection = TimeSpan.FromSeconds(60);

        public TimeSpan NextDelay()
        {
            var delay = _next;
            var doubled = TimeSpan.FromTicks(_next.Ticks * 2);
            _next = doubled > maximum ? maximum : doubled;
            return delay;
        }

        public void Reset() => _next = initial;
    }
}
