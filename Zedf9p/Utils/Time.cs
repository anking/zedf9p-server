using System;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;

namespace Zedf9p.Utils
{
    public static class Time
    {
        public static long millis() => DateTimeOffset.Now.ToUnixTimeMilliseconds();
    }
}
