using System;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;

namespace Zedf9p
{
    public class Utils
    {
        public static long millis() => DateTimeOffset.Now.ToUnixTimeMilliseconds();
    }
}
