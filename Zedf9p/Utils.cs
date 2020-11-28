using System;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;

namespace Zedf9p
{
    public static class Utils
    {
        public static long millis() => DateTimeOffset.Now.ToUnixTimeMilliseconds();

        /// <summary>
        /// Retries action multiple times after which throws an exception
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="action"></param>
        /// <param name="retries"></param>
        /// <param name="retryDelay"></param>
        public static void RetryHelper<T>(this Action action, int retries, TimeSpan retryDelay, Action afterFailHandler = null) where T : Exception
        {
            if (action == null)
                throw new ArgumentNullException("action");

            while (retries-- > 0)
            {
                try
                {
                    action();
                    return;
                }
                catch (T e)
                {
                    Console.WriteLine(e.Message);
                    afterFailHandler?.Invoke();
                    Thread.Sleep(retryDelay);
                }
            }

            action();
        }
    }
}
