using System;
using System.ComponentModel;
using System.Linq;
using Zedf9p.Enums;
using Zedf9p.Exceptions;

namespace Zedf9p.Models
{
    class SyncIncomingCommand
    {
        string _command;

        public SyncIncomingCommandType Type = SyncIncomingCommandType.UNDEFINED;

        public SyncIncomingCommand(string command)
        {
            _command = command;

            if (!parseCommandString()) throw new UnknownSyncCommandException("Command cannot be parsed");
        }

        bool parseCommandString() => Enum.TryParse(_command.Split(':')[0], out Type);

        /// <summary>
        /// Gets the value that was provided with a command
        /// </summary>
        /// <typeparam name="T">Type of value to get</typeparam>
        /// <param name="valueNumber">Sequential number of a param in a string(zero-indexed)</param>
        /// <returns></returns>
        public T getValue<T>(int valueNumber = 0) {
            try
            {
                var converter = TypeDescriptor.GetConverter(typeof(T));
                if (converter != null)
                {
                    // Cast ConvertFromString(string text) : object to (T)
                    return (T)converter.ConvertFromString(_command.Split(':')[valueNumber+1]);
                }
                return default(T);
            }
            catch (NotSupportedException)
            {
                return default(T);
            }
        }
    }
}