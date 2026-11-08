using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Globalization;
using Zedf9p.Enums;
using Zedf9p.Exceptions;

namespace Zedf9p.Interfaces
{
    class SyncIncomingCommand
    {
        private readonly string[] _parts;

        public SyncIncomingCommandType Type = SyncIncomingCommandType.UNDEFINED;

        public SyncIncomingCommand(string command)
        {
            _parts = command.Trim().Split(':');

            if (!Enum.TryParse(_parts[0], out Type) || !Enum.IsDefined(Type)) throw new UnknownSyncCommandException($"Command cannot be parsed: {_parts[0]}");
        }

        public int ValueCount => _parts.Length - 1;

        /// <summary>
        /// Gets the value that was provided with a command
        /// </summary>
        /// <typeparam name="T">Type of value to get</typeparam>
        /// <param name="valueNumber">Sequential number of a param in a string(zero-indexed)</param>
        /// <returns></returns>
        public T GetValue<T>(int valueNumber = 0)
        {
            var converter = TypeDescriptor.GetConverter(typeof(T));
            return (T)converter.ConvertFromString(null, CultureInfo.InvariantCulture, _parts[valueNumber + 1]);
        }
    }

    class ErrorFlags(Action<ErrorFlags> sendHandler)
    {
        private readonly object _lock = new();
        private readonly List<string> _errors = [];
        private readonly Action<ErrorFlags> _sendHandler = sendHandler;

        /// <summary>A copy of the current flags: they change from several tasks.</summary>
        public List<string> Errors
        {
            get { lock (_lock) return [.. _errors]; }
        }

        public ErrorFlags Add(string errorFlag)
        {
            bool changed;
            lock (_lock)
            {
                changed = !_errors.Contains(errorFlag);
                if (changed) _errors.Add(errorFlag);
            }

            if (changed) Send(); //send updates immediately
            return this;
        }

        public ErrorFlags Remove(string errorFlag)
        {
            bool changed;
            lock (_lock)
            {
                changed = _errors.Remove(errorFlag);
            }

            if (changed) Send(); //send updates immediately
            return this;
        }

        private ErrorFlags Send()
        {
            _sendHandler?.Invoke(this);

            return this;
        }
    }
}
