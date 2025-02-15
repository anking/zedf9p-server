using System;
using System.Collections.Generic;
using System.ComponentModel;
using Zedf9p.Enums;
using Zedf9p.Exceptions;

namespace Zedf9p.Interfaces
{
    class SyncIncomingCommand
    {
        private readonly string _command;

        public SyncIncomingCommandType Type = SyncIncomingCommandType.UNDEFINED;

        public SyncIncomingCommand(string command)
        {
            _command = command;

            if (!ParseCommandString()) throw new UnknownSyncCommandException("Command cannot be parsed");
        }

        bool ParseCommandString() => Enum.TryParse(_command.Split(':')[0], out Type);

        /// <summary>
        /// Gets the value that was provided with a command
        /// </summary>
        /// <typeparam name="T">Type of value to get</typeparam>
        /// <param name="valueNumber">Sequential number of a param in a string(zero-indexed)</param>
        /// <returns></returns>
        public T GetValue<T>(int valueNumber = 0)
        {
            try
            {
                var converter = TypeDescriptor.GetConverter(typeof(T));
                if (converter != null)
                {
                    // Cast ConvertFromString(string text) : object to (T)
                    return (T)converter.ConvertFromString(_command.Split(':')[valueNumber + 1]);
                }
                return default;
            }
            catch (NotSupportedException)
            {
                return default;
            }
        }
    }

    class ErrorFlags(Action<ErrorFlags> sendHandler)
    {
        public List<string> Errors { get; set; } = [];

        private readonly Action<ErrorFlags> _sendHandler = sendHandler;

        public ErrorFlags Add(string errorFlag)
        {
            if (!Errors.Contains(errorFlag))
            {
                Errors.Add(errorFlag);
                Send(); //send updates immediately
            }

            return this;
        }

        public ErrorFlags Remove(string errorFlag)
        {
            if (!Errors.Contains(errorFlag))
            {
                return this;
            }
            Errors.Remove(errorFlag);
            Send(); //send updates immediately

            return this;
        }

        public ErrorFlags ClearErrors()
        {
            Errors = [];
            Send(); //send updates immediately

            return this;
        }

        private ErrorFlags Send()
        {
            _sendHandler?.Invoke(this);

            return this;
        }
    }
}