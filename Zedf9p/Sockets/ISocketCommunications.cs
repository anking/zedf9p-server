using Zedf9p.DTO;

namespace Zedf9p.Sockets
{
    public interface ISocketCommunications
    {
        void ConnectDataSockets();

        // Method to clean up and close the data sockets
        void CleanupSockets();

        // Other socket-related methods here, such as Send/Receive data
        //void SendSyncData(string data);

        void SendSyncData(SyncData syncData);

        void SendNmeaData(string data);

        void SendRtcmData(byte data);

        int ReceiveSyncData(byte[] syncInBuffer);

        bool IsSyncDataSocketConnected();

        bool IsRtcmDataSocketConnected();

        bool IsNmeaDataSocketConnected();
    }
}
