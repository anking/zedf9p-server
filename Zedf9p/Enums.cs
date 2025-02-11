namespace Zedf9p.Enums
{
    public enum SyncIncomingCommandType
    {
        UNDEFINED = 0,
        RESTART_SURVEY = 1,
        //START_SERVER = 2,
        //START_CLIENT = 3,
        RESTART_FIXED = 4,
    }

    public enum OperationMode
    {
        Idle = 0,          //setting driver into this mode will run it listening to UI commands and sending GPS data back if available(in case NTRIP connection fails)
        Station = 1,        //if set to true program will configure gps as NTRIP server
        Client = 2         //if set to true program will configure gps as NTRIP client              
    }

    public enum ReceiverMode
    {
        SurveyIn = 1,       //Survey Mode
        Fixed = 2           //Fixed Mode         
    }
}