/* Defines */
#define MAXIDLEN            100
#define MAXNUMREQUESTS      100

/* Structures */
enum Message_type
{
    MESSAGE_INVALID = -1,
    MESSAGE_UPDATE = 0,
    MESSAGE_FRIENDS,
    MESSAGE_HISTORY,
    MESSAGE_LEAVE,
    MESSAGE_CHECKID,
    MESSAGE_PING
};

struct Message
{
    enum Message_type type;
    int length;
    char *client_id;
    char *data;
};
