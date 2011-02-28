/* Defines */
#define MAXIDLEN            40
#define MAXNUMREQUESTS      100
#define SENDBUFSIZE         5000
#define RECVBUFSIZE         6000

/* Structures */
enum Message_type
{
    MESSAGE_INVALID = -1,
    MESSAGE_UPDATE = 0,
    MESSAGE_FRIENDS,
    MESSAGE_HISTORY,
    MESSAGE_LEAVE,
    MESSAGE_CHECKID,
    MESSAGE_IDTAKEN,                 //server message
    MESSAGE_IDAVAILABLE,             //server message
    MESSAGE_PING
};

typedef struct Message
{
    enum Message_type type;
    int id_len;
    int length;                 //In bytes
    char *client_id;
    char *data;
} Message;

/* Typedefs */
