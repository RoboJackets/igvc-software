#include <stdlib.h>
#include <string.h>
//#include <sys/types.h>
//#include <netinet/in.h>

#include "playerc.h"
#include "error.h"

// Local declarations
void playerc_speech_recognition_putmsg(playerc_speechrecognition_t *device, player_msghdr_t *header,player_speech_recognition_data_t *data, size_t len);

// Create a new speech_recognition proxy
playerc_speechrecognition_t *playerc_speechrecognition_create(playerc_client_t *client, int index)
{
  playerc_speechrecognition_t *device;

  device = malloc(sizeof(playerc_speechrecognition_t));
  memset(device, 0, sizeof(playerc_speechrecognition_t));
  playerc_device_init(&device->info, client, PLAYER_SPEECH_RECOGNITION_CODE, index,
                      (playerc_putmsg_fn_t) playerc_speech_recognition_putmsg);
  return device;
}


// Destroy a speech_recognition proxy
void playerc_speechrecognition_destroy(playerc_speechrecognition_t *device)
{
  playerc_device_term(&device->info);
  free(device);
  return;
}

// Subscribe to the speech_recognition device
int playerc_speechrecognition_subscribe(playerc_speechrecognition_t *device, int access)
{
  return playerc_device_subscribe(&device->info, access);
}

// Un-subscribe from the speech_recognition device
int playerc_speechrecognition_unsubscribe(playerc_speechrecognition_t *device)
{
  return playerc_device_unsubscribe(&device->info);
}

void playerc_speech_recognition_putmsg(playerc_speechrecognition_t *device, player_msghdr_t *hdr, player_speech_recognition_data_t *buffer, size_t len)
{
  memset(device->words,0,30*20);
  player_speech_recognition_data_t *data = (player_speech_recognition_data_t*)buffer;

  if((hdr->type == PLAYER_MSGTYPE_DATA) && (hdr->subtype == PLAYER_SPEECH_RECOGNITION_DATA_STRING ))
  {
    char * str1;
    int i;
    device->wordCount = 0;
//    printf("data->text %s\n",data->text);

    for (str1 = strtok((*data).text, " ") ; str1 != NULL ; str1 = strtok(NULL, " ") )
    {
      for (i=0;i<strlen(str1);i++)
      {
//        printf("str1[%d]=%c",i,str1[i]);
//        printf("device->words[%i][%i]=%c\n",device->wordCount,i,device->words[device->wordCount][i]);
        device->words[device->wordCount][i]=str1[i];
      }
      device->wordCount++;
    }
  }
};

