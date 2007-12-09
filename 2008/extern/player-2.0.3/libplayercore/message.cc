/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000
 *     Brian Gerkey, Kasper Stoy, Richard Vaughan, & Andrew Howard
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 * Desc: Message class and message queues
 * CVS:  $Id: message.cc,v 1.16 2006/03/17 21:22:05 gerkey Exp $
 * Author: Toby Collett - Jan 2005
 */

#include <pthread.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>

#include <libplayercore/message.h>
#include <libplayercore/player.h>
#include <libplayercore/error.h>

Message::Message(const struct player_msghdr & Header,
                 const void * data,
                 unsigned int data_size,
                 MessageQueue* _queue)
{
  this->Queue = _queue;
  this->Lock = new pthread_mutex_t;
  assert(this->Lock);
  pthread_mutex_init(this->Lock,NULL);
  this->Size = sizeof(struct player_msghdr)+data_size;
  assert(this->Size);
  this->Data = new unsigned char[this->Size];
  assert(this->Data);

  // copy the header and then the data into out message data buffer
  memcpy(this->Data,&Header,sizeof(struct player_msghdr));
  // Force header size to be same as data size
  ((player_msghdr *) Data)->size = data_size;

  memcpy(&this->Data[sizeof(struct player_msghdr)],data,data_size);
  this->RefCount = new unsigned int;
  assert(this->RefCount);
  *this->RefCount = 1;
}

Message::Message(const Message & rhs)
{
  assert(rhs.Lock);
  pthread_mutex_lock(rhs.Lock);

  assert(rhs.Data);
  assert(rhs.RefCount);
  assert(*(rhs.RefCount));
  Lock = rhs.Lock;
  Data = rhs.Data;
  Size = rhs.Size;
  Queue = rhs.Queue;
  RefCount = rhs.RefCount;
  (*RefCount)++;
  ready = false;

  pthread_mutex_unlock(Lock);
}

Message::~Message()
{
  this->DecRef();
}

bool
Message::Compare(Message &other)
{
  player_msghdr_t* thishdr = this->GetHeader();
  player_msghdr_t* otherhdr = other.GetHeader();
  return(Message::MatchMessage(thishdr,
                               otherhdr->type,
                               otherhdr->subtype,
                               otherhdr->addr));
};

void
Message::DecRef()
{
  pthread_mutex_lock(Lock);
  (*RefCount)--;
  assert((*RefCount) >= 0);
  if((*RefCount)==0)
  {
    delete [] Data;
    delete RefCount;
    pthread_mutex_unlock(Lock);
    pthread_mutex_destroy(Lock);
    delete Lock;
  }
  else
    pthread_mutex_unlock(Lock);
}

MessageQueueElement::MessageQueueElement()
{
  msg = NULL;
  prev = next = NULL;
}

MessageQueueElement::~MessageQueueElement()
{
}

MessageQueue::MessageQueue(bool _Replace, size_t _Maxlen)
{
  this->Replace = _Replace;
  this->Maxlen = _Maxlen;
  this->head = this->tail = NULL;
  this->Length = 0;
  pthread_mutex_init(&this->lock,NULL);
  pthread_mutex_init(&this->condMutex,NULL);
  pthread_cond_init(&this->cond,NULL);
  this->ClearFilter();
  this->filter_on = false;
  this->replaceRules = NULL;
  this->pull = false;
}

MessageQueue::~MessageQueue()
{
  // clear the queue
  Message* msg;
  while((msg = this->Pop()))
    delete msg;

  // clear the list of replacement rules
  MessageReplaceRule* tmp;
  MessageReplaceRule* curr = this->replaceRules;
  while(curr)
  {
    tmp = curr->next;
    delete curr;
    curr = tmp;
  }

  pthread_mutex_destroy(&this->lock);
  pthread_mutex_destroy(&this->condMutex);
  pthread_cond_destroy(&this->cond);
}

/// @brief Add a replacement rule to the list
void
MessageQueue::AddReplaceRule(int _host, int _robot, int _interf, int _index,
                             int _type, int _subtype, bool _replace)
{
  MessageReplaceRule* curr;
  for(curr=this->replaceRules;curr;curr=curr->next)
  {
    // Check for an existing rule with the same criteria; replace if found
    if (curr->Equivalent (_host, _robot, _interf, _index, _type, _subtype))
    {
      curr->replace = _replace;
      return;
    }
	if (curr->next == NULL)
		// Break before for() increments if this is the last one in the list
		break;
  }
  if(!curr)
  {
    curr = replaceRules = new MessageReplaceRule(_host, _robot, _interf, _index,
                                  _type, _subtype, _replace);
//	assert(curr);  // This is bad. What happens if there's a memory allocation failure when not built with debug?
    if (!curr)
      PLAYER_ERROR ("memory allocation failure; could not add new replace rule");
  }
  else
  {
    curr->next = new MessageReplaceRule(_host, _robot, _interf, _index,
                                        _type, _subtype, _replace);
//	assert(curr->next);  // This is bad. What happens if there's a memory allocation failure when not built with debug?
    if (!curr->next)
      PLAYER_ERROR ("memory allocation failure; could not add new replace rule");
  }
}

/// @brief Add a replacement rule to the list
void
MessageQueue::AddReplaceRule(const player_devaddr_t &device,
                             int _type, int _subtype, bool _replace)
{
  this->AddReplaceRule (device.host, device.robot, device.interf, device.index,
                        _type, _subtype, _replace);
}

bool
MessageQueue::CheckReplace(player_msghdr_t* hdr)
{
  // First look through the replacement rules
  for(MessageReplaceRule* curr=this->replaceRules;curr;curr=curr->next)
  {
    if(curr->Match(hdr))
      return(curr->replace);
  }

  // Didn't find it; follow the default rule

  // Don't replace config requests or replies
  if((hdr->type == PLAYER_MSGTYPE_REQ) ||
     (hdr->type == PLAYER_MSGTYPE_RESP_ACK) ||
     (hdr->type == PLAYER_MSGTYPE_RESP_NACK) ||
     (hdr->type == PLAYER_MSGTYPE_SYNCH))
    return(false);
  // Replace data and command according to the this->Replace flag
  else if((hdr->type == PLAYER_MSGTYPE_DATA) ||
          (hdr->type == PLAYER_MSGTYPE_CMD))
    return(this->Replace);
  else
  {
    PLAYER_ERROR1("encountered unknown message type %u", hdr->type);
    return(false);
  }
}

// Waits on the condition variable associated with this queue.
void
MessageQueue::Wait(void)
{
  MessageQueueElement* el;

  // don't wait if there's data on the queue
  this->Lock();
  // start at the head and traverse the queue until a filter-friendly
  // message is found
  for(el = this->head; el; el = el->next)
  {
    if(!this->filter_on || this->Filter(*el->msg))
      break;
  }
  this->Unlock();
  if(el)
    return;

  // need to push this cleanup function, cause if a thread is cancelled while
  // in pthread_cond_wait(), it will immediately relock the mutex.  thus we
  // need to unlock ourselves before exiting.
  pthread_cleanup_push((void(*)(void*))pthread_mutex_unlock,
                       (void*)&this->condMutex);
  pthread_mutex_lock(&this->condMutex);
  pthread_cond_wait(&this->cond,&this->condMutex);
  pthread_mutex_unlock(&this->condMutex);
  pthread_cleanup_pop(0);
}

bool
MessageQueue::Filter(Message& msg)
{
  player_msghdr_t* hdr = msg.GetHeader();
  return(((this->filter_host < 0) ||
          ((unsigned int)this->filter_host == hdr->addr.host)) &&
         ((this->filter_robot < 0) ||
          ((unsigned int)this->filter_robot == hdr->addr.robot)) &&
         ((this->filter_interf < 0) ||
          ((unsigned int)this->filter_interf == hdr->addr.interf)) &&
         ((this->filter_index < 0) ||
          ((unsigned int)this->filter_index == hdr->addr.index)) &&
         (((this->filter_type < 0) &&
           ((hdr->type == PLAYER_MSGTYPE_RESP_ACK) ||
            (hdr->type == PLAYER_MSGTYPE_RESP_NACK))) ||
          ((unsigned int)this->filter_type == hdr->type)) &&
         ((this->filter_subtype < 0) ||
          ((unsigned int)this->filter_subtype == hdr->subtype)));
}

void
MessageQueue::SetFilter(int host, int robot, int interf,
                        int index, int type, int subtype)
{
  this->filter_host = host;
  this->filter_robot = robot;
  this->filter_interf = interf;
  this->filter_index = index;
  this->filter_type = type;
  this->filter_subtype = subtype;
  this->filter_on = true;
}

size_t 
MessageQueue::GetLength(void)
{
  size_t len;
  this->Lock();
  len = this->Length;
  this->Unlock();
  return(len);
}

void MessageQueue::MarkAllReady (void)
{
  MessageQueueElement *current;

  if (!pull)
    return;   // No need to mark ready if not in pull mode

  Lock();
  // Mark all messages in the queue ready
  for (current = head; current != NULL; current = current->next)
  {
    current->msg->SetReady ();
  }
  Unlock ();
  // Push a sync message onto the end
  struct player_msghdr syncHeader;
  syncHeader.addr.host = 0;
  syncHeader.addr.robot = 0;
  syncHeader.addr.interf = PLAYER_PLAYER_CODE;
  syncHeader.addr.index = 0;
  syncHeader.type = PLAYER_MSGTYPE_SYNCH;
  syncHeader.subtype = 0;
  Message syncMessage (syncHeader, 0, 0);
  syncMessage.SetReady ();
  Push (syncMessage, true);
}


void
MessageQueue::ClearFilter(void)
{
  this->filter_on = false;
}


// Signal that new data is available (calls pthread_cond_broadcast()
// on this device's condition variable, which will release other
// devices that are waiting on this one).
void
MessageQueue::DataAvailable(void)
{
  pthread_mutex_lock(&this->condMutex);
  pthread_cond_broadcast(&this->cond);
  pthread_mutex_unlock(&this->condMutex);
}

MessageQueueElement*
MessageQueue::Push(Message & msg, bool UseReserved)
{
  player_msghdr_t* hdr;

  assert(*msg.RefCount);
  this->Lock();
  hdr = msg.GetHeader();
  // Should we try to replace an older message of the same signature?
  if(this->CheckReplace(hdr))
  {
    for(MessageQueueElement* el = this->tail;
        el != NULL;
        el = el->prev)
    {
      // Ignore ready flag outside pull mode - when a client goes to pull mode, only the client's
      // queue is set to pull, so other queues will still ignore ready flag
      if(el->msg->Compare(msg) && (!el->msg->Ready () || !pull))
      {
        this->Remove(el);
        // DAVID: FIX: delete the message
        delete el->msg;
        delete el;
        break;
      }
    }
  }
  // Are we over the limit?
  if(this->Length >= this->Maxlen - (UseReserved ? 0 : 1))
  {
    PLAYER_WARN("tried to push onto a full message queue");
    this->Unlock();
    if(!this->filter_on)
      this->DataAvailable();
    return(NULL);
  }
  else
  {
    MessageQueueElement* newelt = new MessageQueueElement();
    newelt->msg = new Message(msg);
    if (!pull || newelt->msg->GetHeader ()->type != PLAYER_MSGTYPE_DATA)
    {
      // If not in pull mode, or message is not data, set ready to true immediatly
      newelt->msg->SetReady ();
    }
    if(!this->tail)
    {
      this->head = this->tail = newelt;
      newelt->prev = newelt->next = NULL;
    }
    else
    {
      this->tail->next = newelt;
      newelt->prev = this->tail;
      newelt->next = NULL;
      this->tail = newelt;
    }
    this->Length++;
    this->Unlock();
    if(!this->filter_on || this->Filter(msg))
      this->DataAvailable();
    return(newelt);
  }
}

Message*
MessageQueue::Pop()
{
  MessageQueueElement* el;
  Lock();
  if(this->Empty())
  {
    Unlock();
    return(NULL);
  }

  // start at the head and traverse the queue until a filter-friendly
  // message is found
  for(el = this->head; el; el = el->next)
  {
    if(!this->filter_on || this->Filter(*el->msg))
    {
      this->Remove(el);
      Unlock();
      Message* retmsg = el->msg;
      delete el;
      return(retmsg);
    }
  }
  Unlock();
  return(NULL);
}

Message* MessageQueue::PopReady (void)
{
  MessageQueueElement* el;
  Lock();
  if(this->Empty())
  {
    Unlock();
    return(NULL);
  }

  // start at the head and traverse the queue until a filter-friendly
  // message (that is marked ready if in pull mode) is found
  for(el = this->head; el; el = el->next)
  {
    if((!this->filter_on || this->Filter(*el->msg)) && ((pull && el->msg->Ready ()) || !pull))
    {
      this->Remove(el);
      Unlock();
      Message* retmsg = el->msg;
      delete el;
      return(retmsg);
    }
  }
  Unlock();
  return(NULL);
}

void
MessageQueue::Remove(MessageQueueElement* el)
{
  if(el->prev)
    el->prev->next = el->next;
  else
    this->head = el->next;
  if(el->next)
    el->next->prev = el->prev;
  else
    this->tail = el->prev;
  this->Length--;
}

