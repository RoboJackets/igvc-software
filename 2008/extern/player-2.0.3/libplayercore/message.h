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
 * CVS:  $Id: message.h,v 1.16 2006/03/17 21:22:05 gerkey Exp $
 * Author: Toby Collett - Jan 2005
 */

#ifndef MESSAGE_H
#define MESSAGE_H

#include <pthread.h>

#include <libplayercore/player.h>

// TODO:
//   - Add support for stealing the data pointer when creating a message.
//     That would save one allocation & copy in some situations.

class MessageQueue;

/** @brief Reference-counted message objects

All Player messages are transferred between drivers as pointers to Message
objects.  These objects are reference-counted so that messages can be
delivered to multiple recipients with minimal memory overhead.

Messages are not usually manipulated directly in driver code.  The details
of allocating, filling, parsing, and deleting Message objects are handled
by the Driver and MessageQueue classes.

The only method of interest to driver authors is the helper MatchMessage(),
which can be used in a Driver::ProcessMessage method to determine if a
message header matches a given signature.
*/
class Message
{
  public:
    /// Create a new message.
    Message(const struct player_msghdr & Header,
            const void* data,
            unsigned int data_size,
            MessageQueue* _queue = NULL);
    /// Copy pointers from existing message and increment refcount.
    Message(const Message & rhs);

    /// Destroy message, dec ref counts and delete data if ref count == 0
    ~Message();

    /** @brief Helper for message processing.

    Returns true if @p hdr matches the supplied @p type, @p subtype,
    and @p addr.  Set type and/or subtype to -1 for don't care.
    */
    static bool MatchMessage(player_msghdr_t* hdr,
                             int type,
                             int subtype,
                             player_devaddr_t addr)
    {
      return(((type < 0) || (hdr->type == (uint8_t)type)) &&
             ((subtype < 0) || (hdr->subtype == (uint8_t)subtype)) &&
             (hdr->addr.host == addr.host) &&
             (hdr->addr.robot == addr.robot) &&
             (hdr->addr.interf == addr.interf) &&
             (hdr->addr.index == addr.index));
    }

    /// GetData from message.
    void* GetData() {return (void*)Data;};
    /// Get pointer to header.
    player_msghdr_t * GetHeader() {return reinterpret_cast<player_msghdr_t *> (Data);};
    /// Get pointer to payload.
    void* GetPayload() {return (void*)(&Data[sizeof(player_msghdr_t)]);};
    /// Get Payload size.
    size_t GetPayloadSize() {return Size - sizeof(player_msghdr_t);};
    /// Size of message data.
    unsigned int GetSize() {return Size;};
    /// Compare type, subtype, device, and device_index.
    bool Compare(Message &other);
    /// Decrement ref count
    void DecRef();
    /// Set ready to send
    void SetReady ()            { ready = true; }
    /// Check if ready to send
		bool Ready (void) const     { return ready; }

    /// queue to which any response to this message should be directed
    MessageQueue* Queue;

    /// Reference count.
    unsigned int * RefCount;

  private:
    /// Pointer to the message data.
    uint8_t * Data;
    /// Length (in bytes) of Data.
    unsigned int Size;
    /// Used to lock access to Data.
    pthread_mutex_t * Lock;
    /// Marks if the message is ready to be sent to the client
    bool ready;
};

/**
 This class is a helper for maintaining doubly-linked queues of Messages.
*/
class MessageQueueElement
{
  public:
    /// Create a queue element with NULL prev and next pointers.
    MessageQueueElement();
    /// Destroy a queue element.
    ~MessageQueueElement();

    /// The message stored in this queue element.
    Message* msg;
  private:
    /// Pointer to previous queue element.
    MessageQueueElement * prev;
    /// Pointer to next queue element.
    MessageQueueElement * next;

    friend class MessageQueue;
};

/** We keep a singly-linked list of (addr,type,subtype,replace) tuples.
 * When a new message comes in, we check its (addr,type,subtype) signature
 * against this list to find a replace rule.  If its not in the list, the
 * default rule is used: never replace config requests or replies, replace
 * data and command msgs if the Replace flag is set.
 */
class MessageReplaceRule
{
  private:
    // The address to match (not using a player_devaddr_t so that we can
    // use -1 to indicate don't care)
    int host, robot, interf, index;
    // The type and subtype to match (-1 is don't care)
    int type, subtype;
  public:
    MessageReplaceRule(int _host, int _robot, int _interf, int _index,
                       int _type, int _subtype, bool _replace) :
            host(_host), robot(_robot), interf(_interf), index(_index),
            type(_type), subtype(_subtype), replace(_replace), next(NULL) {}

    bool Match(player_msghdr_t* hdr)
    {
      return(((this->host < 0) ||
              ((uint32_t)this->host == hdr->addr.host)) &&
             ((this->robot < 0) ||
              ((uint32_t)this->robot == hdr->addr.robot)) &&
             ((this->interf < 0) ||
              ((uint16_t)this->interf == hdr->addr.interf)) &&
             ((this->index < 0) ||
              ((uint16_t)this->index == hdr->addr.index)) &&
             ((this->type < 0) ||
              ((uint8_t)this->type == hdr->type)) &&
             ((this->subtype < 0) ||
              ((uint8_t)this->subtype == hdr->subtype)));
    }

    bool Equivalent (int _host, int _robot, int _interf, int _index, int _type, int _subtype)
    {
      return (host == _host && robot == _robot && _interf && index == _index &&
          type == _type && subtype == _subtype);
    }

    // To replace, or not to replace
    // That is the question
    bool replace;
    // Next rule in the list
    MessageReplaceRule* next;
};

/** @brief A doubly-linked queue of messages.

Player Message objects are delivered by being pushed on and popped off
queues of this type.  Importantly, every driver has one, Driver::InQueue.
All messages sent to the driver arrive on this queue.  However, the driver
rarely pops messages off its queue directly; instead the driver calls
Driver::ProcessMessages, which hands off each incoming message to
Driver::ProcessMessage (which the driver has presumably re-implemented).

Every queue has a maximum length that is determined when it is created; for
drivers this happens in the constructor Driver::Driver.  This length
determines the maximum number of messages that can be pushed onto the
queue.  Since messages vary greatly in size, there is not a direct
correspondence between the length of a queue and the memory that it
occupies.  Furthermore, since messages are reference counted and may be
shared across multiple queues, it is not necessarily meaningful to consider
how much memory a given queue "occupies."

The queue supports configurable message replacement.  This functionality is
useful when, for example, a driver wants new incoming commands to overwrite
old ones, rather than to have them queue up.  To decide whether an incoming
message should replace an existing message that has the same address
(host:robot:interface:index), type, and subtype, the following logic is
applied:

- If a matching replacement rule was set via AddReplaceRule(), that rule is
  followed.
- Else:
  - If the message type is PLAYER_MSGTYPE_REQ, PLAYER_MSGTYPE_RESP_ACK, or
    PLAYER_MSGTYPE_RESP_NACK, the message is not replaced.
  - Else:
    - If MessageQueue::Replace is false (it is set in the constructor and
      can be changed via SetReplace()), the message is not replaced.
    - Else:
      - The message is replaced.

Most drivers can simply choose true or false in their constructors (this
setting is passed on to the MessageQueue constructor).  However, drivers
that support multiple interfaces may use AddReplaceRule() to establish
different replacement rules for messages that arrive for different
interfaces.  For example, the @ref driver_p2os driver has incoming commands
to the wheelmotors overwrite each other, but queues up commands to the
manipulator arm.

The queue also supports filtering based on device address.  After
SetFilter() is called, Pop() will only return messages that match the given
filter.  Use ClearFilter() to return to normal operation.  This filter is
not usually manipulated directly in driver code; it's main use inside
Device::Request.

*/
class MessageQueue
{
  public:
    /// Create an empty message queue.
    MessageQueue(bool _Replace, size_t _Maxlen);
    /// Destroy a message queue.
    ~MessageQueue();
    /// Check whether a queue is empty
    bool Empty() { return(this->head == NULL); }
    /** Push a message onto the queue.  Returns a pointer to the new last
    element in the queue. UseReserved should only be set true when pushing sync
    messages on to the queue. If UseReserved is false then a single message slot
    is reserved on the queue for a sync message */
    MessageQueueElement * Push(Message& msg, bool UseReserved = false);
    /** Pop a message off the queue.
    Pop the head (i.e., the first-inserted) message from the queue.
    Returns pointer to said message, or NULL if the queue is empty */
    Message* Pop();
    /** Pop a ready message off the queue.
    Pop the head (i.e., the first-inserted) message from the queue.
    If pull_flag is true, only pop messages marked as ready.
    Returns pointer to said message, or NULL if the queue is empty */
    Message* PopReady (void);
    /** Set the @p Replace flag, which governs whether data and command
    messages of the same subtype from the same device are replaced in
    the queue. */
    void SetReplace(bool _Replace) { this->Replace = _Replace; };
    /** Add a replacement rule to the list.  The first 6 arguments
     * determine the signature that a message will have to match in order
     * for this rule to be applied.  If an incoming message matches this
     * signature, the last argument determines whether replacement will
     * occur.  Set any of the first 6 arguments to -1 to indicate don't
     * care. */
    void AddReplaceRule(int _host, int _robot, int _interf, int _index,
                        int _type, int _subtype, bool _replace);
    /** Add a replacement rule to the list.  Use this version if you
     * already have the device address assembled in a player_devaddr_t
     * structure.  The tradeoff is that you cannot use -1 to indicate don't
     * care for those values (the fields in that structure are unsigned).
     * */
    void AddReplaceRule(const player_devaddr_t &device,
                        int _type, int _subtype, bool _replace);
    /// @brief Check whether a message with the given header should replace
    /// any existing message of the same signature.
    bool CheckReplace(player_msghdr_t* hdr);
    /** Wait on this queue.  This method blocks until new data is available
    (as indicated by a call to DataAvailable()). */
    void Wait(void);
    /** Signal that new data is available.  Calling this method will
     release any threads currently waiting on this queue. */
    void DataAvailable(void);
    /// @brief Check whether a message passes the current filter.
    bool Filter(Message& msg);
    /// @brief Clear (i.e., turn off) message filter.
    void ClearFilter(void);
    /// @brief Set filter values
    void SetFilter(int host, int robot, int interf, int index,
                   int type, int subtype);
    /** Set the @p pull flag, which if true then requires messages to be marked
    as ready before they will be sent to the client. */
    void SetPull (bool _pull) { this->pull = _pull; }
    /// Mark all messages in the queue as ready to be sent
    void MarkAllReady (void);
    
    /// @brief Get current length of queue, in elements.
    size_t GetLength(void);
  private:
    /// @brief Lock the mutex associated with this queue.
    void Lock() {pthread_mutex_lock(&lock);};
    /// @brief Unlock the mutex associated with this queue.
    void Unlock() {pthread_mutex_unlock(&lock);};
    /** Remove element @p el from the queue, and rearrange pointers
    appropriately. */
    void Remove(MessageQueueElement* el);
    /// @brief Head of the queue.
    MessageQueueElement* head;
    /// @brief Tail of the queue.
    MessageQueueElement* tail;
    /// @brief Mutex to control access to the queue, via Lock() and Unlock().
    pthread_mutex_t lock;
    /// @brief Maximum length of queue in elements.
    size_t Maxlen;
    /// @brief Singly-linked list of replacement rules
    MessageReplaceRule* replaceRules;
    /// @brief When a (data or command) message doesn't match a rule in
    /// replaceRules, should we replace it?
    bool Replace;
    /// @brief Current length of queue, in elements.
    size_t Length;
    /// @brief A condition variable that can be used to signal, via
    /// DataAvailable(), other threads that are Wait()ing on this
    /// queue.
    pthread_cond_t cond;
    /// @brief Mutex to go with condition variable cond.
    pthread_mutex_t condMutex;
    /// @brief Current filter values
    bool filter_on;
    int filter_host, filter_robot, filter_interf,
        filter_index, filter_type, filter_subtype;
    /// @brief Flag for if in pull mode. If false, push mode. Push is default mode,
    /// but pull is the recommended method to avoid getting delays in data on the client.
    bool pull;
};

#endif
