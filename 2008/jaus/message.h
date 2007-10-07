#ifndef MESSAGE_H
#define MESSAGE_H

//list of structs for all the different messages that have data

//union of messageproperties

class message {
	struct header;
	data;
	message(jushort messagecode);//for message with no data
	message(jushort messagecode, Jbyte* data);
}

struct header {
	Jushort MessageProperties;//page 13
	Jushort CommandCode;//a two byte command code
	Jbyte DestInstanceID;//sec 3.4
	Jbyte DestComponentID;
	Jbyte DestNodeID;
	Jbyte DestSubsysID;
	Jbyte SourceInstanceID;
	Jbyte SourceComponentID;
	Jbyte SourceNodeID;
	Jbyte SourceSubsystemID;
	Jushort DataControl;//two bit fields -- 0-11 size 12-15 data flags
	Jushort SequenceNumber;//number of message in serial connection, 0-65535
}

/*message properties bits
0-3 priority (0 = low, 6 = default, 11 = high; 12 = Lowest priority safety critical message, 15 = highest priority safety critical message
4-5 ACK/NAK (origninator: 0 = no response needed, 1 = responce needed;  responder: 2 = message negitive aknoldge, 3 - message ok;  messages needing responce must have a sequence number
6 message is service connection -- see 3.6
7 experimental message -- command code must be in range 0xD000 - 0xFFFF see 3.7.2
8-13 message vesion (0 = 2.0/2.1 compat; 1 = 3.0/3.1 compat; 2 = 3.2/3.3 compat)
14-15 reserved - set to 0
*/

//command codes are in jauscodes2

//ids are XXX:XXX:XXLowest priority safety critical messageX:XXX -- (SubcomponentID):(nodeID):(componentID):(InstanceID) -- value range 0-255 inclusive, 0 invalid, 255 broadcast

/*data control
0-11 data size (0-4080 - number of bytes sent, excluding header (total limit in trnasaction is 4096) see 3.5
12-15 data flags (0000 = only data  packet in single stream, 0001 - first of multi, 0100 - retansmitted data packet, 1000 last data packet)
*/

/*
service connections -- depereciate in v4.0
send inform or command messages
never qued - run immediatly or stored and overwritten with most recent
*/

#endif //MESSAGE_H
