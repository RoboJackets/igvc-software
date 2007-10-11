// -------------------------------------------------------------------------------------

/**
 * Base class for all types of JAUS messages.
 */
class JMessage {
private:
	JMessageType type; // this is in the header

protected:
	JMessage(JMessageType type) {
		this->type = type;
	}
	
	virtual ~JMessage() {
		// nothing
	}
	
public:
	// TODO: implement
	//virtual void write(OutputStream& out);
};

// -------------------------------------------------------------------------------------

/**
 * Class for all JAUS messages that have no extra arguments
 * (i.e., the message only consists of a type code).
 */
class JSimpleMessage : public JMessage {
public:
	JSimpleMessage(JMessageType type) : JMessage(type) {
		// nothing
	}
};

// -------------------------------------------------------------------------------------

class RunMission : public JMessage {
private:
	Jushort missionID;
	
public:
	RunMission(Jushort missionID) : JMessage(JMT_RUN_MISSION) {
		this->missionID = missionID;
	}
}; 

// -------------------------------------------------------------------------------------

/**
 * Header of a JAUS message that can be transmitted.
 */
// TODO: make a macro that will change these bitpacket structs if on a Big Endian system
typedef struct {
	union {
		struct {
			Jushort priority:4; 
			Jushort ackNak:2;
			Jushort scFlag:1; 
			Jushort expFlag:1;
			Jushort version:6; 
			Jushort reserved:2;
		}
		Jushort messageProperties;
	}
	Jushort commandCode;
	Juint destAddress
	Juint sourceAddress
	union {
		struct {
			Jushort dataSize:12;
			Jushort dataFlag:4;
		}
		Jushort dataControl;
	}
	Jushort sequenceNumber;//number of message in serial connection, 0-65535
} jmessage_header;

// -------------------------------------------------------------------------------------
