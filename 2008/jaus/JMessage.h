// -------------------------------------------------------------------------------------

// TODO: make a macro that will change these bitpacket structs if on a Big Endian system
class JMessage {
public:
	struct header;
	void *data;

private:
	JMessageType type; // this is in the header

protected:
	JMessage(JMessageType type) {
		this->type = type;
	}
	
	virtual ~JMessage() {
		// nothing
	}
	
	// TODO: implement
	//virtual void write(OutputStream& out);
};

// -------------------------------------------------------------------------------------

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

struct header {
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
}

// -------------------------------------------------------------------------------------
