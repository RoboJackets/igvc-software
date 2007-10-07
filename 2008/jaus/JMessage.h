// -------------------------------------------------------------------------------------

class JMessage {
private:
	JMessageType type;

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