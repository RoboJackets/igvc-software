/**
 * JAUS Message Types
 * 
 * @see The JAUS Reference Architecture Specification v3.3 §2
 */
typedef enum {
	// ### COMMAND CLASS ###
	
	/* Core Subgroup: 0001-01FF */
	JMT_SET_COMPONENT_AUTHORITY		= 0x0001;
	JMT_SHUTDOWN					= 0x0002;
	// ...
	
	/* Event Setup and Control: 01F0-01FF */
	JMT_CREATE_EVENT				= 0x01F0;
	JMT_UPDATE_EVENT				= 0x01F1;
	// ...
	
	// ... (more command subgroups)
	
	// ### QUERY CLASS ###
	
	/* Core Subgroup: 2000-21FF */
	JMT_QUERY_COMPONENT_AUTHORITY	= 0x2001;
	JMT_QUERY_COMPONENT_STATUS		= 0x2002;
	// ...
	
	
} JMessageType;


typedef Jbyte unsigned char;	// Byte
typedef Jshort signed short;	// Short Integer
typedef Jint signed int;	// Integer
typedef Jlong long;		// Long Int
typedef Jushort unsigned short; // Unsigned Short Int
typedef Juint unsigned int;	// Unsigned Int
typedef Julong unsigned long;	// Unsigned Long
typedef Jfloat float;		// Float
typedef Jdouble double;		// Long Float
