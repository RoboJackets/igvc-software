package mypackage.FriendTracker;

public class Message {
		public static final int MESSAGE_INVALID = -1;
		public static final int MESSAGE_UPDATE = 0;
		public static final int MESSAGE_FRIENDS = 1;
		public static final int MESSAGE_HISTORY = 2;
		public static final int MESSAGE_LEAVE = 3;
		public static final int MESSAGE_CHECKID = 4;
		public static final int MESSAGE_IDTAKEN = 5;
		public static final int MESSAGE_IDAVAILABLE = 6; 
		public static final int MESSAGE_PING = 7;
	
	private int type;
	private int id_length;
	private String clientId;
	private int length;
	private String message;
	
	public static Message receive() {
		return null;
	}
	
	public void send() {
	}
	
	public Message(int t, String id, String data) 
	{
		if(t > MESSAGE_INVALID && t <= MESSAGE_PING) {
			type = t;
		}
		else
			type = MESSAGE_INVALID;
		
		id_length = id.length();
		clientId = new String(id);
		length = data.length();
		message = new String(data);
	}
}
