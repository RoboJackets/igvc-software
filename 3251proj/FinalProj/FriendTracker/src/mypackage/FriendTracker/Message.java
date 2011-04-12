package mypackage.FriendTracker;

import java.io.BufferedReader;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.Socket;

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
	
	
	public static Message receive(Socket clientSock) throws IOException {
		BufferedReader inStream = new BufferedReader(new InputStreamReader(clientSock.getInputStream()));
		Message message = new Message();
		String input;
		
		//Read the type
		input = readNextField(inStream, 9);
		if(input == null) {
			message.type = MESSAGE_INVALID;
			return message;
		}
		
		message.type = Integer.parseInt(input);
		
		//Read the Id length
		input = readNextField(inStream, 9);
		if(input == null) {
			message.type = MESSAGE_INVALID;
			return message;
		}
		message.id_length = Integer.parseInt(input);
		
		//Read the id
		input = readNextField(inStream, message.id_length);
		if(input == null) {
			message.type = MESSAGE_INVALID;
			return message;
		}
		message.clientId = new String(input);
		
		//Read the data length 
		input = readNextField(inStream, 9);
		if(input == null) {
			message.type = MESSAGE_INVALID;
			return message;
		}
		message.length = Integer.parseInt(input);
		
		//Read the data
		if(message.length > 0) {
			input = readNextField(inStream, message.length);
			if(input == null) {
				message.type = MESSAGE_INVALID;
				return message;
			}
			message.message = new String(input);
		}
		
		return message;
	}
	
	private static String readNextField(BufferedReader inStream, int size) throws IOException {
		int count = 0;
		int c = 0;
		String input = "";
		
		while(count < size) {
			if((c = inStream.read()) != -1) {
				input += c;
				count++;
			}
			else {
				return null;
			}
		}
		
		return input;
	}
	
	public void send(Socket clientSock) throws IOException {
		String typeStr = String.valueOf(type);
		String id_lenStr = String.valueOf(id_length);
		String lenStr = String.valueOf(length);
		
		DataOutputStream outStream = new DataOutputStream(clientSock.getOutputStream());
		
		//Pad the integer Strings to be 9 chars
		while(typeStr.length() < 9) {typeStr = '0' + typeStr;}
		while(id_lenStr.length() < 9) {id_lenStr = '0' + id_lenStr;}
		while(lenStr.length() < 9) {lenStr = '0' + lenStr;}
		
		outStream.writeBytes(typeStr);
		outStream.writeBytes(id_lenStr);
		outStream.writeBytes(clientId);
		outStream.writeBytes(lenStr);
		if(length > 0) {
			outStream.writeBytes(message);
		}
	}
	
	public Message(int t, String id, String data) {
		if(t > MESSAGE_INVALID && t <= MESSAGE_PING) {
			type = t;
		}
		else {
			type = MESSAGE_INVALID;
		}
		
		id_length = id.length();
		clientId = new String(id);
		length = data.length();
		message = new String(data);
	}
	
	public Message() {
		type = MESSAGE_INVALID;
		id_length = -1;
		clientId = null;
		length = -1;
		message = null;
	}
	
	public int getType() {return type;}
	public int getIdLen() {return id_length;}
	public String getId() {return clientId;}
	public int getLength() {return length;}
	public String getMessage() {return message;}
}
