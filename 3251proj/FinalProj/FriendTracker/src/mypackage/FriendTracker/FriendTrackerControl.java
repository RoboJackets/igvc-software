package mypackage.FriendTracker;

import android.app.Service;
import android.content.ComponentName;
import android.content.Intent;
import android.os.Binder;
import android.os.Bundle;
import android.os.IBinder;
import java.io.*;
import java.net.*;

public class FriendTrackerControl extends Service {

	Socket clientSock;
	String clientId;
	String serverIP;
	//FriendProvider mProvider;

	/**
	 * Class for clients to access.  Because we know this service always
	 * runs in the same process as its clients, we don't need to deal with
	 * IPC.
	 */
	public class LocalBinder extends Binder {
		FriendTrackerControl getService() {
			return FriendTrackerControl.this;
		}
	}

	// This is the object that receives interactions from clients.  See
	// RemoteService for a more complete example.
	private final IBinder mBinder = new LocalBinder();


	@Override
	public IBinder onBind(Intent intent) {
		return mBinder;
	}

	@Override
	public int onStartCommand(Intent intent, int flags, int startId) {
		//mProvider = new FriendProvider();
		// We want this service to continue running until it is explicitly
		// stopped, so return sticky.
		return START_STICKY;
	}

	public void login(Intent intent) throws IOException {
		//Bundle bundle = intent.getExtras();
		//clientId = bundle.getString("id");
		//serverIP = bundle.getString("ip");

		Intent i = new Intent("android.intent.action.MAIN");
		i.setFlags(Intent.FLAG_ACTIVITY_NEW_TASK);


		clientSock = new Socket("10.0.2.2", 25250);
		clientId = "test";
		
		//clientSock = new Socket(serverIP, 25250);
		ServerMessage msg = new ServerMessage(ServerMessage.MESSAGE_CHECKID, clientId, "");
		msg.send(clientSock);

		ServerMessage reply = ServerMessage.receive(clientSock);

		if(reply.getType() == ServerMessage.MESSAGE_IDAVAILABLE) {
			ComponentName n = new ComponentName("mypackage.FriendTracker", "mypackage.FriendTracker.FriendTracker");
			i.setComponent(n);
			startActivity(i); 
		}
		else {
			ComponentName n = new ComponentName("mypackage.FriendTracker", "mypackage.FriendTracker.LoginFailed");
			i.setComponent(n);
			startActivity(i);
		}
	}
	
	public void logout() throws IOException {
		if(clientSock != null && clientId != null) {
			ServerMessage msg = new ServerMessage(ServerMessage.MESSAGE_LEAVE, clientId, "");
			msg.send(clientSock);
		}
		clientSock = null;
		clientId = null;
	}

	public void lookupFriend(Intent intent) throws IOException {
		Bundle b = intent.getExtras();
		String id = b.getString("FriendId");
		ServerMessage msg = new ServerMessage(ServerMessage.MESSAGE_FRIENDS, clientId,  new String(id+"\n"));
		msg.send(clientSock);
		ServerMessage response = ServerMessage.receive(clientSock);
		
		//get the message and remove the id at the front of it and the \n at the end
		String message = response.getMessage();
		message = message.substring(message.indexOf(" ") + 1, message.length() - 1);
		FriendProvider.addFriend(id, message);
	}


	public void updateLocation(Intent intent) throws IOException {
		Bundle b = intent.getExtras();
		String lat = b.getString("Lat");
		String lon = b.getString("Lon");
		String data = lat + " " + lon + "\n";
		ServerMessage msg = new ServerMessage(ServerMessage.MESSAGE_UPDATE, clientId,  data);
		msg.send(clientSock);
		
		//Send the data to the provider
		FriendProvider.addFriend(clientId, data);
	}


	public void startHistory(Intent intent) throws IOException {
		Intent i = new Intent("android.intent.action.MAIN");
		i.setFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
		ComponentName n = new ComponentName("mypackage.FriendTracker", "mypackage.FriendTracker.ViewHistory");
		i.setComponent(n);
		String history = "No Locations";
		ServerMessage msg = new ServerMessage(ServerMessage.MESSAGE_HISTORY, clientId, "");
		msg.send(clientSock);
		ServerMessage response = ServerMessage.receive(clientSock);
		history = response.getMessage();
		i.putExtra("History", history);
		startActivity(i);
	}
}
