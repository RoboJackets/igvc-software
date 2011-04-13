package mypackage.FriendTracker;

import android.app.Service;
import android.content.ComponentName;
import android.content.Context;
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
		Bundle bundle = intent.getExtras();
		String binder = bundle.getString("binder");

		if(binder.equalsIgnoreCase("LoginScreen"))
		{
			clientId = bundle.getString("id");
			serverIP = bundle.getString("ip");
			
			Intent i = new Intent("android.intent.action.MAIN");
			i.setFlags(Intent.FLAG_ACTIVITY_NEW_TASK);

			try {
				clientSock = new Socket(serverIP, 25250);
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
			} catch (UnknownHostException e) {
				ComponentName n = new ComponentName("mypackage.FriendTracker", "mypackage.FriendTracker.LoginFailed");
				i.setComponent(n);
				startActivity(i);
			//	e.printStackTrace();
			} catch (IOException e) {
				ComponentName n = new ComponentName("mypackage.FriendTracker", "mypackage.FriendTracker.LoginFailed");
				i.setComponent(n);
				startActivity(i);
			//	e.printStackTrace();
			}
		}
		return mBinder;
	}

	@Override
	public int onStartCommand(Intent intent, int flags, int startId) {
		// We want this service to continue running until it is explicitly
		// stopped, so return sticky.
		return START_STICKY;
	}
	
	public void startViewFriends(Intent intent) {
		
	}
	
	public void lookupFriend(Intent intent) throws IOException {
		Bundle b = intent.getExtras();
		String id = b.getString("FriendId");
		id += "\n";
		ServerMessage msg = new ServerMessage(ServerMessage.MESSAGE_FRIENDS, clientId,  id);
		msg.send(clientSock);
		ServerMessage response = ServerMessage.receive(clientSock);
		//TODO: add the response to the database
	}
	
	
}
