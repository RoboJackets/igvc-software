package mypackage.FriendTracker;

import android.app.Service;
import android.content.ComponentName;
import android.content.Intent;
import android.os.Bundle;
import android.os.IBinder;
import java.io.*;
import java.net.*;

public class FriendTrackerControl extends Service {

	Socket clientSock;
	String clientId;
	String serverIP;
	@Override
	public IBinder onBind(Intent intent) {
		//Bundle bundle = intent.getExtras();
		//String binder = bundle.getString("binder");

	//	if(binder.equalsIgnoreCase("LoginScreen"))
	//	{
	//		clientId = bundle.getString("id");
		//	serverIP = bundle.getString("ip");

		/*	try {
				clientSock = new Socket(serverIP, 25250);
				Message msg = new Message(Message.MESSAGE_CHECKID, clientId, "");
				msg.send(clientSock);

				Message reply = Message.receive(clientSock);

				if(reply.getType() == Message.MESSAGE_IDAVAILABLE) {
		*/			Intent i = new Intent("android.intent.action.MAIN");
					ComponentName n = new ComponentName("mypackage.FriendTracker", "mypackage.FriendTracker.FriendTracker");
					i.setComponent(n);
					startActivity(i); 
		/*		}
				else {
					Intent i = new Intent("android.intent.action.MAIN");
					ComponentName n = new ComponentName("mypackage.FriendTracker", "mypackage.FriendTracker.LoginFailed");
					i.setComponent(n);
					startActivity(i);
				}
			} catch (UnknownHostException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}*/
		//}
		return null;
	}

	@Override
	public int onStartCommand(Intent intent, int flags, int startId) {
		// We want this service to continue running until it is explicitly
		// stopped, so return sticky.
		return START_STICKY;
	}
}
