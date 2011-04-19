package mypackage.FriendTracker;

import android.app.Service;
import android.content.ComponentName;
import android.content.ContentValues;
import android.content.Intent;
import android.database.Cursor;
import android.net.Uri;
import android.os.Binder;
import android.os.Bundle;
import android.os.CountDownTimer;
import android.os.IBinder;
import android.widget.Toast;

import java.io.*;
import java.net.*;
import java.util.ArrayList;
import mypackage.FriendTracker.Friend.Friends;

public class FriendTrackerControl extends Service {

	Socket clientSock;
	String clientId;
	String serverIP;
	static String myHistory;
	CountDownTimer updateTimer = new CountDownTimer(40000, 5000) {

	     public void onTick(long millisUntilFinished) {
	         try {
				updateDatabase();
			} catch (Exception e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
	     }

	     //Restart the timer
	     public void onFinish() {
	    	 this.start();
	     }
	  };

	/**
	 * Class for clients to access. Because we know this service always runs in
	 * the same process as its clients, we don't need to deal with IPC.
	 */
	public class LocalBinder extends Binder {
		FriendTrackerControl getService() {
			return FriendTrackerControl.this;
		}
	}

	// This is the object that receives interactions from clients. See
	// RemoteService for a more complete example.
	private final IBinder mBinder = new LocalBinder();

	@Override
	public IBinder onBind(Intent intent) {
		return mBinder;
	}

	@Override
	public int onStartCommand(Intent intent, int flags, int startId) {
		//		myListener = new myActionListener();
		// mProvider = new FriendProvider();
		// We want this service to continue running until it is explicitly
		// stopped, so return sticky.
		return START_STICKY;
	}

	public void login(Intent intent) throws Exception {
		Bundle bundle = intent.getExtras();
		clientId = bundle.getString("id");
		serverIP = bundle.getString("ip");

		Intent i = new Intent("android.intent.action.MAIN");
		i.setFlags(Intent.FLAG_ACTIVITY_NEW_TASK);

		// clientSock = new Socket("10.0.2.2", 25250);
		// clientId = "test";

		clientSock = new Socket(serverIP, 25250);
		ServerMessage msg = new ServerMessage(ServerMessage.MESSAGE_CHECKID,
				clientId, "");
		msg.send(clientSock);

		ServerMessage reply = ServerMessage.receive(clientSock);

		if (reply.getType() == ServerMessage.MESSAGE_IDAVAILABLE) {
			ComponentName n = new ComponentName("mypackage.FriendTracker", "mypackage.FriendTracker.FriendTracker");
			i.setComponent(n);
			startActivity(i);
			updateTimer.start();

		} else {
			Toast toast = Toast.makeText(getApplicationContext(),
					"Invalid Client Id Specified\n", Toast.LENGTH_SHORT);
			toast.show();
		}
	}

	public void logout() throws IOException {
		if (clientSock != null && clientId != null) {
			ServerMessage msg = new ServerMessage(ServerMessage.MESSAGE_LEAVE,
					clientId, "");
			msg.send(clientSock);
		}
		clientSock = null;
		clientId = null;
	}

	public void viewFriends(Intent intent) throws Exception {	
		String temp = "HELLO";
		updateDatabase();

		Intent i = new Intent("android.intent.action.MAIN");
		i.setFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
		ComponentName n = new ComponentName("mypackage.FriendViewer", "mypackage.FriendViewer.FriendViewer");
		i.setComponent(n);
		startActivity(i);
	}

	public void addFriend(Intent intent) throws IOException {
		Bundle b = intent.getExtras();
		String id = b.getString("FriendId");
		ServerMessage msg = new ServerMessage(ServerMessage.MESSAGE_FRIENDS,
				clientId, new String(id + "\n"));
		msg.send(clientSock);
		ServerMessage response = ServerMessage.receive(clientSock);

		String message = formatResponse(response.getMessage());

		addRecord(id, message);

		Toast toast = Toast.makeText(getApplicationContext(), "Friend Added\n",
				Toast.LENGTH_SHORT);
		toast.show();
	}

	public void removeFriend(Intent intent) throws IOException {
		Bundle b = intent.getExtras();
		String id = b.getString("FriendId");

		int count = removeRecord(id);

		if(count > 0) {
			Toast toast = Toast.makeText(getApplicationContext(), "Friend Removed\n",
					Toast.LENGTH_SHORT);
			toast.show();
		}
	}

	public void updateDatabase() throws Exception {
		ArrayList<String> ids = new ArrayList<String>(); 

		Cursor cursor =  getContentResolver().query(Friends.CONTENT_URI, null, null, null, null);
		cursor.moveToNext();

		do {

			int infoColumn = cursor.getColumnIndex(Friends.INFO);
			String value = cursor.getString(infoColumn);
			String id = value.substring(0, value.indexOf(" "));
			ids.add(id);
		} while (cursor.moveToNext());

		//Update all the ids in the database
		for(int i = 0; i < ids.size(); i++) {
			removeRecord(ids.get(i));
			ServerMessage msg = new ServerMessage(ServerMessage.MESSAGE_FRIENDS, clientId, new String(ids.get(i) + '\n'));
			msg.send(clientSock);
			ServerMessage response = ServerMessage.receive(clientSock);
			String message = formatResponse(response.getMessage());
			addRecord(ids.get(i), message);
		}
	}

	public void updateLocation(Intent intent) throws IOException {
		Bundle b = intent.getExtras();
		String lat = b.getString("Lat");
		String lon = b.getString("Lon");

		// Trim excess spaces and uppercase them
		lat = lat.trim();
		lat = lat.toUpperCase();
		lon = lon.trim();
		lon = lon.toUpperCase();

		// Validate the lat and lon coordinates
		boolean validLat = validateLatitude(lat);
		boolean validLon = validateLongitude(lon);

		if (validLat && validLon) {
			String data = lat + " " + lon;
			ServerMessage msg = new ServerMessage(ServerMessage.MESSAGE_UPDATE,
					clientId, data);
			msg.send(clientSock);

			addRecord(clientId, new String(clientId + " " + data));

			Toast toast = Toast.makeText(getApplicationContext(),
					"Location Updated\n", Toast.LENGTH_SHORT);
			toast.show();
		} else if (!validLat) {
			Toast toast = Toast.makeText(getApplicationContext(),
					"Invalid Latitude Specified\nFormat Deg.Min.SecN\n",
					Toast.LENGTH_SHORT);
			toast.show();
		} else {// !validLon
			Toast toast = Toast.makeText(getApplicationContext(),
					"Invalid Longitude Specified\nFormat Deg.Min.SecE\n",
					Toast.LENGTH_SHORT);
			toast.show();
		}
	}

	private int removeRecord(String id) {
		String match = findRecord(id);

		if(match.equalsIgnoreCase("NO MATCHES"))
			return 0;

		return getContentResolver().delete(Friends.CONTENT_URI, Friends.INFO+"=?", new String[]{match});
	}

	private String findRecord(String id) {
		Cursor cursor =  getContentResolver().query(Friends.CONTENT_URI, null, null, null, null);

		while (cursor.moveToNext()) {
			int infoColumn = cursor.getColumnIndex(Friends.INFO);
			String value = cursor.getString(infoColumn);
			if(value.startsWith(id)) {
				return value;
			}
		}

		return "NO MATCHES";
	}

	private Uri addRecord(String id, String data) {
		ContentValues cv=new ContentValues();
		cv.put(Friend.Friends.INFO, data);

		//Try to delete the old one and add the new o
		int count = removeRecord(id);

		return getContentResolver().insert(Friends.CONTENT_URI, cv);
	}

	private String formatResponse(String response) {
		String message = response;

		message = message.substring(0, message.length() - 1); //remove the \n char

		//Remove the : chars
		while(message.indexOf(":") != -1)
			message = message.substring(0, message.indexOf(":")) + message.substring(message.indexOf(":") + 1); 

		return message;
	}

	private boolean validateLatitude(String lat) {
		if (lat.contains(" "))
			return false;

		if (!lat.endsWith("N") && !lat.endsWith("S"))
			return false;

		// Get the first field
		String temp = lat.substring(0, lat.indexOf("."));
		String rest = lat.substring(lat.indexOf(".") + 1, lat.length() - 1); // Save the rest w/o N/S
		rest = rest.trim();

		try {
			int num = Integer.parseInt(temp);

			// Degrees
			if (num < 0 || num > 90)
				return false;

			// Get the next field
			temp = rest.substring(0, rest.indexOf("."));
			rest = rest.substring(rest.indexOf(".") + 1);
			rest = rest.trim();

			// Minutes
			num = Integer.parseInt(temp);
			if (num < 0 || num > 60)
				return false;

			// Get the last field
			temp = rest;

			num = Integer.parseInt(temp);
			if (num < 0 || num > 60)
				return false;

			return true;
		} catch (NumberFormatException e) {
			return false;
		}
	}

	private boolean validateLongitude(String lon) {
		if (lon.contains(" "))
			return false;

		if (!lon.endsWith("E") && !lon.endsWith("W"))
			return false;

		// Get the first field
		String temp = lon.substring(0, lon.indexOf("."));
		String rest = lon.substring(lon.indexOf(".") + 1, lon.length() - 1); // Save the rest w/o E/S
		rest = rest.trim();

		try {
			int num = Integer.parseInt(temp);

			// Degrees
			if (num < 0 || num > 180)
				return false;

			// Get the next field
			temp = rest.substring(0, rest.indexOf("."));
			rest = rest.substring(rest.indexOf(".") + 1);
			rest = rest.trim();

			// Minutes
			num = Integer.parseInt(temp);
			if (num < 0 || num > 60)
				return false;

			// Get the last field
			temp = rest;

			num = Integer.parseInt(temp);
			if (num < 0 || num > 60)
				return false;

			return true;
		} catch (NumberFormatException e) {
			return false;
		}
	}

	public void startHistory(Intent intent) throws IOException {
		Intent i = new Intent("android.intent.action.MAIN");
		i.setFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
		ComponentName n = new ComponentName("mypackage.FriendTracker",
		"mypackage.FriendTracker.ViewHistory");
		i.setComponent(n);
		myHistory = "No Locations";
		ServerMessage msg = new ServerMessage(ServerMessage.MESSAGE_HISTORY,
				clientId, "");
		msg.send(clientSock);
		ServerMessage response = ServerMessage.receive(clientSock);
		myHistory = response.getMessage();
		startActivity(i);
	}
}
