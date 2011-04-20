package mypackage.FriendViewer;

import java.util.Set;

import mypackage.FriendTracker.Friend;
import mypackage.FriendTracker.FriendProvider;
import mypackage.FriendTracker.Friend.Friends;
import android.app.Activity;
import android.content.ComponentName;
import android.content.ContentResolver;
import android.content.ContentValues;
import android.content.Intent;
import android.database.ContentObserver;
import android.database.Cursor;
import android.net.Uri;
import android.os.Bundle;
import android.os.Handler;
import android.view.View;
import android.widget.EditText;
//import mypackage.FriendViewer.*;
import android.widget.Toast;

public class FriendViewer extends Activity {

	private String friendList;
	private FriendContentObserver friendsObserver = null;
	private Handler handler = new Handler();

	private class FriendContentObserver extends ContentObserver {
		public FriendContentObserver( Handler h ) {
			super( h );
		}

		public void onChange(boolean selfChange) {
			displayList();
		}
	}


	/** Called when the activity is first created. */
	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.main);
		registerContentObservers();
		displayList();
	}

	public void displayList() {
		friendList = "";

		Cursor cursor =  getContentResolver().query(Friends.CONTENT_URI, null, null, null, null);

		while (cursor.moveToNext()) {
			int infoColumn = cursor.getColumnIndex(Friends.INFO);
			String value = cursor.getString(infoColumn);
			friendList += value + "\n";
		}

		View layout = findViewById(R.id.friendListLayout);
		EditText list = (EditText) layout.findViewById(R.id.FriendListId);
		
		if(friendList.equals(""))  {
			list.setText("");
			return;
		}
		
		friendList = friendList.substring(0, friendList.length() - 1); //Remove the extra \n at the end
		list.setText(friendList);
	}

	//These methods are called when there button is pressed

	public void viewMap(View view) {
		Intent i = new Intent("android.intent.action.MAIN");
		i.setFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
		i.putExtra("Friends", friendList);
		ComponentName n = new ComponentName("mypackage.FriendViewer", "mypackage.FriendViewer.FriendMap");
		i.setComponent(n);
		startActivity(i); 
	}

	public void logout(View view) {
		Intent i = new Intent("android.intent.action.MAIN");
		i.setFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
		ComponentName n = new ComponentName("mypackage.FriendTracker", "mypackage.FriendTracker.LoginScreen");
		i.setComponent(n);
		startActivity(i);
		unregisterContentObservers();
	}

	public void close(View view) {
		Intent i = new Intent("android.intent.action.MAIN");
		i.setFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
		ComponentName n = new ComponentName("mypackage.FriendTracker", "mypackage.FriendTracker.FriendTracker");
		i.setComponent(n);
		startActivity(i);
		unregisterContentObservers();
	}

	//Kill the activity when its not in the foreground anymore
	public void onPause() {
		super.onPause();
		unregisterContentObservers();
		finish();
	}

	@Override
	protected void onStop() {
		super.onStop();
		unregisterContentObservers();
	}


	// Set up content observer for our content provider
	private void registerContentObservers() {
		ContentResolver cr = getContentResolver();
		friendsObserver = new FriendContentObserver( handler );
		cr.registerContentObserver( Friend.Friends.CONTENT_URI, true,
				friendsObserver );

	}

	private void unregisterContentObservers() {
		ContentResolver cr = getContentResolver();
		if( friendsObserver != null ) {		// just paranoia
			cr.unregisterContentObserver( friendsObserver );
			friendsObserver = null;
		}
	}
}