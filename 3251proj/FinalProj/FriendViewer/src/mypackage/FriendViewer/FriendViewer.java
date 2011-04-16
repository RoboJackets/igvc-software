package mypackage.FriendViewer;

import java.util.Set;
import mypackage.FriendTracker.FriendProvider;
import mypackage.FriendTracker.FriendTrackerControl;
import android.app.Activity;
import android.content.ComponentName;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.widget.EditText;

public class FriendViewer extends Activity {
	/** Called when the activity is first created. */
	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.main);
		displayList();
	}
	
	public void displayList() {
		Set<String> keys = FriendProvider.getAllFriends();
		if(keys == null)
			return;
		
		String[] keyList = (String[]) keys.toArray();
		String friends = "";
		
		for(int i = 0; i < keyList.length; i++) {
			String friend = keyList[i] + " " + FriendProvider.getFriend(keyList[i]) + "\n";
			friends += friend;
		}
		
		//EditText text = (EditText) findViewById(R.id.FriendListId);
		//text.setText(friends);
	}
	
	//These methods are called when there button is pressed
	
	public void addFriend(View view) {
		String id = "";
		EditText text = (EditText) findViewById(R.id.FriendID);
		id = text.getText().toString();
		FriendProvider.addFriend(id);
		displayList();
	}
	
	public void removeFriend(View view) {
		String id = "";
		EditText text = (EditText) findViewById(R.id.FriendID);
		id = text.getText().toString();
		FriendProvider.removeFriend(id);
		displayList();
	}
	
	public void viewMap(View view) {
		Intent i = new Intent("android.intent.action.MAIN");
		i.setFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
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
    	finish();
    }
    
    public void close(View view) {
    	Intent i = new Intent("android.intent.action.MAIN");
    	i.setFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
    	ComponentName n = new ComponentName("mypackage.FriendTracker", "mypackage.FriendTracker.FriendTracker");
    	i.setComponent(n);
    	startActivity(i);
    	finish();
    }
}