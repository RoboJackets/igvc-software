package mypackage.FriendViewer;

import java.util.Set;
import mypackage.FriendTracker.FriendProvider;
import android.app.Activity;
import android.content.ComponentName;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.widget.EditText;
//import mypackage.FriendViewer.*;
import android.widget.Toast;

public class FriendViewer extends Activity {
	
	/** Called when the activity is first created. */
	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.main);
		displayList();
	}
	
	public void displayList() {
	//	if(!FriendProvider.isInitialized())
		//	return;
		
		Set<String> keys = FriendProvider.getAllFriends();
		if(keys == null)
			return;
		
		String[] keyList = (String[]) keys.toArray();
		String friends = "";
		
		for(int i = 0; i < keyList.length; i++) {
			String friend = keyList[i] + " " + FriendProvider.getFriend(keyList[i]) + "\n";
			friends += friend;
		}
		View layout = findViewById(R.id.friendListLayout);
		EditText list = (EditText) layout.findViewById(R.id.FriendListId);
		list.setText(friends);
	}
	
	//These methods are called when there button is pressed
	
	public void addFriend(View view) {
		String id = "";
		View FriendLayout = findViewById(R.id.FrinedInputLayout);
		EditText text = (EditText) FriendLayout.findViewById(R.id.FriendTxtId);
		id = text.getText().toString();
		
		if(FriendProvider.isInitialized()) {
			FriendProvider.addFriend(id);
			Toast toast = Toast.makeText(getApplicationContext(), "Friend Added\n",
					Toast.LENGTH_SHORT);
			toast.show();
		}
		else {
			Toast toast = Toast.makeText(getApplicationContext(), "Friend Provider Not Initialized\n",
					Toast.LENGTH_SHORT);
			toast.show();
		}
		displayList();
	}
	
	public void removeFriend(View view) {
		String id = "";
		View FriendLayout = findViewById(R.id.FrinedInputLayout);
		EditText text = (EditText) FriendLayout.findViewById(R.id.FriendTxtId);
		id = text.getText().toString();
		
		if(FriendProvider.isInitialized()) {
			FriendProvider.removeFriend(id);
			Toast toast = Toast.makeText(getApplicationContext(), "Friend Added\n",
					Toast.LENGTH_SHORT);
			toast.show();
		}
		else {
			Toast toast = Toast.makeText(getApplicationContext(), "Friend Provider Not Initialized\n",
					Toast.LENGTH_SHORT);
			toast.show();
		};
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
    
    //Kill the activity when its not in the foreground anymore
    public void onPause() {
    	super.onPause();
    	finish();
    }
}