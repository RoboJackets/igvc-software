package mypackage.FriendViewer;

import android.app.Activity;
import android.content.ComponentName;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;

public class FriendViewer extends Activity {
	/** Called when the activity is first created. */
	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.main);
	}
	
	@Override
	public void onStart() {
		super.onStart();
		setContentView(R.layout.main);
	}
	
	//These methods are called when there button is pressed
	
	public void addFriend(View view) {
		
	}
	
	public void removeFriend(View view) {
		
	}
	
	public void viewMap(View view) {
		Intent i = new Intent("android.intent.action.MAIN");
    	ComponentName n = new ComponentName("mypackage.FriendViewer", "mypackage.FriendViewer.FriendMap");
    	i.setComponent(n);
    	startActivity(i); 
	}
	
	public void logout(View view) {
		Intent i = new Intent("android.intent.action.MAIN");
    	ComponentName n = new ComponentName("mypackage.FriendTracker", "mypackage.FriendTracker.LoginScreen");
    	i.setComponent(n);
    	startActivity(i); 
    }
    
    public void close(View view) {
    	
    }
}