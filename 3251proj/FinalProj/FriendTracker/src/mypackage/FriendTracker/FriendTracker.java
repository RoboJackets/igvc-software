package mypackage.FriendTracker;

import android.app.Activity;
import android.content.ComponentName;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;


public class FriendTracker extends Activity {
    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
        
    }
    
    //The following methods are called when there button is pressed
    
    public void viewFriends(View view) {
    	Intent i = new Intent("android.intent.action.MAIN");
    	ComponentName n = new
    	ComponentName("mypackage.FriendViewer",
    	"mypackage.FriendViewer.FriendViewer");
    	i.setComponent(n);
    	startActivity(i); 
    }
    
    public void lookupFriend(View view) {
    	
    }
    
    public void updateLocation(View view) {
    	
    }
    
    public void history(View view) {
    	Intent i = new Intent("android.intent.action.MAIN");
    	ComponentName n = new
    	ComponentName("mypackage.FriendTracker",
    	"mypackage.FriendTracker.ViewHistory");
    	i.setComponent(n);
    	startActivity(i);
    }
    
    public void logout(View view) {
    	Intent i = new Intent("android.intent.action.MAIN");
    	ComponentName n = new
    	ComponentName("mypackage.FriendTracker",
    	"mypackage.FriendTracker.LoginScreen");
    	i.setComponent(n);
    	startActivity(i); 
    }
    
    public void close(View view) {
    	
    }
}