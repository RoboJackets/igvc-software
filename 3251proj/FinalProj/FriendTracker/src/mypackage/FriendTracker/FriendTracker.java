package mypackage.FriendTracker;

import java.io.IOException;

import mypackage.FriendTracker.FriendTrackerControl.LocalBinder;
import android.app.Activity;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.os.Bundle;
import android.os.IBinder;
import android.view.View;
import android.widget.EditText;


public class FriendTracker extends Activity implements ServiceConnection {
	FriendTrackerControl mService;
	
    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
        
        Context context = this.getApplicationContext();
        Intent intent = new Intent(context, mypackage.FriendTracker.FriendTrackerControl.class);
        context.bindService(intent, this, Context.BIND_AUTO_CREATE); 
    }
    
    //The following methods are called when there button is pressed
    
    public void viewFriends(View view) {
    	Intent i = new Intent("android.intent.action.MAIN");
    	ComponentName n = new ComponentName("mypackage.FriendViewer", "mypackage.FriendViewer.FriendViewer");
    	i.setComponent(n);
    	startActivity(i); 
    }
    
    public void lookupFriend(View view) {
    	//Start the FriendTrackerControl Service & bind this screen to it
    	Context context = getApplicationContext();
		Intent intent = new Intent(context, mypackage.FriendTracker.FriendTrackerControl.class);
    	EditText friendId = (EditText)findViewById(R.id.FriendsTxtId);
		intent.putExtra("FriendId", friendId.getText().toString());
		
    	try {
			mService.lookupFriend(intent);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
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
    	finish();
    }
    
    public void close(View view) {
    	Intent i = new Intent("android.intent.action.MAIN");
    	ComponentName n = new
    	ComponentName("mypackage.FriendTracker",
    	"mypackage.FriendTracker.LoginScreen");
    	i.setComponent(n);
    	startActivity(i); 
    	finish();
    }

	@Override
	public void onServiceConnected(ComponentName name, IBinder service) {
		// TODO Auto-generated method stub
		LocalBinder binder = (LocalBinder) service;
		mService = binder.getService();
	}

	@Override
	public void onServiceDisconnected(ComponentName name) {
		// TODO Auto-generated method stub
		
	}
}