package mypackage.FriendTracker;

import android.app.Activity;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.os.Bundle;
import android.os.IBinder;
import android.view.View;

public class ViewHistory extends Activity implements ServiceConnection {
    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.history_layout);
        
        Context context = this.getApplicationContext();
        Intent intent = new Intent(context, mypackage.FriendTracker.FriendTrackerControl.class);
        context.bindService(intent, this, Context.BIND_AUTO_CREATE) ;  
    }
    
    //The following methods are called when there button is pressed

    public void logout(View view) {
    	Intent i = new Intent("android.intent.action.MAIN");
    	ComponentName n = new ComponentName("mypackage.FriendTracker", "mypackage.FriendTracker.LoginScreen");
    	i.setComponent(n);
    	startActivity(i); 
    	finish();
    }
    
    public void close(View view) {
    	Intent i = new Intent("android.intent.action.MAIN");
    	ComponentName n = new ComponentName("mypackage.FriendTracker", "mypackage.FriendTracker.FriendTracker");
    	i.setComponent(n);
    	startActivity(i); 
    	finish();
    }

	@Override
	public void onServiceConnected(ComponentName name, IBinder service) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onServiceDisconnected(ComponentName name) {
		// TODO Auto-generated method stub
		
	}
}