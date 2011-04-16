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

public class LoginScreen extends Activity implements ServiceConnection {
	FriendTrackerControl mService;
	Context mContext;
	
    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {	
        super.onCreate(savedInstanceState);
        setContentView(R.layout.login_layout);
        mContext = this.getApplicationContext();
        Intent intent = new Intent(mContext, mypackage.FriendTracker.FriendTrackerControl.class);
        mContext.bindService(intent, this, Context.BIND_AUTO_CREATE); 
    }
    
    //The following methods are called when there button is pressed
    
    public void login(View view) {
		Intent intent = new Intent(mContext, mypackage.FriendTracker.FriendTrackerControl.class);
	
		//Get the login id and add it to the intent
		EditText login = (EditText)findViewById(R.id.LoginTxtId);
		intent.putExtra("id", login.getText().toString());
		
		//Get the server ip and add it to the intent
		EditText ip = (EditText)findViewById(R.id.serverIPTxtID);
		intent.putExtra("ip", ip.getText().toString());
        
		try {
			mService.login(intent);
		} catch (IOException e) {
			Intent i = new Intent("android.intent.action.MAIN");
			i.setFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
			ComponentName n = new ComponentName("mypackage.FriendTracker", "mypackage.FriendTracker.LoginFailed");
			i.setComponent(n);
			startActivity(i);
		}
		//mContext.unbindService(this); //Finish this activity it's no longer needed
		finish();
    }
    
    public void close(View view) {
    		finish();
    }


	@Override
	public void onServiceConnected(ComponentName name, IBinder service) {
		// TODO Auto-generated method stub
		LocalBinder binder = (LocalBinder) service;
		mService = binder.getService();
		
		//Logout any logged in user
		try {
			mService.logout();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	@Override
	public void onServiceDisconnected(ComponentName name) {
		// TODO Auto-generated method stub
	}
}
