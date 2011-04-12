package mypackage.FriendTracker;

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
    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.login_layout);
    }
    
    //The following methods are called when there button is pressed
    
    public void login(View view) {
    	//Start the FriendTrackerControl Service & bind this screen to it
    	Context context = this.getApplicationContext();
		Intent intent = new Intent(context, mypackage.FriendTracker.FriendTrackerControl.class);
		
		//Add this class as the binder for the intent
		intent.putExtra("binder", "LoginScreen");
		
		//Get the login id and add it to the intent
		EditText login = (EditText)findViewById(R.id.LoginTxtId);
		intent.putExtra("id", login.getText().toString());
		
		//Get the server ip and add it to the intent
		EditText ip = (EditText)findViewById(R.id.serverIPTxtID);
		intent.putExtra("ip", ip.getText().toString());	
        context.bindService(intent, this, Context.BIND_AUTO_CREATE);
    }
    
    public void close(View view) {
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
