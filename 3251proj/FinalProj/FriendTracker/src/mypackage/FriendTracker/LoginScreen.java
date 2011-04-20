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
import android.widget.Toast;

public class LoginScreen extends Activity implements ServiceConnection {
	FriendTrackerControl mService;
	Context mContext;

	/** Called when the activity is first created. */
	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.login_layout);
		mContext = this.getApplicationContext();
		Intent intent = new Intent(mContext,
				mypackage.FriendTracker.FriendTrackerControl.class);
		mContext.bindService(intent, this, Context.BIND_AUTO_CREATE);
	}

	// The following methods are called when there button is pressed

	public void login(View view) {
		Intent intent = new Intent(mContext,
				mypackage.FriendTracker.FriendTrackerControl.class);

		// Get the login id and add it to the intent
		View loginLayout = findViewById(R.id.LoginScreen_cliendIDLayout);
		EditText login = (EditText) loginLayout
				.findViewById(R.id.LoginScreen_LoginTxtId);
		String id = login.getText().toString();
		intent.putExtra("id", id);

		if (id.contains(" ") || id.equals("")) {
			Toast toast = Toast.makeText(getApplicationContext(),
					"Error: Id cannot contains spaces\n", Toast.LENGTH_SHORT);
			toast.show();
			return;
		}

		if (id.equals("q") || id.equals("Q")) {
			Toast toast = Toast.makeText(getApplicationContext(),
					"Error: Id cannot be q/Q\n", Toast.LENGTH_SHORT);
			toast.show();
			return;
		}

		// Get the server ip and add it to the intent
		View ipLayout = findViewById(R.id.LoginScreen_ServerIPLayout);
		EditText ip = (EditText) ipLayout
				.findViewById(R.id.LoginScreen_serverIPTxtID);
		
		String addr = ip.getText().toString();
		if(addr.equals("")) {
			Toast toast = Toast.makeText(getApplicationContext(),
					"Error: Ip is not set\n", Toast.LENGTH_SHORT);
			toast.show();
			return;
		}
			
		intent.putExtra("ip", addr);

		try {
			mService.login(intent);
			// finish();
		} catch (Exception e) {
			Toast toast = Toast.makeText(getApplicationContext(),
					"Unable to Connect to Server\n", Toast.LENGTH_SHORT);
			toast.show();
		}
		// mContext.unbindService(this); //Finish this activity it's no longer
		// needed
	}

	public void close(View view) {
		//unbindService(this);
		finish();
	}

	// Kill the activity when its not in the foreground anymore
	public void onPause() {
		super.onPause();
		finish();
	}

	@Override
	public void onServiceConnected(ComponentName name, IBinder service) {
		// TODO Auto-generated method stub
		LocalBinder binder = (LocalBinder) service;
		mService = binder.getService();

		// Logout any logged in user
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
		finish();
	}
}
