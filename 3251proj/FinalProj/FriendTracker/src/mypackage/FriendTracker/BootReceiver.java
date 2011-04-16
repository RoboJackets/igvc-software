package mypackage.FriendTracker;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;

public class BootReceiver extends BroadcastReceiver {

	@Override
	public void onReceive(Context context, Intent intent) {
		//Start the LoginScreen Activity
		Intent startupIntent = new Intent(context, LoginScreen.class);
		startupIntent.setFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
		context.startActivity(startupIntent);
		
		//Start the FriendTrackerContron Service
		Intent startupService = new Intent(context, FriendTrackerControl.class);
		startupIntent.setFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
		context.startService(startupService);
	}
}
