package mypackage.FriendTracker;

import android.app.Activity;
import android.content.ComponentName;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.widget.EditText;

public class ViewHistory extends Activity {
	/** Called when the activity is first created. */
	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.history_layout);

		String history = formatHistory(FriendTrackerControl.myHistory);

		View historyLayout = findViewById(R.id.HistoryLayout);
		EditText historyText = (EditText) historyLayout
				.findViewById(R.id.History_historyTxtId);
		historyText.setText(history);
	}

	public String formatHistory(String input) {
		int nextW, nextE;
		String history = "";

		String temp = input.toUpperCase();
		temp = temp.substring(temp.indexOf(" ")); // remove the user id
		temp = temp.trim(); // trim any extra spaces

		// Parse the string into each history and add a \n to separate them
		while (temp.indexOf("E") != -1 || temp.indexOf("W") != -1) {
			nextE = temp.indexOf("E");
			nextW = temp.indexOf("W");
			
			if(nextE != -1 && nextE < nextW) {
				history += temp.substring(0, nextE+1) + "\n";
				temp = temp.substring(nextE+1);
			}
			else {
				history += temp.substring(0, nextW+1) + "\n";
				temp = temp.substring(nextW+1);
			}
			//update the string
			
			temp = temp.trim();
		}

		if(history.equals("")) 
			return "No Locations";
		
		history = history.substring(0, history.length() - 1); // trim off the
																// extra \n at
																// the end

		return history;
	}

	// The following methods are called when there button is pressed

	public void logout(View view) {
		Intent i = new Intent("android.intent.action.MAIN");
		i.setFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
		ComponentName n = new ComponentName("mypackage.FriendTracker",
				"mypackage.FriendTracker.LoginScreen");
		i.setComponent(n);
		startActivity(i);
		finish();
	}

	public void close(View view) {
		Intent i = new Intent("android.intent.action.MAIN");
		i.setFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
		ComponentName n = new ComponentName("mypackage.FriendTracker",
				"mypackage.FriendTracker.FriendTracker");
		i.setComponent(n);
		startActivity(i);
		finish();
	}

	// Kill the activity when its not in the foreground anymore
	public void onPause() {
		super.onPause();
		finish();
	}
}