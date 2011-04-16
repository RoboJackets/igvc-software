package mypackage.FriendTracker;

import android.app.Activity;
import android.content.ComponentName;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.widget.EditText;
import android.widget.TextView;

public class ViewHistory extends Activity {
    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.history_layout);
        
        //displayHistory(savedInstanceState);
    }
    
    /*public void displayHistory(Bundle b) {
    	//String history = b.getString("History");
        //EditText historyText = (EditText) this.
        //historyText.setText(history);
        
       // setContentView(R.layout.history_layout);
    }*/
    
    //The following methods are called when there button is pressed

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
}