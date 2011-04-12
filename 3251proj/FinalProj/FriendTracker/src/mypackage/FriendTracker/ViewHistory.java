package mypackage.FriendTracker;

import android.app.Activity;
import android.content.ComponentName;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;

public class ViewHistory extends Activity {
    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.history_layout);
        
    }
    
    //The following methods are called when there button is pressed

    
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