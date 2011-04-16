package mypackage.FriendTracker;

import android.app.Activity;
import android.content.ComponentName;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;

public class LoginFailed extends Activity {
	@Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.login_failed_layout);
    }
	
	public void close(View view) {
		Intent i = new Intent("android.intent.action.MAIN");
    	ComponentName n = new ComponentName("mypackage.FriendTracker", "mypackage.FriendTracker.LoginScreen");
    	i.setComponent(n);
    	startActivity(i); 
    	//finish();
	}
}
