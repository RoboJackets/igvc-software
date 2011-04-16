package mypackage.ViewContact;

import android.app.Activity;
import android.content.ComponentName;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.widget.EditText;

public class ViewContact extends Activity {
    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
        
        String id = savedInstanceState.getString("Id");
        String lat = savedInstanceState.getString("Lat");
        String lon = savedInstanceState.getString("Lon");
        
        EditText idText = (EditText) findViewById(R.id.FriendsTxtId);
        EditText latText = (EditText) findViewById(R.id.LatTxtId);
        EditText lonText = (EditText) findViewById(R.id.LonTxtId);
        
        idText.setText(id);
        latText.setText(lat);
        lonText.setText(lon);
    }
    
    //These methods are called when their button is pressed
    
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
    	ComponentName n = new ComponentName("mypackage.FriendViewer", "mypackage.FriendViewer.FriendMap");
    	i.setComponent(n);
    	startActivity(i);
    	finish();
    }
}