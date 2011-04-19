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
        
        if(savedInstanceState == null || savedInstanceState.isEmpty())
        	return;
        
        String id = savedInstanceState.getString("Id");
        int lat = savedInstanceState.getInt("Lat");
        int lon = savedInstanceState.getInt("Lon");
        
        View idLayout = findViewById(R.id.FriendIDLayout);
        EditText idText = (EditText) idLayout.findViewById(R.id.FriendsTxtId);
        
        View latLayout = findViewById(R.id.LatLayout);
        EditText latText = (EditText) latLayout.findViewById(R.id.LatTxtId);
        
        View lonLayout = findViewById(R.id.LonLayout);
        EditText lonText = (EditText) lonLayout.findViewById(R.id.LonTxtId);
        
        idText.setText(id);
        latText.setText(String.valueOf(lat));
        lonText.setText(String.valueOf(lon));
    }
    
    //These methods are called when their button is pressed
    
    public void logout(View view) {
    	Intent i = new Intent("android.intent.action.MAIN");
    	i.setFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
    	ComponentName n = new ComponentName("mypackage.FriendTracker", "mypackage.FriendTracker.LoginScreen");
    	i.setComponent(n);
    	startActivity(i);
    	//finish();
    }
    
    public void close(View view) {
    	Intent i = new Intent("android.intent.action.MAIN");
    	i.setFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
    	ComponentName n = new ComponentName("mypackage.FriendViewer", "mypackage.FriendViewer.FriendMap");
    	i.setComponent(n);
    	startActivity(i);
    	//finish();
    }
    
  //Kill the activity when its not in the foreground anymore
	public void onPause() {
		super.onPause();
		finish();
	}
}