package mypackage.FriendViewer;

import android.os.Bundle;
import android.view.View;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import com.google.android.maps.GeoPoint;
import com.google.android.maps.MapActivity;
import com.google.android.maps.MapController;
import com.google.android.maps.MapView;


public class FriendMap extends MapActivity {
	@Override
	public void onCreate(Bundle savedInstanceState) {
	    super.onCreate(savedInstanceState);
	    setContentView(R.layout.map_layout);
	}
	
	/** Called when the activity is first created. */
	@Override
	protected boolean isRouteDisplayed() {
	    return false;
	}
	
	//These methods are called when their button is pressed
	
	public void logout(View view) {
		Intent i = new Intent("android.intent.action.MAIN");
    	ComponentName n = new ComponentName("mypackage.FriendTracker", "mypackage.FriendTracker.LoginScreen");
    	i.setComponent(n);
    	startActivity(i); 
    }
    
    public void close(View view) {
    	Intent i = new Intent("android.intent.action.MAIN");
    	ComponentName n = new ComponentName("mypackage.ViewContact", "mypackage.ViewContact.ViewContact");
    	i.setComponent(n);
    	startActivity(i); 
    }
}