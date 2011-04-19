package mypackage.FriendViewer;

import java.util.List;

import mypackage.FriendTracker.Friend.Friends;

import android.os.Bundle;
import android.view.View;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.database.Cursor;
import android.graphics.drawable.Drawable;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import com.google.android.maps.GeoPoint;
import com.google.android.maps.MapActivity;
import com.google.android.maps.MapController;
import com.google.android.maps.MapView;
import com.google.android.maps.Overlay;
import com.google.android.maps.OverlayItem;

public class FriendMap extends MapActivity {
	private FriendPositionOverlay itemizedoverlay;
	private List<Overlay> mapOverlays;
	//private String serverIP;
	
	@Override
	public void onCreate(Bundle savedInstanceState) {
	    super.onCreate(savedInstanceState);
	    setContentView(R.layout.map_layout);
	    
	    View mapLayout = findViewById(R.id.mapLayoutId);
	    MapView mapView = (MapView) mapLayout.findViewById(R.id.MapID);
	    mapView.setBuiltInZoomControls(true);
	    
	    mapOverlays = mapView.getOverlays();
	    Drawable drawable = this.getResources().getDrawable(R.drawable.icon);
	    itemizedoverlay = new FriendPositionOverlay(drawable, getApplicationContext());
	    
	    GeoPoint point = new GeoPoint(19240000,-99120000);
	    OverlayItem overlayitem = new OverlayItem(point, "Hola, Mundo!", "I'm in Mexico City!");
	    itemizedoverlay.addOverlay(overlayitem);
	    mapOverlays.add(itemizedoverlay);

	    
	    //addOverlays();  
	}
	
	private void addOverlays() {
		String friend, value;
		int lat, lon;
		
		Cursor cursor =  getContentResolver().query(Friends.CONTENT_URI, null, null, null, null);

		while (cursor.moveToNext()) {
			int infoColumn = cursor.getColumnIndex(Friends.INFO);
			value = cursor.getString(infoColumn);
		    friend = value.substring(0, value.indexOf(" "));
		    value = value.substring(value.indexOf(" "));
		    value.trim();
		    lat = getLatitude(value);
		    lon = getLongitude(value);
		    
		    GeoPoint point = new GeoPoint(lat, lon);
		    OverlayItem overlayitem = new OverlayItem(point, friend, "");
		    
		    itemizedoverlay.addOverlay(overlayitem);
		    mapOverlays.add(itemizedoverlay);
		}
	}
	
	private int getLatitude(String value) {
		return 0;
	}
	
	private int getLongitude(String value) {
		return 0;
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
    	finish();
    }
    
    public void close(View view) {
    	Intent i = new Intent("android.intent.action.MAIN");
    	ComponentName n = new ComponentName("mypackage.FriendViewer", "mypackage.FriendViewer.FriendViewer");
    	i.setComponent(n);
    	startActivity(i); 
    	finish();
    }
}