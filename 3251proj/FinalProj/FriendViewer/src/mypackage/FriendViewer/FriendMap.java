package mypackage.FriendViewer;

import java.util.List;

import mypackage.FriendTracker.Friend;
import mypackage.FriendTracker.Friend.Friends;

import android.os.Bundle;
import android.os.Handler;
import android.view.View;
import android.content.ComponentName;
import android.content.ContentResolver;
import android.content.Context;
import android.content.Intent;
import android.database.ContentObserver;
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

	private MapContentObserver friendsObserver = null;
	private Handler handler = new Handler();

	private class MapContentObserver extends ContentObserver {
		public MapContentObserver( Handler h ) {
			super( h );
		}

		public void onChange(boolean selfChange) {
			updateOverlays();
		}
	}


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
		registerContentObservers();

		/*   GeoPoint point = new GeoPoint(19240000,-99120000);
	    OverlayItem overlayitem = new OverlayItem(point, "Hola, Mundo!", "I'm in Mexico City!");
	    itemizedoverlay.addOverlay(overlayitem);
	    mapOverlays.add(itemizedoverlay);
		 */
		
		updateOverlays();  
	}

	private void updateOverlays() {
		removeOverlays();
		addOverlays();
	}

	private void removeOverlays() {
	
		while(mapOverlays.size() > 0) {
			mapOverlays.remove(0);
		}
		
		while(itemizedoverlay.size() > 0) {
			itemizedoverlay.removeOverlay(0);
		}
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
			value = value.trim();

			//This indicates that there are no coordinates for that friend don't add them to the map
			if(value.contains("Has No Coordinates") || value.contains("Not Found"))
				continue;
			
			value = value.toUpperCase();
			
			String latText = value.contains("N") ? value.substring(0, value.indexOf("N")+1) : 
				value.substring(0, value.indexOf("S") + 1);

			String lonText = value.contains("N") ? value.substring(value.indexOf("N")+1).trim() : 
				value.substring(value.indexOf("S") + 1).trim();
			
			lat = getLatitude(latText);
			lon = getLongitude(lonText);

			//Error getting the value don't add this point
			if(lat == -1 || lon == -1) 
				continue;

			GeoPoint point = new GeoPoint(lat, lon);
			OverlayItem overlayitem = new OverlayItem(point, friend, "");

			itemizedoverlay.addOverlay(overlayitem);
			mapOverlays.add(itemizedoverlay);
		}
	}

	private int getLatitude(String value) {
		int deg, min, sec, lat = 0;
		double num;
		String dir;

		//Get the direction
		dir = value.substring(value.length() - 1);

		String temp = value.substring(0, value.indexOf("."));
		//Get the rest without the direction
		String rest = value.substring(value.indexOf(".") + 1, value.length() - 1);

		try {
			deg = Integer.parseInt(temp);	
			temp = rest.substring(0, rest.indexOf("."));
			rest = rest.substring(rest.indexOf(".") + 1);
			min = Integer.parseInt(temp);
			sec = Integer.parseInt(rest);

			//Get the value in deg.xxxx format
			num  = deg + (double)(min/60.0) + (double)(sec/3600.);

			//Multiply that to point format of latE6
			lat = (int) (num * 1000000);

			if(dir.equals("S"))
				return lat * -1;

			return lat;
		}
		catch (NumberFormatException e) {
			return -1;
		}
	}

	private int getLongitude(String value) {
		int deg, min, sec, lon = 0;
		double num;
		String dir;

		//Get the direction
		dir = value.substring(value.length() - 1);

		String temp = value.substring(0, value.indexOf("."));
		//Get the rest without the direction
		String rest = value.substring(value.indexOf(".") + 1, value.length() - 1);

		try {
			deg = Integer.parseInt(temp);	
			temp = rest.substring(0, rest.indexOf("."));
			rest = rest.substring(rest.indexOf(".") + 1);
			min = Integer.parseInt(temp);
			sec = Integer.parseInt(rest);

			//Get the value in deg.xxxx format
			num  = deg + (double)(min/60.0) + (double)(sec/3600.);

			//Multiply that to point format of latE6
			lon = (int) (num * 1000000);

			if(dir.equals("W"))
				return lon * -1;

			return lon;
		}
		catch (NumberFormatException e) {
			return -1;
		}
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

	//Kill the activity when its not in the foreground anymore
	public void onPause() {
		super.onPause();
		unregisterContentObservers();
		finish();
	}

	@Override
	protected void onStop() {
		super.onStop();
		unregisterContentObservers();
	}

	// Set up content observer for our content provider
	private void registerContentObservers() {
		ContentResolver cr = getContentResolver();
		friendsObserver = new MapContentObserver( handler );
		cr.registerContentObserver( Friend.Friends.CONTENT_URI, true,
				friendsObserver );

	}

	private void unregisterContentObservers() {
		ContentResolver cr = getContentResolver();
		if( friendsObserver != null ) {		// just paranoia
			cr.unregisterContentObserver( friendsObserver );
			friendsObserver = null;
		}
	}
}