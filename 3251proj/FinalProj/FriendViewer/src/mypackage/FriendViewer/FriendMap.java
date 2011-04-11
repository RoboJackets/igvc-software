package mypackage.FriendViewer;

import android.os.Bundle;
import android.content.Context;
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
}