package mypackage.FriendViewer;

import java.util.ArrayList;

import android.app.AlertDialog;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.graphics.drawable.Drawable;
import android.os.Bundle;
import android.widget.Toast;

import com.google.android.maps.ItemizedOverlay;
import com.google.android.maps.OverlayItem;

public class FriendPositionOverlay extends ItemizedOverlay {

	private ArrayList<OverlayItem> mOverlays = new ArrayList<OverlayItem>();
	private Context mContext;
	
	public FriendPositionOverlay(Drawable defaultMarker) {
		super(boundCenterBottom(defaultMarker));
		// TODO Auto-generated constructor stub
	}
	
	public FriendPositionOverlay(Drawable defaultMarker, Context context) {
		  super(boundCenterBottom(defaultMarker));
		  mContext = context;
		}

	@Override
	public int size() {
	  return mOverlays.size();
	}
	
	public void addOverlay(OverlayItem overlay) {
	    mOverlays.add(overlay);
	    populate();
	}
	
	@Override
	protected OverlayItem createItem(int i) {
	  return mOverlays.get(i);
	}
	
	@Override
	protected boolean onTap(int index) {
	  OverlayItem item = mOverlays.get(index);
	  Intent i = new Intent("android.intent.action.MAIN");
	  i.setFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
	  ComponentName n = new ComponentName("mypackage.ViewContact", "mypackage.ViewContact.ViewContact");
	  i.setComponent(n);
	  Bundle b = new Bundle();
	  
	  b.putString("Id", item.getTitle());
	  b.putInt("Lat", item.getPoint().getLatitudeE6());
	  b.putInt("Lon", item.getPoint().getLongitudeE6());
	  i.putExtras(b);
	  mContext.startActivity(i);
	  return true;
	}
}
