package mypackage.FriendTracker;

import android.net.Uri;
import android.provider.BaseColumns;

public class Friend {
	
	public Friend() {
	}
	
	public static final class Friends implements BaseColumns {
		private Friends() {
		}
		
		public static final Uri CONTENT_URI = Uri.parse("content://" + 
				FriendProvider.AUTHORITY + "/friends");
		
		public static final Uri CLIENT_URI = Uri.parse("content://" + 
				FriendProvider.AUTHORITY + "/friends/client");
		
		
		public static final String CONTENT_TYPE = "vnd.android.cursor.dir/vnd.mypackage.friends";
		
		public static final String FRIEND_ID = "_id";
		
		public static final String INFO = "info";
	}
}
