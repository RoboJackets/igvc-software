package mypackage.FriendTracker;

import android.net.Uri;
import android.provider.BaseColumns;

public class MyFriends {

    public static final String AUTHORITY = "myPackage.FriendTracker.provider.Friends";

    // BaseColumn contains _id.
    public static final class Friend implements BaseColumns {

        public static final Uri    CONTENT_URI = Uri.parse("content://myPackage.FriendTracker.provider.Friends");

        // Table column
        public static final String  FRIEND_ID   = "FRIEND_ID";
        public static final String  FRIEND_LAT   = "FRIEND_LAT";
        public static final String  FRIEND_LON   = "FRIEND_LON";
    }
}
