package mypackage.FriendTracker;

import java.util.HashMap;

import mypackage.FriendTracker.Friend.Friends;

import android.content.ContentProvider;
import android.content.ContentUris;
import android.content.ContentValues;
import android.content.Context;
import android.content.UriMatcher;
import android.database.Cursor;
import android.database.SQLException;
import android.database.sqlite.SQLiteDatabase;
import android.database.sqlite.SQLiteOpenHelper;
import android.database.sqlite.SQLiteQueryBuilder;
import android.net.Uri;
import android.util.Log;

public class FriendProvider extends ContentProvider {

	public static final Uri CONTENT_URI = 
		Uri.parse("content://mypackage.friendtracker.friendprovider");

	private static final String TAG = "NotesContentProvider"; 

	private static final String DATABASE_NAME = "notes.db";

	private static final int DATABASE_VERSION = 1;

	private static final String FRIENDS_TABLE_NAME = "friends";

	public static final String AUTHORITY = "mypackage.friendtracker.friendprovider";

	private static final UriMatcher sUriMatcher;

	private static final int FRIENDS = 1;

	private static HashMap<String, String> friendsProjectionMap;

	private static class DatabaseHelper extends SQLiteOpenHelper {

		DatabaseHelper(Context context) {
			super(context, DATABASE_NAME, null, DATABASE_VERSION);
		}

		@Override
		public void onCreate(SQLiteDatabase db) {
			db.execSQL("CREATE TABLE " + FRIENDS_TABLE_NAME + " (" + Friends.FRIEND_ID 
					+ " INTEGER PRIMARY KEY AUTOINCREMENT," + Friends.INFO
					+ " LONGTEXT" + ");");
		}

		@Override
		public void onUpgrade(SQLiteDatabase db, int oldVersion, int newVersion) {
			Log.w(TAG, "Upgrading database from version " + oldVersion + " to " + newVersion
					+ ", which will destroy all old data");
			db.execSQL("DROP TABLE IF EXISTS " + FRIENDS_TABLE_NAME);
			onCreate(db);
		}
	}

	private DatabaseHelper dbHelper;

	@Override
	public int delete(Uri uri, String where, String[] whereArgs) {
		SQLiteDatabase db = dbHelper.getWritableDatabase();
		int count;
		switch (sUriMatcher.match(uri)) {
		case FRIENDS:
			count = db.delete(FRIENDS_TABLE_NAME, where, whereArgs);
			break;
			
		default:
			throw new IllegalArgumentException("Unknown URI " + uri);
		}

		getContext().getContentResolver().notifyChange(uri, null);
		return count;
	}

	@Override
	public String getType(Uri uri) {
		switch (sUriMatcher.match(uri)) {
		case FRIENDS:
			return Friends.CONTENT_TYPE;
		default:
			throw new IllegalArgumentException("Unknown URI " + uri);
		}
	}

	@Override
	public Uri insert(Uri uri, ContentValues initialValues) {
		if (sUriMatcher.match(uri) != FRIENDS) { throw new IllegalArgumentException("Unknown URI " + uri); }

		ContentValues values;
		if (initialValues != null) {
			values = new ContentValues(initialValues);
		} else {
			values = new ContentValues();
		}

		SQLiteDatabase db = dbHelper.getWritableDatabase();
		long rowId = db.insert(FRIENDS_TABLE_NAME, Friends.INFO, values);
		if (rowId > 0) {
			Uri noteUri = ContentUris.withAppendedId(Friends.CONTENT_URI, rowId);
			getContext().getContentResolver().notifyChange(noteUri, null);
			return noteUri;
		}

		throw new SQLException("Failed to insert row into " + uri);
	}

	@Override
	public boolean onCreate() {
		dbHelper = new DatabaseHelper(getContext());
		return true;
	}

	@Override
	public Cursor query(Uri uri, String[] projection, String selection, String[] selectionArgs, String sortOrder) {
		SQLiteQueryBuilder qb = new SQLiteQueryBuilder();

		switch (sUriMatcher.match(uri)) {
		case FRIENDS:
			qb.setTables(FRIENDS_TABLE_NAME);
			qb.setProjectionMap(friendsProjectionMap);
			break;

		default:
			throw new IllegalArgumentException("Unknown URI " + uri);
		}

		SQLiteDatabase db = dbHelper.getReadableDatabase();
		Cursor c = qb.query(db, projection, selection, selectionArgs, null, null, sortOrder);

		c.setNotificationUri(getContext().getContentResolver(), uri);
		return c;
	}

	@Override
	public int update(Uri uri, ContentValues values, String where, String[] whereArgs) {
		SQLiteDatabase db = dbHelper.getWritableDatabase();
		int count;
		switch (sUriMatcher.match(uri)) {
		case FRIENDS:
			count = db.update(FRIENDS_TABLE_NAME, values, where, whereArgs);
			break;

		default:
			throw new IllegalArgumentException("Unknown URI " + uri);
		}

		getContext().getContentResolver().notifyChange(uri, null);
		return count;
	}

	static {
		sUriMatcher = new UriMatcher(UriMatcher.NO_MATCH);
		sUriMatcher.addURI(AUTHORITY, FRIENDS_TABLE_NAME, FRIENDS);

		friendsProjectionMap = new HashMap<String, String>();
		friendsProjectionMap.put(Friends.FRIEND_ID, Friends.FRIEND_ID);
		friendsProjectionMap.put(Friends.INFO, Friends.INFO);
	}
}