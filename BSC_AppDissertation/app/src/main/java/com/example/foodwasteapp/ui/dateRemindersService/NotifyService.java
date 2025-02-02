package com.example.foodwasteapp.ui.dateRemindersService;

import android.R;
import android.app.Notification;
import android.app.NotificationManager;
import android.app.PendingIntent;
import android.app.Service;
import android.content.Intent;
import android.content.res.Resources;
import android.os.Binder;
import android.os.IBinder;
import android.util.Log;

import com.example.foodwasteapp.ui.itemsList.ItemsListFragment;

/**
 * This service is started when an Alarm has been raised
 *
 * We pop a notification into the status bar for the user to click on
 * When the user clicks the notification a new activity is opened
 *
 * @author paul.blundell
 */
public class NotifyService extends Service {

    Notification myNotication;

    /**
     * Class for clients to access
     */
    public class ServiceBinder extends Binder {
        NotifyService getService() {
            return NotifyService.this;
        }
    }

    // Unique id to identify the notification.
    private static final int NOTIFICATION = 123;
    // Name of an intent extra we can use to identify if this service was started to create a notification
    public static final String INTENT_NOTIFY = "INTENT_NOTIFY";
    // The system notification manager
    private NotificationManager mNM;

    @Override
    public void onCreate() {
        Log.i("NotifyService", "onCreate()");
        mNM = (NotificationManager) getSystemService(NOTIFICATION_SERVICE);
    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {
        Log.i("LocalService", "Received start id " + startId + ": " + intent);

        // If this service was started by out AlarmTask intent then we want to show our notification
        if(intent.getBooleanExtra(INTENT_NOTIFY, false))
            showNotification();

        // We don't care if this service is stopped as we have already delivered our notification
        return START_NOT_STICKY;
    }

    @Override
    public IBinder onBind(Intent intent) {
        return mBinder;
    }

    // This is the object that receives interactions from clients
    private final IBinder mBinder = new ServiceBinder();

    /**
     * Creates a notification and shows it in the OS drag-down status bar
     */
    private void showNotification() {
//        // This is the 'title' of the notification
//        CharSequence title = "Alarm!!";
//        // This is the icon to use on the notification
//        int icon = R.drawable.ic_dialog_alert;
//        // This is the scrolling text of the notification
//        CharSequence text = "Your notification time is upon us.";
//        // What time to show on the notification
//        long time = System.currentTimeMillis();
//
//        Notification notification = new Notification(icon, text, time);
//
//        // The PendingIntent to launch our activity if the user selects this notification
//        PendingIntent contentIntent = PendingIntent.getActivity(this, 0, new Intent(this, ShoppingListFragment.class), PendingIntent.FLAG_IMMUTABLE);
//
//        // Set the info for the views that show in the notification panel.
//        notification.setLatestEventInfo(this, title, text, contentIntent);
//
//        // Clear the notification when it is pressed
//        notification.flags |= Notification.FLAG_AUTO_CANCEL;
//
//        // Send the notification to the system.
//        mNM.notify(NOTIFICATION, notification);

        Intent notificationIntent = new Intent(this, ItemsListFragment.class);
        PendingIntent contentIntent = PendingIntent.getActivity(this,
                1001, notificationIntent,
                PendingIntent.FLAG_IMMUTABLE);

        Resources res = this.getResources();
        Notification.Builder builder = new Notification.Builder(this);

        builder.setContentIntent(contentIntent)
                .setSmallIcon(R.drawable.ic_dialog_alert)
                .setWhen(System.currentTimeMillis())
                .setAutoCancel(true)
                .setContentTitle("Notification")
                .setContentText("you got a notification");
        Notification n = builder.build();

        mNM.notify(1, n);

        // Stop the service when we are finished
        stopSelf();
    }
}