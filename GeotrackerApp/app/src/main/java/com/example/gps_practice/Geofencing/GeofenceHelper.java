package com.example.gps_practice.Geofencing;

import android.app.PendingIntent;
import android.content.Context;
import android.content.ContextWrapper;
import android.content.Intent;
import android.content.pm.LabeledIntent;

import com.google.android.gms.common.api.ApiException;
import com.google.android.gms.location.Geofence;
import com.google.android.gms.location.GeofenceStatusCodes;
import com.google.android.gms.location.GeofencingRequest;
import com.google.android.gms.maps.model.LatLng;

public class GeofenceHelper extends ContextWrapper {

    private static final String TAG = "GeofenceHelper";
    PendingIntent pendingIntent;

    public GeofenceHelper(Context base) {

      super(base);

    }

    public GeofencingRequest getGeofencingRequest(Geofence geofence) {

        return new GeofencingRequest.Builder()
                .addGeofence(geofence)
                .setInitialTrigger(GeofencingRequest.INITIAL_TRIGGER_ENTER)
                .build();

    }

    public Geofence getGeofence(String ID, LatLng latLng, float radius, int transitionTypes) {

        return new Geofence.Builder()
                .setCircularRegion(latLng.latitude, latLng.longitude, radius)
                .setRequestId(ID)
                .setTransitionTypes(transitionTypes)
                .setLoiteringDelay(5000)
                .setExpirationDuration(Geofence.NEVER_EXPIRE)
                .build();

    }

    public PendingIntent getPendingIntent() {

        if(pendingIntent != null) {

            return pendingIntent;

        } else {

            Intent intent = new Intent(this, GeofenceBroadcastReceiver.class);
            pendingIntent = PendingIntent.getBroadcast(this, 1, intent,
                    PendingIntent.FLAG_MUTABLE);

        }

        return pendingIntent;

    }

    public String getError(Exception e) {

        if(e instanceof ApiException) {

            ApiException apiException = (ApiException) e;
            switch (apiException.getStatusCode()) {

                case GeofenceStatusCodes.GEOFENCE_NOT_AVAILABLE:
                    return "GEOFENCE_NOT_AVAILABLE";
                case GeofenceStatusCodes.GEOFENCE_TOO_MANY_GEOFENCES:
                    return "GEOFENCE_TOO_MANY_GEOFENCES";
                case GeofenceStatusCodes.GEOFENCE_TOO_MANY_PENDING_INTENTS:
                    return "GEOFENCE_TOO_MANY_PENDING_INTENTS";

            }

        }

        return e.getLocalizedMessage();
    }

}
