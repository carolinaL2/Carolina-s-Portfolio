package com.example.gps_practice.fragments;

import android.Manifest;
import android.annotation.SuppressLint;
import android.app.PendingIntent;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.os.Bundle;

import androidx.activity.result.ActivityResultCallback;
import androidx.activity.result.ActivityResultLauncher;
import androidx.activity.result.contract.ActivityResultContracts;
import androidx.annotation.NonNull;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;
import androidx.fragment.app.Fragment;

import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;

import com.example.gps_practice.Geofencing.GeofenceHelper;
import com.example.gps_practice.R;
import com.google.android.gms.common.api.ApiException;
import com.google.android.gms.location.FusedLocationProviderClient;
import com.google.android.gms.location.Geofence;
import com.google.android.gms.location.GeofenceStatusCodes;
import com.google.android.gms.location.GeofencingClient;
import com.google.android.gms.location.GeofencingRequest;
import com.google.android.gms.location.LocationServices;
import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.CircleOptions;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.MarkerOptions;
import com.google.android.gms.tasks.OnFailureListener;
import com.google.android.gms.tasks.OnSuccessListener;
import com.google.android.libraries.places.api.Places;
import com.google.android.libraries.places.api.net.PlacesClient;

import java.util.Arrays;
import java.util.concurrent.atomic.AtomicBoolean;

public class MapFragment extends Fragment implements OnMapReadyCallback {

    private GoogleMap mMap;

    // The entry point to the Places API.
    private PlacesClient placesClient;

    // The entry point to the Fused Location Provider. 
    private FusedLocationProviderClient fusedLocation;
    private GeofencingClient geofencingClient;
    private float GEOFENCE_RADIUS = 200F;
    private String GEOFENCE_ID = "ID";
    private GeofenceHelper geofenceHelper;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {

        // Inflate the layout for this fragment
        View view = inflater.inflate(R.layout.fragment_map, container, false);
        SupportMapFragment supportMapFragment = (SupportMapFragment)
                getChildFragmentManager().findFragmentById(R.id.map_container);
        supportMapFragment.getMapAsync(this);

        geofencingClient = LocationServices.getGeofencingClient(getActivity());
        geofenceHelper = new GeofenceHelper(getActivity());

        // Construct a PlacesClient
        Places.initialize(getActivity().getApplicationContext(), getString(R.string.MAPS_API_KEY));
        placesClient = Places.createClient(getActivity());

        // Construct a FusedLocationProviderClient
        fusedLocation = LocationServices.getFusedLocationProviderClient(getActivity());

        return view;

    }

    // Get a handle to the GoogleMap object and display marker.
    @Override
    public void onMapReady(GoogleMap mMap) {

        LatLng latLng = new LatLng(20, -10);
        mMap.setMapType(GoogleMap.MAP_TYPE_HYBRID);
        mMap.moveCamera(CameraUpdateFactory.newLatLngZoom(latLng, 8));

        mMap.setOnMapLongClickListener(new GoogleMap.OnMapLongClickListener() {
            @Override
            public void onMapLongClick(@NonNull LatLng latLng) {

                mMap.clear();
                MarkerOptions markerOptions = new MarkerOptions();
                //Set marker's position and title
                mMap.addMarker(new MarkerOptions()
                        .position(latLng)
                        .title("Location Clicked"));
                CircleOptions circleOptions = new CircleOptions();
                circleOptions.center(latLng);
                circleOptions.radius(GEOFENCE_RADIUS);
                circleOptions.strokeColor(Color.argb(255, 255, 0, 0));
                circleOptions.fillColor(Color.argb(64, 255, 0, 0));
                circleOptions.strokeWidth(4);
                mMap.addCircle(circleOptions);
                addGeofence(latLng, GEOFENCE_RADIUS);

            }

        });

        // 1. Check if permissions are granted,
        // if so, enable the device's location layer
        if (ActivityCompat.checkSelfPermission(getActivity(),
                Manifest.permission.ACCESS_FINE_LOCATION)
                == PackageManager.PERMISSION_GRANTED
                && ActivityCompat.checkSelfPermission(getActivity(),
                Manifest.permission.ACCESS_COARSE_LOCATION)
                == PackageManager.PERMISSION_GRANTED) {

            mMap.setMyLocationEnabled(true);

        }

//        mMap.setOnMapClickListener(new GoogleMap.OnMapClickListener() {
//            @Override
//            public void onMapClick(@NonNull LatLng latLng) {
//
//                //When a location is clicked on map
//                //Initialise marker options
//                MarkerOptions markerOptions = new MarkerOptions();
//                //Set marker's position and title
//                mMap.addMarker(new MarkerOptions()
//                        .position(latLng)
//                        .title("Marker"));
//                //Remove all markers
//                mMap.clear();
//                //Animating to zoom the marker
//                mMap.moveCamera(CameraUpdateFactory.newLatLngZoom(latLng, 10));
//                //Set map type to hybrid
//                mMap.setMapType(GoogleMap.MAP_TYPE_HYBRID);
//
//            }
//
//        });

    }

    // Add a geofence
    private void addGeofence(LatLng latLng, float radius) {

        Geofence geofence = geofenceHelper.getGeofence(GEOFENCE_ID, latLng, radius, Geofence.GEOFENCE_TRANSITION_ENTER);
        GeofencingRequest geofencingRequest = geofenceHelper.getGeofencingRequest(geofence);
        PendingIntent pendingIntent = geofenceHelper.getPendingIntent();

        if (ActivityCompat.checkSelfPermission(getActivity(),
                Manifest.permission.ACCESS_FINE_LOCATION)
                == PackageManager.PERMISSION_GRANTED) {

            geofencingClient.addGeofences(geofencingRequest, pendingIntent)
                    .addOnSuccessListener(new OnSuccessListener<Void>() {
                        @Override
                        public void onSuccess(Void unused) {

                            Log.d("MAPFragment", "OnSuccess: ");

                        }

                    })
                    .addOnFailureListener(new OnFailureListener() {
                        @Override
                        public void onFailure(@NonNull Exception e) {

                            // Error in case geofence cannot be entered
                            String errorMessage = geofenceHelper.getError(e);
                            Log.d("MAPFragment", "OnFailure: " + errorMessage);

                        }

                    });

        }

    }

}