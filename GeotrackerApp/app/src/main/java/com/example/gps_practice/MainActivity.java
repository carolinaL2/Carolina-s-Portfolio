package com.example.gps_practice;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;
import androidx.fragment.app.Fragment;
import androidx.fragment.app.FragmentManager;
import androidx.fragment.app.FragmentTransaction;

import android.Manifest;
import android.content.pm.PackageManager;
import android.os.Bundle;

import com.example.gps_practice.databinding.ActivityMainBinding;
import com.example.gps_practice.fragments.HomeFragment;
import com.example.gps_practice.fragments.MapFragment;
import com.example.gps_practice.fragments.SettingsFragment;
import com.example.gps_practice.fragments.RemindersFragment;

public class MainActivity extends AppCompatActivity {

    private static final int PERMISSIONS_REQUEST_ACCESS_FINE_LOCATION = 1;
    private static final int PERMISSIONS_REQUEST_ACCESS_COARSE_LOCATION = 2;
    private boolean locationPermissionGranted;

    //Binding of fragments for navigation bar
    ActivityMainBinding binding;

    @Override
    protected void onCreate(Bundle savedInstanceState) {

        super.onCreate(savedInstanceState);
        //Binding
        binding = ActivityMainBinding.inflate(getLayoutInflater());
        // Set the layout file as the content view.
        setContentView(binding.getRoot());
        getFragment(new HomeFragment());

        getLocationPermission();

        binding.navBar.setOnItemSelectedListener(item -> {

            if (item.getItemId() == R.id.home) {

                getFragment(new HomeFragment());
                return (true);

            } else if (item.getItemId() == R.id.locationMap) {

                getFragment(new MapFragment());
                return (true);

            } else if (item.getItemId() == R.id.reminders) {

                getFragment(new RemindersFragment());
                return (true);

            } else if (item.getItemId() == R.id.settings) {

                getFragment(new SettingsFragment());
                return (true);

            } else {

                return false;

            }

        });

    }

    private void getFragment(Fragment fragment) {

        FragmentManager fragmentManager = getSupportFragmentManager();
        FragmentTransaction fragmentTransaction = fragmentManager.beginTransaction();
        fragmentTransaction.replace(R.id.frame_layout, fragment);
        fragmentTransaction.commit();

    }

    /*
     * Request location permission, so that we can get the location of the
     * device. The result of the permission request is handled by a callback,
     * onRequestPermissionsResult.
     */
    private void getLocationPermission() {

        if ((ContextCompat.checkSelfPermission(this.getApplicationContext(),
                android.Manifest.permission.ACCESS_FINE_LOCATION)
                == PackageManager.PERMISSION_GRANTED)
                && (ContextCompat.checkSelfPermission(this.getApplicationContext(),
                Manifest.permission.ACCESS_COARSE_LOCATION)
                == PackageManager.PERMISSION_GRANTED)) {

                locationPermissionGranted = true;

        } else {

                ActivityCompat.requestPermissions(this,
                    new String[]{android.Manifest.permission.ACCESS_FINE_LOCATION},
                    PERMISSIONS_REQUEST_ACCESS_FINE_LOCATION);

            ActivityCompat.requestPermissions(this,
                    new String[]{Manifest.permission.ACCESS_COARSE_LOCATION},
                    PERMISSIONS_REQUEST_ACCESS_COARSE_LOCATION);

        }

    }

    // Location permissions
    @Override
    public void onRequestPermissionsResult(int requestCode,
                                           @NonNull String[] permissions,
                                           @NonNull int[] grantResults) {

                locationPermissionGranted = false;

                if ((requestCode == PERMISSIONS_REQUEST_ACCESS_FINE_LOCATION)
                    || (requestCode == PERMISSIONS_REQUEST_ACCESS_COARSE_LOCATION)) {

                        // If request is cancelled, the result arrays are empty.
                        if (grantResults.length > 0
                            && grantResults[0] == PackageManager.PERMISSION_GRANTED) {

                                locationPermissionGranted = true;

                        }

                } else {

                    super.onRequestPermissionsResult(requestCode, permissions, grantResults);

                }

    }

}