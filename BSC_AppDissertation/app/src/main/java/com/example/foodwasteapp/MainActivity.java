package com.example.foodwasteapp;

import android.app.PendingIntent;
import android.content.Intent;
import android.content.IntentFilter;
import android.nfc.NdefMessage;
import android.nfc.NdefRecord;
import android.nfc.NfcAdapter;
import android.os.Bundle;
import android.os.Parcelable;
import android.util.Log;
import android.view.Window;
import android.view.WindowManager;

import com.google.android.material.bottomnavigation.BottomNavigationView;

import androidx.appcompat.app.AppCompatActivity;
import androidx.lifecycle.ViewModelProvider;
import androidx.navigation.NavController;
import androidx.navigation.Navigation;
import androidx.navigation.ui.AppBarConfiguration;
import androidx.navigation.ui.NavigationUI;

import com.example.foodwasteapp.databinding.ActivityMainBinding;

import java.util.Arrays;

/*
    This class is the main activity of the application. It is responsible for setting up the bottom navigation bar and handling NFC data.
 */

public class MainActivity extends AppCompatActivity {

    // Declare variables for the binding, view model, and NFC data elements.
    private ActivityMainBinding binding;
    private singletonVM viewModel;
    String name, quantity, date, nfcText;
    PendingIntent pendingIntent;
    IntentFilter[] readFilters;

    // This method creates the activity and sets up the bottom navigation bar.
    // It also sets up the NFC adapter and processes the NFC data.
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        binding = ActivityMainBinding.inflate(getLayoutInflater());

        requestWindowFeature(Window.FEATURE_NO_TITLE);
        this.getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN, WindowManager.LayoutParams.FLAG_FULLSCREEN);
        getSupportActionBar().hide(); //This line hides the action bar
        setContentView(binding.getRoot());

        //Creating a bottom navigation bar
        BottomNavigationView navView = findViewById(R.id.nav_view);

        //Initialising the view model
        viewModel = new ViewModelProvider(this).get(singletonVM.class);

        // Set up the navigation bar by checking for unique IDs
        AppBarConfiguration appBarConfiguration = new AppBarConfiguration.Builder(
                R.id.navigation_list,
                R.id.navigation_tracker,
                R.id.navigation_home,
                R.id.navigation_recipes,
                R.id.navigation_settings)
                .build();

        // Set up the navigation controller for the fragment's graph navigation
        NavController navController = Navigation.findNavController(this, R.id.nav_host_fragment_activity_main);
        NavigationUI.setupActionBarWithNavController(this, navController, appBarConfiguration);
        NavigationUI.setupWithNavController(binding.navView, navController);

        // Set up the NFC adapter and process the NFC data
        // The NFC data is split into separate strings for the name, quantity, and expiry date of the item
        try {

            Intent intent = new Intent(this, getClass());
            intent.addFlags(Intent.FLAG_ACTIVITY_SINGLE_TOP);

            pendingIntent = PendingIntent.getActivity(this, 0, intent, PendingIntent.FLAG_UPDATE_CURRENT);

            IntentFilter textFilter = new IntentFilter(NfcAdapter.ACTION_NDEF_DISCOVERED, "text/plain");

            readFilters = new IntentFilter[]{textFilter};

        } catch (IntentFilter.MalformedMimeTypeException e) {

            throw new RuntimeException(e);

        }

        // Process the NFC data by splitting it into separate strings for the name, quantity, and expiry date of the item.
        processNFC(getIntent());

    }

    // This method enables the NFC adapter to read data from the NFC tag.
    // It uses the foreground dispatch system to read the data.
    private void enableRead() {

        NfcAdapter.getDefaultAdapter(this).enableForegroundDispatch(this, pendingIntent, readFilters, null);

    }

    // This method disables the NFC adapter from reading data from the NFC tag.
    // It disables the foreground dispatch system.
    private void disableRead() {

        NfcAdapter.getDefaultAdapter(this).disableForegroundDispatch(this);

    }

    // This method resumes the activity and enables the NFC adapter to read data from the NFC tag.
    // It calls the enableRead() method.
    @Override
    protected void onResume() {

        super.onResume();
        enableRead();

    }

    // This method pauses the activity and disables the NFC adapter from reading data from the NFC tag.
    // It calls the disableRead() method.
    @Override
    protected void onPause() {

        super.onPause();
        disableRead();

    }

    // This method processes the NFC data when a new intent is received.
    // It calls the processNFC() method.
    @Override
    protected void onNewIntent(Intent intent) {

        super.onNewIntent(intent);
        processNFC(intent);

    }

    // This method processes the NFC data by decoding the messages on the intent,
    // thus, asking the intent for the list of messages encoded on it.
    // It then decodes the messages and records, and checks the record.
    private void processNFC(Intent intent) {

        Parcelable[] messages = intent.getParcelableArrayExtra(NfcAdapter.EXTRA_NDEF_MESSAGES);

        if(messages != null) {

            for(Parcelable message : messages) {

                NdefMessage ndefMessage = (NdefMessage) message;
                for(NdefRecord record : ndefMessage.getRecords()) {

                    switch(record.getTnf()) {

                        case NdefRecord.TNF_WELL_KNOWN:

                            if(Arrays.equals(record.getType(), NdefRecord.RTD_TEXT)) {

                                nfcText = new String(record.getPayload());
                                splitText(nfcText);

                            }

                    }

                }

            }

        }

    }

    // This method splits the NFC text into separate strings for the name, quantity, and expiry date of the item.
    // It then sets the name, quantity, and expiry date of the item in the view model.
    private void splitText(String nfcText) {

        String[] arr = nfcText.split("\n");

        for (int i = 0; i < arr.length; i++) {

            Log.d("ARRRAYYY", arr[i]);

            name = arr[0];
            quantity = arr[1];
            date = arr[2];

        }

        if(name != null && name.contains("en")) {

            name = name.replace("en", "");

        }

        viewModel.setName(name);
        viewModel.setQuantity(quantity);
        viewModel.setDate(date);

    }

}