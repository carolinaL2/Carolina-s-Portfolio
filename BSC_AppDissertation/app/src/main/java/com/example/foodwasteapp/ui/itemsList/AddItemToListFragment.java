package com.example.foodwasteapp.ui.itemsList;

import android.app.AlertDialog;
import android.app.DatePickerDialog;
import android.content.DialogInterface;
import android.graphics.Color;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.DatePicker;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.cardview.widget.CardView;
import androidx.fragment.app.Fragment;
import androidx.lifecycle.ViewModelProvider;
import androidx.navigation.Navigation;

import com.example.foodwasteapp.R;
import com.example.foodwasteapp.ui.dateRemindersService.ScheduleClient;
import com.example.foodwasteapp.singletonVM;
import com.google.android.material.textfield.TextInputEditText;

import java.util.Calendar;
import java.util.Objects;

/*
    This class is responsible for adding items to the at home stocked item list.
    It allows the user to input the name, quantity, storage type and expiry date of the item.
    The user can also scan the NFC tag to input the data automatically.
 */

public class AddItemToListFragment extends Fragment {

    //Declare variables
    AlertDialog.Builder builder;
    private ScheduleClient scheduleClient;
    private DatePickerDialog datePickerDialog;
    singletonVM viewModel;
    CardView backB;
    public static final int ThemeHoloLight = 3;
    View view;
    Button fridgeB, freezerB, pantryB, date, saveB;
    String storageType;
    TextInputEditText quantity, name;

    //This method is called when the fragment is created
    //It inflates the layout for this fragment and initializes the views and widget components
    public View onCreateView(@NonNull LayoutInflater inflater,
                             ViewGroup container, Bundle savedInstanceState) {

        view = inflater.inflate(R.layout.add_items_to_list_fragment, container, false);

        // Initialize views and widget components
        findViews();

        // Create an alert dialog to inform the user to scan the NFC tag to input the data automatically
        builder = new AlertDialog.Builder(getActivity());
        builder.setCancelable(true);
        builder.setTitle("Scan the NFC Tag");
        builder.setMessage("Place the NFC tag on the back of your phone to scan it, after that the data will be input automatically.");
        builder.setPositiveButton("Ok", new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {
                // Do nothing
            }
        });
        builder.show();

        backB.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Navigation.findNavController(view).navigate(R.id.action_addItemToListFragment_to_navigation_list);
            }
        });

        // Checking which storage button is clicked and saves it as a string
        fridgeB.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                fridgeB.setBackgroundColor(Color.parseColor("#4BAAC8"));
                storageType = "Fridge";
            }

        });
        freezerB.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                freezerB.setBackgroundColor(Color.parseColor("#4BAAC8"));
                storageType = "Freezer";
            }
        });
        pantryB.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                pantryB.setBackgroundColor(Color.parseColor("#4BAAC8"));
                storageType = "Pantry";
            }
        });

        // Initialize the date picker
        initDatePicker();
        date.setText(getTodaysDate());
        date.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                datePickerDialog.show();
            }
        });

        saveB.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                String quantityT = quantity.getText().toString();
                String dateT = date.getText().toString();
                String nameT = name.getText().toString();
                Bundle bundle = new Bundle();
                bundle.putString("QUANTITY", quantityT);
                bundle.putString("DATE", dateT);
                bundle.putString("STORAGE", storageType);
                bundle.putString("NAME", nameT);
                Navigation.findNavController(view).navigate(R.id.action_addItemToListFragment_to_navigation_list, bundle);

            }

        });

        return view;

    }

    //This method is called when the fragment is created and it returns the current date
    private String getTodaysDate() {

        Calendar cal = Calendar.getInstance();
        int year = cal.get(Calendar.YEAR);
        int month = cal.get(Calendar.MONTH);
        month = month + 1;
        int day = cal.get(Calendar.DAY_OF_MONTH);

        return makeDateString(day, month, year);

    }

    //This method is called when the fragment is created and it returns the date in the format day month year
    @Override
    public void onResume(){
        super.onResume();
    }

    //This method is called when the fragment is created and it returns the date in the format day month year
    private void initDatePicker() {

        DatePickerDialog.OnDateSetListener dateSetListener = new DatePickerDialog.OnDateSetListener() {
            @Override
            public void onDateSet(DatePicker view, int year, int month, int dayOfMonth) {

                month = month + 1;
                String dateT = makeDateString(dayOfMonth, month, year);
                date.setText(dateT);

            }

        };

        Calendar cal = Calendar.getInstance();
        int year = cal.get(Calendar.YEAR);
        int month = cal.get(Calendar.MONTH);
        int day = cal.get(Calendar.DAY_OF_MONTH);
        datePickerDialog = new DatePickerDialog(getActivity(), ThemeHoloLight, dateSetListener, year, month, day);

        if(scheduleClient == null) {

            scheduleClient = new ScheduleClient(getActivity());
            scheduleClient.doBindService();

        }

    }

    //This method is called when the fragment is created and it returns the date in the format day month year
    public String makeDateString(int day, int month, int year) {

        return day + " " + getMonthFormat(month) + " " + year;

    }

    //This method takes the month as an integer and returns the month as a string
    private String getMonthFormat(int month) {

        if(month == 1) {
            return "JAN";
        } else if(month == 2) {
            return "FEB";
        } else if(month == 3) {
            return "MAR";
        } else if(month == 4) {
            return "APR";
        } else if(month == 5) {
            return "MAY";
        } else if(month == 6) {
            return "JUN";
        } else if(month == 7) {
            return "JUL";
        } else if(month == 8) {
            return "AUG";
        } else if(month == 9) {
            return "SEP";
        } else if(month == 10) {
            return "OCT";
        } else if(month == 11) {
            return "NOV";
        } else if(month == 12) {
            return "DEC";
        }

        //default date
        return "JAN";

    }

    /**
     * This method ats as a helped method to initialise the widget's IDs to be used in this fragment
     */
    private void findViews() {

        fridgeB = view.findViewById(R.id.fridgeB);
        freezerB = view.findViewById(R.id.freezerB);
        pantryB = view.findViewById(R.id.pantryB);
        saveB = view.findViewById(R.id.saveB);
        quantity = view.findViewById(R.id.quantityT);
        date = view.findViewById(R.id.dateT);
        name = view.findViewById(R.id.nameTxt);
        backB = view.findViewById(R.id.back_button_add);

    }

    //This method is called when the fragment is created and it returns the date in the format day month year
    @Override
    public void onViewCreated(@NonNull View view, @Nullable Bundle savedInstanceState) {

        super.onViewCreated(view, savedInstanceState);
        viewModel = new ViewModelProvider(requireActivity()).get(singletonVM.class);

        viewModel.getName().observe(getActivity(), item -> {

            Log.d("TEXTTTTT", item);
            name.setText(item);

        });

        viewModel.getQuantity().observe(getActivity(), item -> {

            quantity.setText(item);

        });

        viewModel.getDate().observe(getActivity(), item -> {

            date.setText(item);

        });

    }

    private int getMonthFormat(String month) {

        if(Objects.equals(month, "JAN")) {
            return 1;
        } else if(Objects.equals(month, "FEB")) {
            return 2;
        } else if(Objects.equals(month, "MAR")) {
            return 3;
        } else if(Objects.equals(month, "APR")) {
            return 4;
        } else if(Objects.equals(month, "MAY")) {
            return 5;
        } else if(Objects.equals(month, "JUN")) {
            return 6;
        } else if(Objects.equals(month, "JUL")) {
            return 7;
        } else if(Objects.equals(month, "AUG")) {
            return 8;
        } else if(Objects.equals(month, "SEP")) {
            return 9;
        } else if(Objects.equals(month, "OCT")) {
            return 10;
        } else if(Objects.equals(month, "NOV")) {
            return 11;
        } else if(Objects.equals(month, "DEC")) {
            return 12;
        }

        //default date
        return 1;

    }

    //This method is called when the fragment is stopped and it unbinds the service
    @Override
    public void onStop() {
        // When our activity is stopped ensure we also stop the connection to the service
        // this stops us leaking our activity into the system *bad*
        if(scheduleClient != null) {
            scheduleClient.doUnbindService();
        }
        super.onStop();
    }

    //This method is called when the fragment is paused and it unbinds the service
    @Override
    public void onPause() {
        super.onPause();
        scheduleClient.doUnbindService();
    }

}
