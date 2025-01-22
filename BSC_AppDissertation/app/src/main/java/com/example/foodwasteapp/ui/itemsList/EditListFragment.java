package com.example.foodwasteapp.ui.itemsList;

import static com.example.foodwasteapp.ui.itemsList.AddItemToListFragment.ThemeHoloLight;

import android.app.DatePickerDialog;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.DatePicker;
import android.widget.EditText;

import androidx.annotation.NonNull;
import androidx.fragment.app.Fragment;
import androidx.navigation.Navigation;

import com.example.foodwasteapp.R;

import java.util.Calendar;

/*
    This class is responsible for editing the items in the list.
 */

public class EditListFragment extends Fragment {

    //Declares variables for the class
    View view;
    String nameT, quantityT, dateT, storageType;
    private DatePickerDialog datePickerDialog;

    EditText storageTxt, quantityTxt, nameTxt;
    Button saveButton, dateTxt;

    public View onCreateView(@NonNull LayoutInflater inflater,
                             ViewGroup container, Bundle savedInstanceState) {

        view = inflater.inflate(R.layout.edit_items_list_fragment, container, false);

        // Initialize views and widget components
        findViews();

        Bundle bundle = this.getArguments();

        if (bundle != null) {

            nameT = bundle.getString("ITEMNAME", null);
            storageType = bundle.getString("ITEMSTORAGE", null);
            dateT = bundle.getString("ITEMDATE", null);
            quantityT = bundle.getString("ITEMQUANTITY", null);

            Log.d("ITEMNAME", nameT);
            Log.d("ITEMSTORAGE", storageType);
            Log.d("ITEMDATE", dateT);
            Log.d("ITEMQUANTITY", quantityT);

        }

        nameTxt.setText(nameT);
        quantityTxt.setText(quantityT);
        storageTxt.setText(storageType);

        initDatePicker();
        dateTxt.setText(dateT);
        dateTxt.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                datePickerDialog.show();
            }
        });

        saveButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Bundle nBundle = new Bundle();
                nBundle.putString("QUANTITY", quantityT);
                nBundle.putString("DATE", dateT);
                nBundle.putString("STORAGE", storageType);
                nBundle.putString("NAME", nameT);
                Navigation.findNavController(view).navigate(R.id.action_editListFragment_to_navigation_list, nBundle);
            }
        });

        return view;

    }

    // This method is responsible for finding the views in the layout file
    private void findViews() {

        dateTxt = view.findViewById(R.id.reminderTV);
        nameTxt = view.findViewById(R.id.nameItemTV);
        storageTxt = view.findViewById(R.id.storageTV);
        quantityTxt = view.findViewById(R.id.quantityTV);
        saveButton = view.findViewById(R.id.saveItemEdit);

    }

    // This method is responsible for initializing the date picker
    private void initDatePicker() {

        DatePickerDialog.OnDateSetListener dateSetListener = new DatePickerDialog.OnDateSetListener() {
            @Override
            public void onDateSet(DatePicker view, int year, int month, int dayOfMonth) {

                month = month + 1;
                String dateT = makeDateString(dayOfMonth, month, year);
                dateTxt.setText(dateT);

            }

        };

        Calendar cal = Calendar.getInstance();
        int year = cal.get(Calendar.YEAR);
        int month = cal.get(Calendar.MONTH);
        int day = cal.get(Calendar.DAY_OF_MONTH);
        datePickerDialog = new DatePickerDialog(getActivity(), ThemeHoloLight, dateSetListener, year, month, day);

    }

    // This method is responsible for making the date string
    public String makeDateString(int day, int month, int year) {

        return day + " " + getMonthFormat(month) + " " + year;

    }

    // This method is responsible for getting the month format as a string
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

}