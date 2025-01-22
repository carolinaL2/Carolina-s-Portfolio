package com.example.foodwasteapp.ui.wasteTracker;

import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ProgressBar;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.fragment.app.Fragment;

import com.example.foodwasteapp.R;
import com.harrywhewell.scrolldatepicker.DayScrollDatePicker;
import com.harrywhewell.scrolldatepicker.OnDateSelectedListener;

import org.joda.time.DateTime;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;
import java.util.Locale;
import java.util.Objects;

/*
    This class is responsible for displaying the waste tracker fragment.
    It is used to display the amount of waste that has been wasted by the user daily.
    It also displays the amount of money that has been wasted by the user daily.
    The user can see the amount of waste and money wasted on a specific date by selecting the date from the date picker.
 */

public class WasteTrackerFragment extends Fragment {

    //Declaration of variables and objects used in the class
    AlertDialog.Builder builder;
    double newAmountInt;
    FileOutputStream writer1, writer2;
    private TrackerViewModel viewModel;
    DayScrollDatePicker dayDatePicker;
    TextView amountTxt, moneyTxt;
    double progress, moneyProgress, wasted;
    View view;
    String amountT, newAmount;
    ProgressBar progressDialog, moneyProgressBar;

    //This method is called when the fragment is created and displayed to the user.
    public View onCreateView(@NonNull LayoutInflater inflater,
                             ViewGroup container, Bundle savedInstanceState) {

        view = inflater.inflate(R.layout.waste_tracker_fragment, container, false);

        viewModel = new TrackerViewModel();

        // Show the message to the user when the fragment is created
        showMsg();

        findViews(); // Find the views in the layout

        // Get the current date and set the date picker to the current date by default
        Calendar cal = Calendar.getInstance();
        int year = cal.get(Calendar.YEAR);
        int month = cal.get(Calendar.MONTH);
        month = month + 1;
        int day = cal.get(Calendar.DAY_OF_MONTH);

        // Gets the arguments passed to this fragment
        Bundle bundle = this.getArguments();

        if (bundle != null) {

            wasted = bundle.getDouble("QUANTITY_Double", 0);
            amountT = bundle.getString("QUANTITY_String", null);

        }

        viewModel.addAmount(wasted);

        // Set the date picker to the current date
        dayDatePicker.setStartDate(day, month, year);
        dayDatePicker.getSelectedDate(new OnDateSelectedListener() {
            @Override
            public void onDateSelected(@Nullable Date date) {

                // Get the day, month and year of the selected date
                int dateDay = new DateTime(date).getDayOfMonth();
                int dateMonth = new DateTime(date).getMonthOfYear();
                int dateYear = new DateTime(date).getYear();
                String newDate = dateDay + "" + dateMonth + dateYear;

                // Check if the selected date is today's date
                if(Objects.equals(isToday(), newDate)) {

                    updateProgressBar();
                    updateMoneyProgressBar();

                }

            }

        });

        // Load the content of the file and update the amount of waste and money wasted
        loadContent();

        return view;

    }

    //This method is used to get the reference of the widgets used in the layout
    private void findViews() {

        dayDatePicker = view.findViewById(R.id.dayDatePicker);
        progressDialog = view.findViewById(R.id.progressBar);
        amountTxt = view.findViewById(R.id.textViewProgress);
        moneyProgressBar = view.findViewById(R.id.progressBarMoney);
        moneyTxt = view.findViewById(R.id.textViewMoney);

    }

    //This method is used to update the progress bar with the amount of waste wasted by the user daily
    private void updateProgressBar() {

        int maxValue = progressDialog.getMax();
        maxValue = 196;
        progressDialog.setMax(maxValue);
        wasted = Double.parseDouble(amountT);
        progress = wasted * 100;
        progressDialog.setProgress((int) progress);
        amountTxt.setText(amountT);

    }

    //This method is used to update the progress bar with the amount of money wasted by the user daily
    private void updateMoneyProgressBar() {

        int maxValue = moneyProgressBar.getMax();
        maxValue = 200;
        moneyProgressBar.setMax(maxValue);
        wasted = Double.parseDouble(amountT);
        moneyProgress = wasted * 100;
        moneyProgressBar.setProgress((int) moneyProgress);
        moneyTxt.setText(amountT);

    }

    //This method is used to get the current date in the format of day, month and year as a string
    private String isToday() {

        Calendar cal = Calendar.getInstance();
        int year = cal.get(Calendar.YEAR);
        int month = cal.get(Calendar.MONTH);
        month = month + 1;
        int day = cal.get(Calendar.DAY_OF_MONTH);

        return makeDateString(day, month, year);

    }

    //This method is used to make a date string from the day, month and year passed as arguments to the method
    public String makeDateString(int day, int month, int year) {

        return day + "" + month + year;

    }

    //This method is used to show a message to the user when the fragment is created
    //The message is displayed in a pop up dialog box
    //The message is displayed to the user to inform the user about the functionality of the fragment and how to use it
    public void showMsg() {

        //Adding a pop up message
        builder = new AlertDialog.Builder(getActivity());
        builder.setCancelable(true);
        builder.setTitle("Choose the date");
        builder.setMessage("Click on today's date to activate the tracker and see your daily waste.");

        builder.setPositiveButton("Ok", new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {

                // Do nothing

            }

        });

        builder.show();

    }

    //This method is used to load the content of the file and update the amount of waste and money wasted by the user
    //The content of the file is read and the amount of waste and money wasted is updated
    public void loadContent() {

        File path = getContext().getFilesDir();
        File readFrom = new File(path, "shoppingList.txt");
        byte[] content = new byte[(int) readFrom.length()];
        FileInputStream stream = null;

        File path1 = getContext().getFilesDir();
        File readFrom1 = new File(path1, "storeDate.txt");
        byte[] content1 = new byte[(int) readFrom1.length()];
        FileInputStream stream1 = null;
        try {
            stream = new FileInputStream(readFrom);
            stream.read(content);

            newAmount = new String(content);
            Log.d("newAmount", newAmount);
            newAmountInt = Double.parseDouble(newAmount);
            Log.d("newAmountInt", String.valueOf(newAmountInt));
            newAmountInt = newAmountInt + wasted;
            Log.d("newAmountInt", "addition = " + String.valueOf(newAmountInt));
            newAmount = String.valueOf(newAmountInt);
            Log.d("newAmount", "convertback = " + newAmount);
            amountT = newAmount;

            stream1 = new FileInputStream(readFrom1);
            stream1.read(content1);

            Calendar cal = Calendar.getInstance();
            String dayStored = new String(content1);
            int dayInt = Integer.parseInt(dayStored);
            if(dayInt != cal.get(Calendar.DAY_OF_MONTH)) {

                amountT = "0.0";
                Log.d("HEREEEEE22222", "HERE");

            }
            Log.d("dayStored", dayStored);
            Log.d("dayInt", String.valueOf(dayInt));
            Log.d("cal", String.valueOf(cal.get(Calendar.DAY_OF_MONTH)));


        } catch (Exception e) {
            e.printStackTrace();

        }

    }

    //This method is called when the fragment is paused
    //The amount of waste wasted by the user is saved in the file when the fragment is paused
    @Override
    public void onPause() {

        File path1 = getContext().getFilesDir();
        try {
            writer1 = new FileOutputStream(new File(path1, "shoppingList.txt"));
            writer1.write(amountT.getBytes());
            writer1.close();
        } catch (Exception e) {
            e.printStackTrace();
        }

        Calendar cal = Calendar.getInstance();
        int dayStored = cal.get(Calendar.DAY_OF_MONTH);

        File path2 = getContext().getFilesDir();
        try {
            writer2 = new FileOutputStream(new File(path2, "storeDate.txt"));
            writer2.write(dayStored);
            writer2.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
        super.onPause();

    }

}