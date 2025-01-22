package com.example.foodwasteapp.ui.itemsList;

import android.app.AlertDialog;
import android.content.DialogInterface;
import android.graphics.Color;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.constraintlayout.widget.ConstraintLayout;
import androidx.fragment.app.Fragment;
import androidx.lifecycle.ViewModelProvider;
import androidx.navigation.Navigation;
import androidx.recyclerview.widget.ItemTouchHelper;
import androidx.recyclerview.widget.LinearLayoutManager;
import androidx.recyclerview.widget.RecyclerView;
import java.util.Calendar;

import com.example.foodwasteapp.R;
import com.example.foodwasteapp.ui.database.DateDB.DateItem;
import com.example.foodwasteapp.ui.database.DateDB.DateItemViewModel;
import com.example.foodwasteapp.ui.database.Item;
import com.example.foodwasteapp.ui.database.ItemListAdapter;
import com.example.foodwasteapp.ui.database.ItemViewModel;
import com.example.foodwasteapp.ui.wasteTracker.TrackerViewModel;

import java.util.ArrayList;
import java.util.Date;
import java.util.Objects;

/*
    A class that represents the fragment for the list of items in the app.
    This class is responsible for displaying the list of items in the app,
    and allows the user to delete items from the list by swiping left on the item.
    The user can also add new items to the list by clicking on the "Add Item" button.
    The class also contains a pop-up message that explains how to delete an item from the list.
 */

public class ItemsListFragment extends Fragment {

    // Declare variables
    View view;
    double numberInput;
    private ItemViewModel itemViewModel;
    DateItemViewModel dateItemViewModel;
    AlertDialog.Builder builder;
    ItemListAdapter adapter;
    ConstraintLayout constraintLayout;
    private RecyclerView recyclerView;
    Button addItemB;
    int wasteCounter, itemCount;
    String nameT, quantityT, storageType, dateExpiry, textInput;
    TrackerViewModel trackerViewModel;

    public View onCreateView(@NonNull LayoutInflater inflater,
                             ViewGroup container, Bundle savedInstanceState) {

        // Inflate the layout for this fragment
        view = inflater.inflate(R.layout.items_list_fragment, container, false);

        showMsg();

        Bundle bundle = this.getArguments();
        constraintLayout = view.findViewById(R.id.constraint_layout);

        trackerViewModel = new TrackerViewModel();

        if (bundle != null) {

            nameT = bundle.getString("NAME", null);
            storageType = bundle.getString("STORAGE", null);
            dateExpiry = bundle.getString("DATE", null);
            quantityT = bundle.getString("QUANTITY", null);

        }

        Bundle nBundle = new Bundle();
        nBundle.putString("QUANTITY", quantityT);
        nBundle.putString("DATE", dateExpiry);
        nBundle.putString("STORAGE", storageType);
        nBundle.putString("NAME", nameT);

        // Initialise and set up ViewModel for managing location data
        itemViewModel = ViewModelProvider.AndroidViewModelFactory.getInstance(
                getActivity().getApplication()).create(ItemViewModel.class);

        recyclerView = view.findViewById(R.id.recyclerViewUpdate);

        // Set up the recycler view for displaying the reminders as a list
        recyclerView.setLayoutManager(new LinearLayoutManager(getActivity()));

        // Call the Location entity of the Room database to:
        // convert latitude and longitude to a formatted degree representation,
        // and to obtain all data needed to be inserted into this database, such as,
        // reminder's name, description, location's latitude and longitude.
        Item item = new Item(nameT, storageType, dateExpiry, quantityT);
        // Insert all data into the Room database.
        itemViewModel.insertItem(item);

        // Initialise and set up ViewModel for managing location data
        dateItemViewModel = ViewModelProvider.AndroidViewModelFactory.getInstance(
                getActivity().getApplication()).create(DateItemViewModel.class);

        // Call the Location entity of the Room database to:
        // convert latitude and longitude to a formatted degree representation,
        // and to obtain all data needed to be inserted into this database, such as,
        // reminder's name, description, location's latitude and longitude.
        DateItem dateItem = new DateItem(dateExpiry);
        // Insert all data into the Room database.
        dateItemViewModel.insertItem(dateItem);

        // Observe changes in location data and update the RecyclerView accordingly
        dateItemViewModel.getAllDatesFromVm().observe(getActivity(), result ->
        {

            if (result != null && !result.isEmpty()) {

                for (DateItem date : result) {

                    String dateStr = date.getDateItem();  // Assuming you have a getDate() method in your DateItem class
                    Log.d("DATESTR", dateStr);

                }

            }

        });

        // Observe changes in location data and update the RecyclerView accordingly
        itemViewModel.getAllItemsFromVm().observe(getActivity(), result ->
        {

            if (result != null && !result.isEmpty()) {

                adapter = new ItemListAdapter((ArrayList<Item>) result);
                itemCount = adapter.getItemCount();
                Log.d("ITEM_COUNT", "Item count = " + String.valueOf(itemCount));
                recyclerView.setAdapter(adapter);

            }

        });

        // Initialize views and widget components
        addItemB = view.findViewById(R.id.addItemBUpdate);

        addItemB.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                // Navigate to the desired destination fragment using NavController
                Navigation.findNavController(v).navigate(R.id.action_navigation_list_to_addItemToListFragment);

            }

        });

        // Creating the ItemTouchHelper object
        ItemTouchHelper helper = new ItemTouchHelper(callback);
        helper.attachToRecyclerView(recyclerView);

        return view;

    }

    ItemTouchHelper.SimpleCallback callback = new ItemTouchHelper.SimpleCallback(0, ItemTouchHelper.LEFT) {

        @Override
        public boolean onMove(@NonNull RecyclerView recyclerView, @NonNull RecyclerView.ViewHolder viewHolder, @NonNull RecyclerView.ViewHolder target) {

            return false;

        }

        @Override
        public void onSwiped(@NonNull RecyclerView.ViewHolder viewHolder, int direction) {

            buildAlertMsg(viewHolder);

            wasteCounter++; // Increment waste counter
            Log.d("WASTEDDD", "wasteCounter: " + wasteCounter);

        }

    };

    public void buildAlertMsg(RecyclerView.ViewHolder viewHolder) {

        EditText quantityInKG = new EditText(getActivity());

        //Adding a pop up alert when item is deleted
        builder = new AlertDialog.Builder(getActivity());
        builder.setCancelable(true);
        builder.setTitle("Why are you deleting this item?");
        builder.setMessage("If you are deleting this item because you are wasting it, then please enter the amount being wasted in KG.");

        quantityInKG.setHint("Enter amount in KG");
        quantityInKG.setMaxLines(1);
        quantityInKG.setHeight(140);
        builder.setView(quantityInKG);

        builder.setPositiveButton("Consumed", new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {

                // Remove item from list
                int position = viewHolder.getAdapterPosition();
                Item item = adapter.getItemPosition(position);
                Toast.makeText(getActivity(), "Deleting " +
                        item.getItemName(), Toast.LENGTH_LONG).show();

                itemViewModel.deleteItem(item);

                // Observe changes in location data and update the RecyclerView accordingly
                itemViewModel.getAllItemsFromVm().observe(getActivity(), result ->
                {

                    if (result != null && !result.isEmpty()) {

                        adapter = new ItemListAdapter((ArrayList<Item>) result);
                        recyclerView.setAdapter(adapter);

                    }

                });

            }

        });

        builder.setNegativeButton("Wasted", new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {

                numberInput = Double.parseDouble(quantityInKG.getText().toString());
                textInput = quantityInKG.getText().toString();
                trackerViewModel.setAmount(numberInput);
                trackerViewModel.setTxtAmount(textInput);

                Bundle rewardsBundle = new Bundle();
                rewardsBundle.putDouble("QUANTITY_Double", trackerViewModel.getAmount());
                rewardsBundle.putString("QUANTITY_String", trackerViewModel.getTxtAmount());

                // Remove item from list
                int position = viewHolder.getAdapterPosition();
                Item item = adapter.getItemPosition(position);
                Toast.makeText(getActivity(), "Deleting " +
                        item.getItemName(), Toast.LENGTH_LONG).show();

                itemViewModel.deleteItem(item);

                // Observe changes in location data and update the RecyclerView accordingly
                itemViewModel.getAllItemsFromVm().observe(getActivity(), result ->
                {

                    if (result != null && !result.isEmpty()) {

                        adapter = new ItemListAdapter((ArrayList<Item>) result);
                        recyclerView.setAdapter(adapter);

                    }

                });

                Navigation.findNavController(view).navigate(R.id.action_navigation_list_to_navigation_rewards, rewardsBundle);

            }

        });

        AlertDialog dialog = builder.show();
        Button negativeButton = dialog.getButton(DialogInterface.BUTTON_NEGATIVE);
        negativeButton.setTextColor(Color.RED);

    }

    private Date getDate(int year, int month, int day){

        Calendar cal = Calendar.getInstance();
        cal.set(Calendar.YEAR, year);
        cal.set(Calendar.MONTH, month-1);
        cal.set(Calendar.DAY_OF_MONTH, day);
        return cal.getTime();

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

    @Override
    public void onDestroyView() {
        super.onDestroyView();

    }

    public void showMsg() {

        //Adding a pop up message
        builder = new AlertDialog.Builder(getActivity());
        builder.setCancelable(true);
        builder.setTitle("Deleting a food item");
        builder.setMessage("To remove a food item from the list, swipe left on the item.");

        builder.setPositiveButton("Ok", new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {

                // Do nothing

            }

        });

        builder.show();

    }

}