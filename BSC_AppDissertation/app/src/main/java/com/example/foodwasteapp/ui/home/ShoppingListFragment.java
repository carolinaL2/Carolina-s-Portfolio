package com.example.foodwasteapp.ui.home;

import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.AdapterView;
import android.widget.EditText;
import android.widget.ImageView;
import android.widget.ListView;
import androidx.annotation.NonNull;
import androidx.cardview.widget.CardView;
import androidx.fragment.app.Fragment;
import androidx.navigation.Navigation;
import com.example.foodwasteapp.R;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.util.ArrayList;
import java.util.Arrays;

/*
    This class is used to display the user's shopping list.
 */

public class ShoppingListFragment extends Fragment {

    //Declares variables
    View view;
    ListView listView;
    ArrayList<String> items;
    ShoppingListAdapter adapter;
    EditText editText;
    CardView backB;
    ImageView imageView;

    //Creates the view for the shopping list fragment
    public View onCreateView(@NonNull LayoutInflater inflater,
                             ViewGroup container, Bundle savedInstanceState) {

        view = inflater.inflate(R.layout.shopping_list_fragment, container, false);

        findViews();

        //Initializes the list view and array list
        items = new ArrayList<>();
        backB.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Navigation.findNavController(view).navigate(R.id.action_shoppingListFragment_to_navigation_home);
            }
        });

        //Adds an item to the shopping list when the user clicks on the add button
        imageView.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                String text = editText.getText().toString();
                addItem(text);
                editText.setText("");
            }
        });

        //Deletes an item from the shopping list when the user long clicks on it
        listView.setOnItemLongClickListener(new AdapterView.OnItemLongClickListener() {
            @Override
            public boolean onItemLongClick(AdapterView<?> parent, View view, int position, long id) {
                items.remove(position);
                listView.setAdapter(adapter);
                return true;
            }

        });

        //Sets the adapter for the shopping list view
        adapter = new ShoppingListAdapter(getContext(), items);
        listView.setAdapter(adapter);

        //Loads the content of the shopping list
        loadContent();

        return view;

    }

    //Finds the layout components for the shopping list fragment and assigns them to variables
    private void findViews() {
        editText = view.findViewById(R.id.input);
        imageView = view.findViewById(R.id.add);
        backB = view.findViewById(R.id.back_button_shop);
        listView = view.findViewById(R.id.listView_shopping);
    }

    //Adds an item to the shopping list
    private void addItem(String text) {

        items.add(text);
        listView.setAdapter(adapter);

    }

    //Loads the content of the shopping list
    //The content is stored in a file called shoppingList.txt
    public void loadContent() {

        File path = getContext().getFilesDir();
        File readFrom = new File(path, "shoppingList.txt");
        byte[] content = new byte[(int) readFrom.length()];
        FileInputStream stream = null;
        try {
            stream = new FileInputStream(readFrom);
            stream.read(content);
            String s = new String(content);
            s = s.substring(1, s.length() - 1);
            String split[] = s.split(", ");

            items = new ArrayList<>(Arrays.asList(split));
            adapter = new ShoppingListAdapter(getContext(), items);
            listView.setAdapter(adapter);
        } catch (Exception e) {
            e.printStackTrace();

        }

    }

    //Saves the content of the shopping list when the user leaves the fragment 
    @Override
    public void onPause() {

        File path = getContext().getFilesDir();
        try {
            FileOutputStream writer = new FileOutputStream(new File(path, "shoppingList.txt"));
            writer.write(items.toString().getBytes());
            writer.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
        super.onPause();

    }

}