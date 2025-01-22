package com.example.foodwasteapp.ui.home;

import android.app.Activity;
import android.content.Context;
import android.view.LayoutInflater;
import android.view.View;
import android.widget.ArrayAdapter;
import android.widget.ImageView;
import android.widget.TextView;

import androidx.annotation.NonNull;

import com.example.foodwasteapp.R;

import java.util.ArrayList;

/*
    ShoppingListAdapter class is an ArrayAdapter class that is used to display the shopping list items in the ShoppingListFragment.
    It extends the ArrayAdapter class and overrides the getView method to display the shopping list items in the list view.
    The getView method inflates the shopping_item_order layout and sets the name of the item in the text view.
    It also sets an onClickListener on the delete icon to remove the item from the list when clicked.
 */

public class ShoppingListAdapter extends ArrayAdapter<String> {

    //Declare variables
    ArrayList<String> list;
    Context context;

    //Constructor
    public ShoppingListAdapter(Context context, ArrayList<String> list) {

        super(context, R.layout.shopping_item_order, list);
        this.list = list;
        this.context = context;

    }

    //Override getView method to display the shopping list items in the list view
    //Inflate the shopping_item_order layout and set the name of the item in the text view
    //Set an onClickListener on the delete icon to remove the item from the list when clicked
    @NonNull
    @Override
    public View getView(int position, View convertView, @NonNull android.view.ViewGroup parent) {

        if(convertView == null) {

            LayoutInflater layoutInflater = (LayoutInflater) context.getSystemService(Activity.LAYOUT_INFLATER_SERVICE);
            convertView = layoutInflater.inflate(R.layout.shopping_item_order, null);

            TextView name = convertView.findViewById(R.id.nameShop);
            name.setText(list.get(position));

            ImageView delete = convertView.findViewById(R.id.deleteIcon);
            delete.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {

                    list.remove(position);
                    notifyDataSetChanged();

                }

            });

        }

        return convertView;

    }

}