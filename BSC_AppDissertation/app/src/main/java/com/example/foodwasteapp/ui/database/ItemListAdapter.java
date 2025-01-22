package com.example.foodwasteapp.ui.database;

import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ImageButton;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.navigation.Navigation;
import androidx.recyclerview.widget.RecyclerView;

import com.example.foodwasteapp.R;

import java.util.ArrayList;

public class ItemListAdapter extends RecyclerView.Adapter<ItemListAdapter.ItemViewHolder> {

    ArrayList<Item> itemsArrayList;

    public ItemListAdapter(ArrayList<Item> items) {

        this.itemsArrayList = items;

    }

    @NonNull
    @Override
    public ItemViewHolder onCreateViewHolder(@NonNull ViewGroup parent, int viewType) {

        LayoutInflater layoutInflater = LayoutInflater.from(parent.getContext());
        View view = layoutInflater.inflate(R.layout.items_list_order, parent, false);
        return new ItemViewHolder(view);

    }

    @Override
    public void onBindViewHolder(@NonNull ItemViewHolder holder, int position) {

        final Item items = itemsArrayList.get(position);

        holder.itemName.setText(String.format("%s", itemsArrayList.get(position).itemName));
        holder.itemDate.setText(String.format("%s", "Expiry date: " + itemsArrayList.get(position).itemExpiryDate));

        holder.editB.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                Bundle bundle = new Bundle();
                bundle.putString("ITEMNAME", String.format("%s", itemsArrayList.get(position).itemName));
                bundle.putString("ITEMSTORAGE", String.format("%s", itemsArrayList.get(position).itemDescription));
                bundle.putString("ITEMQUANTITY", String.format("%s", itemsArrayList.get(position).itemQuantity));
                bundle.putString("ITEMDATE", String.format("%s", itemsArrayList.get(position).itemExpiryDate));
                Navigation.findNavController(v).navigate(R.id.action_navigation_list_to_editListFragment, bundle);

            }

        });

    }

    @Override
    public int getItemCount() {

        return itemsArrayList.size();

    }

    public Item getItemPosition(int position) {

        return itemsArrayList.get(position);

    }

    static class ItemViewHolder extends RecyclerView.ViewHolder {

        // TextViews to display reminder name, coordinates, and description.
        TextView itemName, itemDate;
        ImageButton editB;

        public ItemViewHolder(@NonNull View itemView) {

            super(itemView);
            findViews();

        }

        private void findViews() {

            itemName = itemView.findViewById(R.id.item_name);
            itemDate = itemView.findViewById(R.id.item_descrip);
            editB = itemView.findViewById(R.id.editButton);

        }

    }

}