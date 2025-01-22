package com.example.foodwasteapp.ui.database.DateDB;

import android.content.Context;
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
import com.example.foodwasteapp.ui.database.Item;
import com.example.foodwasteapp.ui.database.ItemDatabase;
import com.example.foodwasteapp.ui.database.ItemViewModel;

import java.util.ArrayList;

public class DateItemListAdapter extends RecyclerView.Adapter<DateItemViewHolder> {

    ArrayList<DateItem> itemsArrayList;
    private DateItemViewModel itemViewModel;
    DateItemDatabase database;
    Context context;
    ArrayList<String> descriptionArrayList;

    public DateItemListAdapter(ArrayList<DateItem> items) {

        this.itemsArrayList = items;

    }

    @NonNull
    @Override
    public DateItemViewHolder onCreateViewHolder(@NonNull ViewGroup parent, int viewType) {
        return null;
    }

    @Override
    public void onBindViewHolder(@NonNull DateItemViewHolder parent, int position) {

    }

    @Override
    public int getItemCount() {

        return itemsArrayList.size();

    }

    public DateItem getItemPosition(int position) {

        return itemsArrayList.get(position);

    }

}

    class DateItemViewHolder extends RecyclerView.ViewHolder {

        public DateItemViewHolder(@NonNull View itemView) {

            super(itemView);

        }

    }