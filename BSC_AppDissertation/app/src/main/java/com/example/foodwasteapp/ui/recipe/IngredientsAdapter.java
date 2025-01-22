package com.example.foodwasteapp.ui.recipe;

import android.content.Context;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ImageView;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.recyclerview.widget.RecyclerView;

import com.example.foodwasteapp.ui.recipe.POJO.ExtendedIngredient;
import com.example.foodwasteapp.R;
import com.squareup.picasso.Picasso;

import java.util.List;

public class IngredientsAdapter extends RecyclerView.Adapter<IngredientsViewHolder> {

    Context context;
    List<ExtendedIngredient> list;

    public IngredientsAdapter(Context context, List<ExtendedIngredient> list) {

        this.context = context;
        this.list = list;

    }

    @NonNull
    @Override
    public IngredientsViewHolder onCreateViewHolder(@NonNull ViewGroup parent, int viewType) {

        return new IngredientsViewHolder(LayoutInflater.from(context).inflate(R.layout.ingredients_adapter, parent, false));

    }

    @Override
    public void onBindViewHolder(@NonNull IngredientsViewHolder holder, int position) {

        holder.ingredients_name.setText(list.get(position).name);
        holder.ingredients_name.setSelected(true);
        holder.ingredients_quantity.setText(list.get(position).original);
        holder.ingredients_quantity.setSelected(true);
        Picasso.get().load("https://spoonacular.com/cdn/ingredients_100x100/"+list.get(position).image).into(holder.ingredients_image);

    }

    @Override
    public int getItemCount() {

        return list.size();

    }

}

class IngredientsViewHolder extends RecyclerView.ViewHolder {

    TextView ingredients_name, ingredients_quantity;
    ImageView ingredients_image;

    public IngredientsViewHolder(@NonNull View itemView) {

        super(itemView);
        findView();

    }

    private void findView() {

        ingredients_name = itemView.findViewById(R.id.TV_ingredients_name);
        ingredients_quantity = itemView.findViewById(R.id.TV_ingredients_quantity);
        ingredients_image = itemView.findViewById(R.id.IV_ingredients);

    }

}