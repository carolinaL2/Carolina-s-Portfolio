package com.example.foodwasteapp.ui.recipe;

import android.content.Context;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ImageView;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.cardview.widget.CardView;
import androidx.recyclerview.widget.RecyclerView;

import com.example.foodwasteapp.R;
import com.example.foodwasteapp.ui.recipe.listeners.RecipeClickListener;
import com.example.foodwasteapp.ui.recipe.POJO.Recipe;
import com.squareup.picasso.Picasso;

import java.util.List;

/*

 */

public class RecipeAdapter extends RecyclerView.Adapter<RecipeViewHolder> {

    Context context;
    List<Recipe> list;
    RecipeClickListener listener;

    public RecipeAdapter(Context context, List<Recipe> list, RecipeClickListener listener) {

        this.context = context;
        this.list = list;
        this.listener = listener;

    }

    @NonNull
    @Override
    public RecipeViewHolder onCreateViewHolder(@NonNull ViewGroup parent, int viewType) {

        return new RecipeViewHolder(LayoutInflater.from(context).inflate(R.layout.individual_recipe_adapter, parent, false));

    }

    @Override
    public void onBindViewHolder(@NonNull RecipeViewHolder holder, int position) {

        holder.TV_title.setText(list.get(position).title);
        holder.TV_title.setSelected(true);
        holder.TV_likes.setText(list.get(position).aggregateLikes+" Likes");
        holder.TV_servings.setText(list.get(position).servings+" Servings");
        holder.TV_time.setText(list.get(position).readyInMinutes+" Minutes");
        Picasso.get().load(list.get(position).image).into(holder.imageView);

        holder.cardView.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                listener.onRecipeClicked(String.valueOf(list.get(holder.getAbsoluteAdapterPosition()).id));

            }

        });

    }

    @Override
    public int getItemCount() {

        return list.size();

    }

}

class RecipeViewHolder extends RecyclerView.ViewHolder {

    CardView cardView;
    ImageView imageView;
    TextView TV_title, TV_servings, TV_likes, TV_time;

    public RecipeViewHolder(@NonNull View itemView) {

        super(itemView);
        cardView = itemView.findViewById(R.id.RecipesByIngredientContainer);

        imageView = itemView.findViewById(R.id.imageView_food);
        TV_title = itemView.findViewById(R.id.textView_title);
        TV_servings = itemView.findViewById(R.id.textView_amount);
        TV_likes = itemView.findViewById(R.id.textView_type);
        TV_time = itemView.findViewById(R.id.textView_description);

    }

}