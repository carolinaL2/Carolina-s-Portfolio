package com.example.foodwasteapp.ui.recipe;

import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.cardview.widget.CardView;
import androidx.fragment.app.Fragment;
import androidx.navigation.Navigation;
import androidx.recyclerview.widget.LinearLayoutManager;
import androidx.recyclerview.widget.RecyclerView;

import com.example.foodwasteapp.ui.recipe.POJO.RecipeDetailsResponse;
import com.example.foodwasteapp.R;
import com.example.foodwasteapp.ui.recipe.listeners.RecipeDetailsListener;
import com.squareup.picasso.Picasso;

public class IndividualRecipeFragment extends Fragment {

    int id;
    View view;
    TextView TV_meal_name, TV_meal_source, TV_meal_description;
    RecyclerView recyclerViewIngredients;
    ImageView TV_meal_image;
    ManagerRequest manager;
    IngredientsAdapter ingredientsAdapter;
    CardView backB;

    public View onCreateView(@NonNull LayoutInflater inflater,
                             ViewGroup container, Bundle savedInstanceState) {

        view = inflater.inflate(R.layout.individual_recipe_fragment, container, false);

        findViews();

        Bundle bundle = this.getArguments();

        if (bundle != null) {

            String idStr = bundle.getString("ID", null);
            id = Integer.parseInt(idStr);

        }

        manager = new ManagerRequest(getActivity());
        manager.getRecipeDetails(listener, id);

        return view;

    }

    private void findViews() {

        recyclerViewIngredients = view.findViewById(R.id.recycler_ingredients);
        TV_meal_image = view.findViewById(R.id.meal_image);
        TV_meal_name = view.findViewById(R.id.meal_name_TV);
        TV_meal_source = view.findViewById(R.id.meal_source_TV);
        TV_meal_description = view.findViewById(R.id.meal_summary_TV);
        backB = view.findViewById(R.id.back_button_ingredients);

    }

    private final RecipeDetailsListener listener = new RecipeDetailsListener() {
        @Override
        public void didFetch(RecipeDetailsResponse response, String message) {

            TV_meal_name.setText(response.title);
            TV_meal_source.setText(response.sourceName);
            String strippedText = response.summary.replaceAll("(?s)<[^>]*>(\\s*<[^>]*>)*", " ");
            TV_meal_description.setText(strippedText);
            Picasso.get().load(response.image).into(TV_meal_image);

            recyclerViewIngredients.setHasFixedSize(true);
            recyclerViewIngredients.setLayoutManager(new LinearLayoutManager(getActivity(), LinearLayoutManager.HORIZONTAL, false));
            ingredientsAdapter = new IngredientsAdapter(getActivity(), response.extendedIngredients);
            recyclerViewIngredients.setAdapter(ingredientsAdapter);

            backB.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {

                    Navigation.findNavController(view).navigate(R.id.action_individualRecipe_to_recipeFragment);

                }

            });

        }

        @Override
        public void didError(String message) {

            Toast.makeText(getActivity(), message, Toast.LENGTH_SHORT).show();

        }

    };

}