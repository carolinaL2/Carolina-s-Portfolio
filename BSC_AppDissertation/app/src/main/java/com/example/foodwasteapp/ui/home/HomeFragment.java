package com.example.foodwasteapp.ui.home;

import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.cardview.widget.CardView;
import androidx.fragment.app.Fragment;
import androidx.navigation.Navigation;
import androidx.recyclerview.widget.GridLayoutManager;
import androidx.recyclerview.widget.RecyclerView;

import com.example.foodwasteapp.R;
import com.example.foodwasteapp.ui.recipe.ManagerRequest;
import com.example.foodwasteapp.ui.recipe.RecipeAdapter;
import com.example.foodwasteapp.ui.recipe.listeners.RecipeResponseListener;
import com.example.foodwasteapp.ui.recipe.listeners.RecipeClickListener;
import com.example.foodwasteapp.ui.recipe.POJO.RandomRecipeApiResponse;

import java.util.ArrayList;
import java.util.List;

/*
    This class represent the Home fragment, which is the home page.
 */


public class HomeFragment extends Fragment {

    //Declare variables
    View view;
    RecipeAdapter recipeAdapter;
    CardView shareButton, wasteTrackerButton;

    ManagerRequest manager;
    Button shoppingListB;
    RecyclerView recyclerView;
    List<String> tags = new ArrayList<>();

    //This method creates the view of the fragment and sets the listeners for the buttons in the fragment
    //It also calls the method to fetch the random recipes from the API and sets the adapter for the recycler view to display the recipes
    //It also sets the listener for the recipe items in the recycler view to navigate to the individual recipe page when clicked
    public View onCreateView(@NonNull LayoutInflater inflater,
                             ViewGroup container, Bundle savedInstanceState) {

        view = inflater.inflate(R.layout.fragment_home, container, false);

        findViews();

        //Fetch random recipes from the API
        manager = new ManagerRequest(getActivity());
        manager.getRandomRecipes(recipeResponseListener, tags);

        shoppingListB.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Navigation.findNavController(view).navigate(R.id.action_navigation_home_to_shoppingListFragment);
            }
        });

        shareButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Navigation.findNavController(view).navigate(R.id.action_navigation_home_to_sharePage);
            }
        });

        wasteTrackerButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Navigation.findNavController(view).navigate(R.id.action_navigation_home_to_navigation_rewards);
            }
        });

        return view;

    }

    //This method creates the references to the items in the fragment's layout
    private void findViews() {
        wasteTrackerButton = view.findViewById(R.id.rewardsIconButton);
        shoppingListB = view.findViewById(R.id.shoppingListButton);
        shareButton = view.findViewById(R.id.shareIconButton);
        recyclerView = view.findViewById(R.id.RV_Home);
    }

    //This listener is called when the recipes are fetched from the API and sets the adapter for the recycler view to display the recipes
    private final RecipeResponseListener recipeResponseListener = new RecipeResponseListener() {
        @Override
        public void didFetch(RandomRecipeApiResponse response, String message) {

            recyclerView.setHasFixedSize(true);
            recyclerView.setLayoutManager(new GridLayoutManager(getActivity(), 1));
            recipeAdapter = new RecipeAdapter(getActivity(), response.recipes, recipeClickListener);
            recyclerView.setAdapter(recipeAdapter);

        }

        @Override
        public void didError(String message) {

            Toast.makeText(getActivity(), message, Toast.LENGTH_SHORT).show();
            Log.d("ERROR", message);

        }

    };

    //This listener is called when a recipe item in the recycler view is clicked and navigates to the individual recipe page
    private final RecipeClickListener recipeClickListener = new RecipeClickListener() {
        @Override
        public void onRecipeClicked(String id) {

            Bundle bundle = new Bundle();
            bundle.putString("ID", id);
            Navigation.findNavController(view).navigate(R.id.action_navigation_home_to_individualRecipe, bundle);

        }

    };

}