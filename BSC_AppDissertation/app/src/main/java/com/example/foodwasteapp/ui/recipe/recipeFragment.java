package com.example.foodwasteapp.ui.recipe;

import android.app.AlertDialog;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Spinner;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.appcompat.widget.SearchView;
import androidx.fragment.app.Fragment;
import androidx.navigation.Navigation;
import androidx.recyclerview.widget.GridLayoutManager;
import androidx.recyclerview.widget.RecyclerView;

import com.example.foodwasteapp.R;
import com.example.foodwasteapp.ui.recipe.listeners.RecipeResponseListener;
import com.example.foodwasteapp.ui.recipe.listeners.RecipeClickListener;
import com.example.foodwasteapp.ui.recipe.POJO.RandomRecipeApiResponse;

import java.util.ArrayList;
import java.util.List;

/*
    This class is the main recipes fragment.
 */

public class recipeFragment extends Fragment {

    //Declare variables
    View view;
    AlertDialog dialog;
    ManagerRequest manager;
    RecipeAdapter recipeAdapter;
    RecyclerView recyclerView;
    List<String> tags = new ArrayList<>();
    Spinner spinner;
    androidx.appcompat.widget.SearchView searchView;

    public View onCreateView(@NonNull LayoutInflater inflater,
                             ViewGroup container, Bundle savedInstanceState) {

        view = inflater.inflate(R.layout.recipes_fragment, container, false);

        dialog = new AlertDialog.Builder(getActivity()).create();
        dialog.setTitle("Loading ...");

        searchView = view.findViewById(R.id.searchBar_home);
        searchView.setOnQueryTextListener(new SearchView.OnQueryTextListener() {
            @Override
            public boolean onQueryTextSubmit(String query) {

                tags.clear();
                tags.add(query);
                manager.getRandomRecipes(recipeResponseListener, tags);
                dialog.show();
                return true;

            }

            @Override
            public boolean onQueryTextChange(String newText) {

                return false;

            }

        });

        spinner = view.findViewById(R.id.spinner_tags);
        ArrayAdapter arrayAdapter = ArrayAdapter.createFromResource(
           getActivity(), R.array.tags, R.layout.spinner_text
        );
        arrayAdapter.setDropDownViewResource(R.layout.spinner_inner_text);
        spinner.setAdapter(arrayAdapter);
        spinner.setOnItemSelectedListener(spinnerSelectedListener);

        manager = new ManagerRequest(getActivity());

        return view;

    }

    private final RecipeResponseListener recipeResponseListener = new RecipeResponseListener() {
        @Override
        public void didFetch(RandomRecipeApiResponse response, String message) {

            dialog.dismiss();
            recyclerView = view.findViewById(R.id.RecipesRecyclerView);
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

    private final AdapterView.OnItemSelectedListener spinnerSelectedListener = new AdapterView.OnItemSelectedListener() {
        @Override
        public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {

            tags.clear();
            tags.add(parent.getSelectedItem().toString());
            manager.getRandomRecipes(recipeResponseListener, tags);
            dialog.show();

        }

        @Override
        public void onNothingSelected(AdapterView<?> parent) {



        }

    };

    private final RecipeClickListener recipeClickListener = new RecipeClickListener() {
        @Override
        public void onRecipeClicked(String id) {

                Bundle bundle = new Bundle();
                bundle.putString("ID", id);
                Navigation.findNavController(view).navigate(R.id.action_recipeFragment_to_individualRecipe, bundle);

        }

    };

}