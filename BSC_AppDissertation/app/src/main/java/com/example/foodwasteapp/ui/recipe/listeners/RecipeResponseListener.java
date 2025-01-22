package com.example.foodwasteapp.ui.recipe.listeners;

import com.example.foodwasteapp.ui.recipe.POJO.RandomRecipeApiResponse;

public interface RecipeResponseListener {

    void didFetch(RandomRecipeApiResponse response, String message);
    void didError(String message);

}