package com.example.foodwasteapp.ui.recipe.listeners;

import com.example.foodwasteapp.ui.recipe.POJO.RecipeDetailsResponse;

public interface RecipeDetailsListener {

    void didFetch(RecipeDetailsResponse response, String message);
    void didError(String message);

}