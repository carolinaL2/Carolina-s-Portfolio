package com.example.foodwasteapp.ui.recipe;

import android.content.Context;

import com.example.foodwasteapp.R;
import com.example.foodwasteapp.ui.recipe.listeners.RecipeResponseListener;
import com.example.foodwasteapp.ui.recipe.listeners.RecipeDetailsListener;
import com.example.foodwasteapp.ui.recipe.POJO.RandomRecipeApiResponse;
import com.example.foodwasteapp.ui.recipe.POJO.RecipeDetailsResponse;

import java.util.List;

import retrofit2.Call;
import retrofit2.Callback;
import retrofit2.Response;
import retrofit2.Retrofit;
import retrofit2.converter.gson.GsonConverterFactory;
import retrofit2.http.GET;
import retrofit2.http.Path;
import retrofit2.http.Query;

public class ManagerRequest {

        Context context;
        Retrofit retrofit = new Retrofit.Builder()
                .baseUrl("https://api.spoonacular.com/")
                .addConverterFactory(GsonConverterFactory.create())
                .build();

        public ManagerRequest(Context context) {

            this.context = context;

        }

        public void getRandomRecipes(RecipeResponseListener listener, List<String> tags) {

                CallRandomRecipes callRandomRecipes = retrofit.create(CallRandomRecipes.class);
                Call<RandomRecipeApiResponse> call = callRandomRecipes.callRandomRecipe(
                        context.getString(R.string.SpoonacularAPI), "10", tags);
                call.enqueue(new Callback<RandomRecipeApiResponse>() {
                        @Override
                        public void onResponse(Call<RandomRecipeApiResponse> call, Response<RandomRecipeApiResponse> response) {

                                if(!response.isSuccessful()) {

                                        listener.didError(response.message());
                                        return;

                                }

                                listener.didFetch(response.body(), response.message());

                        }

                        @Override
                        public void onFailure(Call<RandomRecipeApiResponse> call, Throwable t) {

                                listener.didError(t.getMessage());

                        }

                });

        }

        public void getRecipeDetails(RecipeDetailsListener listener, int id) {

                CallRecipeInformation callRecipeInformation = retrofit.create(CallRecipeInformation.class);
                Call<RecipeDetailsResponse> call = callRecipeInformation.callRecipeDetails(id, context.getString(R.string.SpoonacularAPI), 2);
                call.enqueue(new Callback<RecipeDetailsResponse>() {
                        @Override
                        public void onResponse(Call<RecipeDetailsResponse> call, Response<RecipeDetailsResponse> response) {

                                if(!response.isSuccessful()) {

                                        listener.didError(response.message());
                                        return;

                                }

                                listener.didFetch(response.body(), response.message());

                        }

                        @Override
                        public void onFailure(Call<RecipeDetailsResponse> call, Throwable t) {

                                listener.didError(t.getMessage());

                        }

                });

        }

        private interface CallRandomRecipes {

                @GET("recipes/random")
                Call<RandomRecipeApiResponse> callRandomRecipe(

                        @Query("apiKey") String apiKey,
                        @Query("number") String number,
                        @Query("tags") List<String> tags

                );

        }

        private interface CallRecipeInformation {

                @GET("recipes/{id}/information")
                Call<RecipeDetailsResponse> callRecipeDetails (

                        @Path("id") int id,
                        @Query("apiKey") String apiKey,
                        @Query("ranking") int ranking

                );

        }

}