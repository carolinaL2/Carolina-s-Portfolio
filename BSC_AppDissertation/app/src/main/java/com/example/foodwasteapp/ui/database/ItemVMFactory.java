package com.example.foodwasteapp.ui.database;

import android.app.Application;

import androidx.annotation.NonNull;
import androidx.lifecycle.ViewModel;
import androidx.lifecycle.ViewModelProvider;

public class ItemVMFactory implements ViewModelProvider.Factory {

    private final Application application;

    public ItemVMFactory(Application myApplication) {

        application = myApplication;

    }

    @NonNull
    @Override
    public <T extends ViewModel> T create(@NonNull Class<T> modelClass) {

        return (T) new ItemViewModel(application);

    }

}