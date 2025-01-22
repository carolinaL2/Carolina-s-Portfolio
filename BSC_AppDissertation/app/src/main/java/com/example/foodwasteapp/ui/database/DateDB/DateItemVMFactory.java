package com.example.foodwasteapp.ui.database.DateDB;

import android.app.Application;

import androidx.annotation.NonNull;
import androidx.lifecycle.ViewModel;
import androidx.lifecycle.ViewModelProvider;

public class DateItemVMFactory implements ViewModelProvider.Factory {

    private final Application application;

    public DateItemVMFactory(Application myApplication) {

        application = myApplication;

    }

    @NonNull
    @Override
    public <T extends ViewModel> T create(@NonNull Class<T> modelClass) {

        return (T) new DateItemViewModel(application);

    }

}