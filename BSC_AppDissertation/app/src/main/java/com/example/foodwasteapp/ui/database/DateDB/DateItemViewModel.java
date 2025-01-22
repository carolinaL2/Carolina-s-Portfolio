package com.example.foodwasteapp.ui.database.DateDB;

import android.app.Application;

import androidx.lifecycle.AndroidViewModel;
import androidx.lifecycle.LiveData;

import com.example.foodwasteapp.ui.database.Item;

import java.util.Date;
import java.util.List;

public class DateItemViewModel extends AndroidViewModel {

    private DateDatabaseRepository itemRepository;
    private final LiveData<List<DateItem>> datesLiveData;

    public DateItemViewModel(Application application) {

        super(application);
        // Creates the repository inside the constructor
        itemRepository = new DateDatabaseRepository(application);
        datesLiveData = itemRepository.getAllDates();

    }

    public LiveData<List<DateItem>> getAllDatesFromVm() {

        return datesLiveData;

    }

    public void insertItem(DateItem item) {

        itemRepository.insertItem(item);

    }

    public void deleteItem(DateItem item) {

        itemRepository.deleteItem(item);

    }

}