package com.example.foodwasteapp.ui.database;

import android.app.Application;

import androidx.lifecycle.AndroidViewModel;
import androidx.lifecycle.LiveData;

import java.util.List;

public class ItemViewModel extends AndroidViewModel {

    private DatabaseRepository itemRepository;
    private final LiveData<List<Item>> listLiveData;

    public ItemViewModel(Application application) {

        super(application);
        // Creates the repository inside the constructor
        itemRepository = new DatabaseRepository(application);
        // Initialise the Live Data using the repository
        listLiveData = itemRepository.getAllItems();

    }

    public LiveData<List<Item>> getAllItemsFromVm() {

        return listLiveData;

    }

    public void insertItem(Item item) {

        itemRepository.insertItem(item);

    }

    public void deleteItem(Item item) {

        itemRepository.deleteItem(item);

    }

}