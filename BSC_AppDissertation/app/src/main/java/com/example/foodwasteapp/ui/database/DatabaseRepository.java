package com.example.foodwasteapp.ui.database;

import android.app.Application;

import androidx.lifecycle.LiveData;

import java.util.List;

public class DatabaseRepository {

    ItemDatabase itemDatabase;
    ItemDao itemDao;
    private final LiveData<List<Item>> itemList;

    public DatabaseRepository(Application application) {

        // Get data of locationDao indirectly from the database
        // By creating a database object inside the repository constructor
        itemDatabase = ItemDatabase.getDatabase(application);
        itemDao = itemDatabase.itemDao();
        itemList = itemDao.getItemName();

    }

    public void insertItem(Item item) {

        ItemDatabase.databaseWriteExecutor.execute(() -> itemDao.insert(item));

    }

    public void deleteItem(Item item) {

        ItemDatabase.databaseWriteExecutor.execute(() -> itemDao.Delete(item));

    }

    public LiveData<List<Item>> getAllItems() {

        return itemList;

    }

}
