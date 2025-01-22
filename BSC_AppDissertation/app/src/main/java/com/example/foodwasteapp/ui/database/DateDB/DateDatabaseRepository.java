package com.example.foodwasteapp.ui.database.DateDB;

import android.app.Application;

import androidx.lifecycle.LiveData;

import com.example.foodwasteapp.ui.database.Item;
import com.example.foodwasteapp.ui.database.ItemDao;
import com.example.foodwasteapp.ui.database.ItemDatabase;

import java.util.List;

public class DateDatabaseRepository {

    DateItemDatabase itemDatabase;
    DateItemDao itemDao;
    ItemDao singleItemDao;
    private final LiveData<List<DateItem>> dateList;

    public DateDatabaseRepository(Application application) {

        // Get data of locationDao indirectly from the database
        // By creating a database object inside the repository constructor
        itemDatabase = DateItemDatabase.getDatabase(application);
        itemDao = itemDatabase.itemDao();
        singleItemDao = itemDatabase.singleItemDao();
        dateList = itemDao.getDates();

    }

    public void insertItem(DateItem item) {

        DateItemDatabase.databaseWriteExecutor.execute(() -> itemDao.insert(item));

    }

    public void deleteItem(DateItem item) {

        DateItemDatabase.databaseWriteExecutor.execute(() -> itemDao.Delete(item));

    }

    public LiveData<List<DateItem>> getAllDates() {

        return dateList;

    }

}
