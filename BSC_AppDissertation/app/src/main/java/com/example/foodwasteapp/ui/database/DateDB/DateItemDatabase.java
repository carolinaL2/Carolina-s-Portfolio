package com.example.foodwasteapp.ui.database.DateDB;

import android.content.Context;

import androidx.room.Database;
import androidx.room.Room;
import androidx.room.RoomDatabase;

import com.example.foodwasteapp.ui.database.Item;
import com.example.foodwasteapp.ui.database.ItemDao;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@Database(entities = {Item.class, DateItem.class}, version = 6)
public abstract class DateItemDatabase extends RoomDatabase {

    public abstract DateItemDao itemDao();
    public abstract ItemDao singleItemDao();

    private static volatile DateItemDatabase itemDatabase;
    private static final int NUMBER_OF_THREADS = 4;

    static final ExecutorService databaseWriteExecutor =
            Executors.newFixedThreadPool(NUMBER_OF_THREADS);

    static DateItemDatabase getDatabase(final Context context) {

        if (itemDatabase == null) {

            synchronized (DateItemDatabase.class) {

                if (itemDatabase == null) {

                    itemDatabase = Room.databaseBuilder(context.getApplicationContext(),
                            DateItemDatabase.class, "dateItem_table")
                            .fallbackToDestructiveMigration()
                            .build();

                }

            }

        }

        return itemDatabase;

    }

}