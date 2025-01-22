package com.example.foodwasteapp.ui.database;

import android.content.Context;

import androidx.room.Database;
import androidx.room.Room;
import androidx.room.RoomDatabase;
import androidx.room.migration.Migration;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@Database(entities = {Item.class}, version = 2)
public abstract class ItemDatabase extends RoomDatabase {

    public abstract ItemDao itemDao();

    private static volatile ItemDatabase itemDatabase;
    private static final int NUMBER_OF_THREADS = 4;

    static final ExecutorService databaseWriteExecutor =
            Executors.newFixedThreadPool(NUMBER_OF_THREADS);

    static ItemDatabase getDatabase(final Context context) {

        if (itemDatabase == null) {

            synchronized (ItemDatabase.class) {

                if (itemDatabase == null) {

                    itemDatabase = Room.databaseBuilder(context.getApplicationContext(),
                            ItemDatabase.class, "products_table")
                            .fallbackToDestructiveMigration()
                            .build();

                }

            }

        }

        return itemDatabase;

    }

}