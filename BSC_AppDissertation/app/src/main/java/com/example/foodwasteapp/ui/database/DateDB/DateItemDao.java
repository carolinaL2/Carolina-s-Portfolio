package com.example.foodwasteapp.ui.database.DateDB;

import androidx.lifecycle.LiveData;
import androidx.room.Dao;
import androidx.room.Delete;
import androidx.room.Insert;
import androidx.room.OnConflictStrategy;
import androidx.room.Query;
import androidx.room.Update;

import java.util.List;

@Dao
public interface DateItemDao {

    @Insert(onConflict = OnConflictStrategy.IGNORE)
    void insert(DateItem name);

    @Delete
    void Delete(DateItem item);

    @Update
    void update(DateItem name);

    @Query("SELECT * from dateItem_table ORDER By date_Foreign Asc")
    LiveData<List<DateItem>> getDates();

}