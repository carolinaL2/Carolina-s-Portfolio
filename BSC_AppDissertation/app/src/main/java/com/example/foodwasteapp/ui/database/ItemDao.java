package com.example.foodwasteapp.ui.database;

import androidx.lifecycle.LiveData;
import androidx.room.Dao;
import androidx.room.Delete;
import androidx.room.DeleteColumn;
import androidx.room.Insert;
import androidx.room.OnConflictStrategy;
import androidx.room.Query;
import androidx.room.Update;

import java.util.List;

@Dao
public interface ItemDao {

    @Insert(onConflict = OnConflictStrategy.IGNORE)
    void insert(Item name);

    @Delete
    void Delete(Item item);

    @Update
    void update(Item name);

    @Query("SELECT * from products_table ORDER By name Asc")
    LiveData<List<Item>> getItemName();

}