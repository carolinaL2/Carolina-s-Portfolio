package com.example.foodwasteapp.ui.database.DateDB;

import androidx.annotation.NonNull;
import androidx.room.ColumnInfo;
import androidx.room.Entity;
import androidx.room.ForeignKey;
import androidx.room.Index;
import androidx.room.PrimaryKey;

import com.example.foodwasteapp.ui.database.Item;

//@Entity(tableName = "dateItem_table",
//        foreignKeys = @ForeignKey(entity = Item.class,
//                parentColumns = "date",
//                childColumns = "date_Foreign",
//                onDelete = ForeignKey.CASCADE))
@Entity(tableName = "dateItem_table")
public class DateItem {

    @PrimaryKey
    @NonNull
    @ColumnInfo(name = "date_Foreign")
    public String dateItem;

    public DateItem(@NonNull String dateItem) {

        this.dateItem = dateItem;

    }

    //Getters and Setters for database variables
    @NonNull
    public String getDateItem() {
        return dateItem;
    }

    public void setDateItem(@NonNull String dateItem) {
        this.dateItem = dateItem;
    }

}
