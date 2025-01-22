package com.example.foodwasteapp.ui.database;

import androidx.annotation.NonNull;
import androidx.room.ColumnInfo;
import androidx.room.Entity;
import androidx.room.ForeignKey;
import androidx.room.Index;
import androidx.room.PrimaryKey;

@Entity(tableName = "products_table")
public class Item {

    @PrimaryKey
    @NonNull
    @ColumnInfo(name = "name")
    public String itemName;

    @ColumnInfo(name = "description")
    public String itemDescription;

    @ColumnInfo(name = "date")
    public String itemExpiryDate;

    @ColumnInfo(name = "quantity")
    public String itemQuantity;

    public Item(@NonNull String itemName, String itemDescription, String itemExpiryDate, String itemQuantity) {

        this.itemName = itemName;
        this.itemDescription = itemDescription;
        this.itemExpiryDate = itemExpiryDate;
        this.itemQuantity = itemQuantity;

    }

    //Getters and Setters for database variables

    @NonNull
    public String getItemName() {
        return itemName;
    }

    public void setItemName(@NonNull String itemName) {
        this.itemName = itemName;
    }

    public String getItemDescription() {
        return itemDescription;
    }

    public void setItemDescription(String itemDescription) {
        this.itemDescription = itemDescription;
    }

    public String getItemExpiryDate() {
        return itemExpiryDate;
    }

    public void setItemExpiryDate(String itemExpiryDate) {
        this.itemExpiryDate = itemExpiryDate;
    }

    public String getItemQuantity() {
        return itemQuantity;
    }

    public void setItemQuantity(String itemQuantity) {
        this.itemQuantity = itemQuantity;
    }

}
