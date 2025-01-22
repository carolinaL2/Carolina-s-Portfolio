package com.example.foodwasteapp;

import androidx.lifecycle.LiveData;
import androidx.lifecycle.MutableLiveData;
import androidx.lifecycle.ViewModel;

/*
   This class is a singleton ViewModel that stores the name, quantity, and expiry date of the item scanned by the NFC tag.
 */

public class singletonVM extends ViewModel {

    // MutableLiveData objects to store the name, quantity, and expiry date of the item scanned by the NFC tag.
    private final MutableLiveData<String> name = new MutableLiveData<String>();
    private final MutableLiveData<String> quantity = new MutableLiveData<String>();
    private final MutableLiveData<String> date = new MutableLiveData<String>();

    // Getters and setters for variables name, quantity, and expiry date.
    public LiveData<String> getName() {
        return name;
    }

    public void setName(String item) {

        name.setValue(item);

    }

    public LiveData<String> getQuantity() {
        return quantity;
    }

    public void setQuantity(String item) {

        quantity.setValue(item);

    }

    public LiveData<String> getDate() {
        return date;
    }

    public void setDate(String item) {

        date.setValue(item);

    }

}