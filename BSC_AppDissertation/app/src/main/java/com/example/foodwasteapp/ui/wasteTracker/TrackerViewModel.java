package com.example.foodwasteapp.ui.wasteTracker;

/*
    This class is used to store the amount of food wasted by the user.
    It has two attributes: amount and txtAmount.
 */

public class TrackerViewModel {

    //Declare variables
    private double amount;
    private String txtAmount;

    //Getters and Setters

    public double getAmount() {
        return amount;
    }

    public void setAmount(double amount) {
        this.amount = amount;
    }

    public void addAmount(double newAmount) {
        amount = amount + newAmount;
    }

    public String getTxtAmount() {
        return txtAmount;
    }

    public void setTxtAmount(String txtAmount) {
        this.txtAmount = txtAmount;
    }

}