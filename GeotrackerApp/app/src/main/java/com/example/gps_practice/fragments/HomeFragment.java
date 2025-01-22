package com.example.gps_practice.fragments;

import android.content.Intent;
import android.os.Bundle;

import androidx.fragment.app.Fragment;
import androidx.fragment.app.FragmentTransaction;
import androidx.lifecycle.ViewModelProvider;

import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;

import com.example.gps_practice.ItemViewModel;
import com.example.gps_practice.R;
import com.example.gps_practice.databinding.FragmentMapBinding;
import com.example.gps_practice.databinding.FragmentRemindersBinding;

public class HomeFragment extends Fragment {

    private ItemViewModel viewModel;
    FragmentMapBinding mapBinding;
    FragmentRemindersBinding remindersBinding;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {

        // Inflate the layout for this fragment
        return inflater.inflate(R.layout.fragment_home, container, false);

    }

}