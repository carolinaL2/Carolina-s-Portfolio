package com.example.foodwasteapp.ui.share;

import android.content.Intent;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;

import androidx.annotation.NonNull;
import androidx.cardview.widget.CardView;
import androidx.fragment.app.Fragment;
import androidx.navigation.Navigation;

import com.example.foodwasteapp.R;

/*
    This class is used to implement the ShareFragment using the sharing feature of the app.
 */

public class ShareFragment extends Fragment {

    //Declaring variables
    View view;
    CardView cardView, backB;

    //This method is used to create the view of the ShareFragment and implement the back button and sharing feature of the app.

    public View onCreateView(@NonNull LayoutInflater inflater,
                             ViewGroup container, Bundle savedInstanceState) {

        view = inflater.inflate(R.layout.share_fragment, container, false);

        findViews();
        backB.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Navigation.findNavController(view).navigate(R.id.action_sharePage_to_navigation_home);
            }

        });

        cardView.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                Intent intent = new Intent(Intent.ACTION_SEND);
                intent.setType("text/plain");
                String body = "Download this App";
                String sub = "http://reduceFoodWasteApp.google.com";
                intent.putExtra(Intent.EXTRA_TEXT, body);
                intent.putExtra(Intent.EXTRA_TEXT, sub);
                startActivity(Intent.createChooser(intent, "Share using"));

            }

        });

        return view;

    }

    //This method is used to find the views of this fragment's layout components and assign them to the variables.
    private void findViews() {
        backB = view.findViewById(R.id.backButton_share);
        cardView = view.findViewById(R.id.share_card);
    }

}