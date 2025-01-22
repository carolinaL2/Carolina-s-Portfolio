package main;

import View.StartScreenView;

/**
 * @Project snakee_2022
 * @Author Carolina Louro
 * @File main.java
 * @Description This class calls the main in the StartScreenView() class to run the game using JavaFX
 * @Version 19
 */

public class main {

    // Methods
    /**
     * This is the main method which calls the 'StartScreenView()' in order to run the start screen using JavaFx
     * @param args
     */
    public void main(String[] args)
    {

            new StartScreenView().main(args);

    }

}