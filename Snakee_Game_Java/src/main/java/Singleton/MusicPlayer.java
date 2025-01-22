package Singleton;

import javazoom.jl.player.Player;

import java.io.BufferedInputStream;
import java.io.FileInputStream;

/**
 * @Project snakee_2022
 * @Author Carolina Louro
 * @File MusicPlayer.java
 * @Description This class acts the Singleton/instance of MusicPlayer() class
 * Responsible for providing and obtaining the music visualisation (play) used within the game (outputs audio file)
 * @Version 19
 */

public class MusicPlayer {

    // Getters and Setters
    /**
     * This method is a getter for the String m_filename
     * @return m_filename
     */
    public static String GetString() {
        return m_filename;
    }

    /**
     * This method is a setter for the String m_filename
     * @param f
     */
    public void SetString(String f) {
        this.m_filename = f;
    }

    /**
     * This method is a getter for the Player m_player
     * @return m_player
     */
    protected Player GetP() {
        return this.m_player;
    }

    /**
     * This method is a setter for the Player m_player
     * @param p
     */
    protected void SetP(Player p) {
        this.m_player = p;
    }


    // Variables
    private static String m_filename;
    private Player m_player;


    // Implementing the Singleton MusicPlayer
    private static MusicPlayer musicPlayerInstance;

    private MusicPlayer() { }

    /**
     * This method acts as the instance of this class reinforcing the Singleton method
     * @return musicPlayerInstance
     */
    public static MusicPlayer GetInstance() {
        if (musicPlayerInstance == null) {
            musicPlayerInstance = new MusicPlayer();
        }
        return musicPlayerInstance;
    }

    // Methods
    /**
     * This method checks if audio file is null
     * If not then plays audio file
     * If yes then it will throw an error message
     * This method is resposible for ouputting sound on screen
     */
    // Rule 8: Method Naming
    public void play(String filename)
    {
        new Thread()
        {
            @Override
            public void run()
            {
                super.run();
                try
                {

                    SetP(new Player(new BufferedInputStream(new FileInputStream(filename))));
                    GetP().play();

                } catch (Exception e)
                {
                    System.out.println(e);
                }
            }
        }.start();
    }

}
