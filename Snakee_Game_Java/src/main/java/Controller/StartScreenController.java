package Controller;

import Singleton.MusicPlayer;
import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.TextField;

/**
 * @Project snakee_2022
 * @Author Carolina Louro
 * @File StartScreenController.java
 * @Description This class is the Controller class (MVC) for the StartScreenModel and StartScreenView class
 * Responsible for reacting to input from user/state of GUI about the JavaFX start screen
 * This class calls the main in the StartScreenView() class to run the game using JavaFX
 * @Version 19
 */

public class StartScreenController {

    // Getters and Setters
    /**
     * This method is a getter for the MusicPlayer mp
     * @return m_mp
     */
    public static MusicPlayer GetMusicP() {
        return m_mp;
    }

    /**
     * This method is a setter for the MusicPlayer mp
     * @param mp
     */
    public static void SetMusicP(MusicPlayer mp) {
        StartScreenController.m_mp = mp;
    }

    /**
     * This method is a getter for the integer m_backg
     * @return m_backg
     */
    public int GetBackg() {
        return m_backg;
    }

    /**
     * This method is a setter for the integer m_backg
     * @param backg
     */
    public void SetBackg(int backg) {
        this.m_backg = backg;
    }

    /**
     * This method is a getter for the integer m_snakeColour
     * @return m_snakeColour
     */
    public int GetSnakeColour() {
        return m_snakeColour;
    }

    /**
     * This method is a setter for the integer m_snakeColour
     * @param snakeColour
     */
    public void SetSnakeColour(int snakeColour) {
        this.m_snakeColour = snakeColour;
    }

    /**
     * This method is a getter for the boolean m_medium
     * @return m_medium
     */
    public boolean GetMedium() {
        return m_medium;
    }

    /**
     * This method is a setter for the boolean m_medium
     * @param medium
     */
    public void SetMedium(boolean medium) {
        this.m_medium = medium;
    }

    /**
     * This method is a getter for the boolean m_hard
     * @return m_hard
     */
    public boolean GetHard() {
        return m_hard;
    }

    /**
     * This method is a setter for the boolean m_hard
     * @param hard
     */
    public void SetHard(boolean hard) {
        this.m_hard = hard;
    }


    // Variables
    private static MusicPlayer m_mp = MusicPlayer.GetInstance();
    private int m_backg = 0;
    private int m_snakeColour = 0;

    boolean m_medium = false;
    boolean m_hard = false;


    // FXML Items
    @FXML
    private Button easyB;

    @FXML
    private Button hardB;

    @FXML
    private Button mediumB;

    @FXML
    private Button purple;

    @FXML
    private Button green;

    @FXML
    private Button orange;

    @FXML
    private Button backg1;

    @FXML
    private Button backg2;

    @FXML
    private TextField text;

    /**
     * This method checks if the first background option was clicked by user input
     * If so then int m_backg is set to 1 and passed to the GameController() class
     * @param event
     */
    @FXML
    void backgroundOne(ActionEvent event) {

        SetBackg(1);

    }

    /**
     * This method checks if the second background option was clicked by user input
     * If so then int m_backg is set to 2 and passed to the GameController() class
     * @param event
     */
    @FXML
    void backgroundTwo(ActionEvent event) {

        SetBackg(2);

    }

    /**
     * This method checks if the purple snake colour button was clicked by user input
     * If so then int m_snakeColour is set to 1 and passed to the GameController() class
     * @param event
     */
    @FXML
    void purpleColour(ActionEvent event) {

        SetSnakeColour(1);

    }

    /**
     * This method checks if the green snake colour button was clicked by user input
     * If so then int m_snakeColour is set to 2 and passed to the GameController() class
     * @param event
     */
    @FXML
    void greenColour(ActionEvent event) {

        SetSnakeColour(2);

    }

    /**
     * This method checks if the orange snake colour button was clicked by user input
     * If so then int m_snakeColour is set to 3 and passed to the GameController() class
     * @param event
     */
    @FXML
    void orangeColour(ActionEvent event) {

        SetSnakeColour(3);

    }

    /**
     * This method checks if the easy level button was clicked by user input
     * If so then the GameController() class is called alongside the LoadFrame() class to load the game
     * @param event
     */
    @FXML
    void buttonClicked(ActionEvent event) {

        // Passing the player's name to the main game
        String t = text.getText();
        // Passing the speed to the controller class so it can be changed according to the game mode chosen
        int s = 1;

        // Load main game - easy level
        GetMusicP().play("src/main/resources/easyLevel.mp3");
        new GameController(t, GetBackg(), s, GetMedium(),
                GetHard(), GetSnakeColour()).LoadFrame();

    }

    /**
     * This method checks if the medium level button was clicked by user input
     * If so then the GameController() class is called alongside the LoadFrame() class to load the game
     * @param event
     */
    @FXML
    void buttonClicked2(ActionEvent event) {

        String t = text.getText();

        // If easy level then speed stays the same
        int s = 2;
        // Changes medium to true and if so then passes this value so obstacles can be added to this game mode
        SetMedium(true);

        // Load main game - medium level
        GetMusicP().play("src/main/resources/mediumLevel.mp3");
        new GameController(t, GetBackg(), s, GetMedium(), GetHard(),
                GetSnakeColour()).LoadFrame();

    }

    /**
     * This method checks if the hard level button was clicked by user input
     * If so then the GameController() class is called alongside the LoadFrame() class to load the game
     * @param event
     */
    @FXML
    void buttonClicked3(ActionEvent event) {

        String t = text.getText();

        // If easy level then speed stays the same
        int s = 3;
        // Changes hard to true and if so then passes this value so walls can be added to this game mode
        SetHard(true);

        // Load main game - hard level
        GetMusicP().play("src/main/resources/hardLevel.mp3");
        new GameController(t, GetBackg(), s, GetMedium(),
                GetHard(), GetSnakeColour()).LoadFrame();

    }

}
