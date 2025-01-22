package View;

import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.stage.Stage;

import java.io.IOException;

/**
 * @Project snakee_2022
 * @Author Carolina Louro
 * @File EndScreenView.java
 * @Description This class is the View class (MVC) for the EndScreenModel class
 * Responsible for outputting the JavaFX end screen on screen
 * @Version 19
 */

public class EndScreenView extends Application {

    // Getters and Setters
    /**
     * This method is a getter for the FXMLLoader m_loader
     * @return m_loader
     */
    public static FXMLLoader GetL() {
        return m_loader;
    }

    /**
     * This method is a setter for the FXMLLoader m_loader
     * @param loader
     */
    public static void SetL(FXMLLoader loader) {
        m_loader = loader;
    }

    /**
     * This method is a getter for the Scene m_scene
     * @return m_scene
     */
    public static Scene GetScene() {
        return m_scene;
    }

    /**
     * This method is a setter for the Scene m_scene
     * @param scene
     */
    public static void SetScene(Scene scene) {
        m_scene = scene;
    }


    // Variables
    private static FXMLLoader m_loader;

    public static Stage getM_stage() {
        return m_stage;
    }

    public static void setM_stage(Stage m_stage) {
        EndScreenView.m_stage = m_stage;
    }

    private static Stage m_stage;
    private static Scene m_scene;
    int V_VARIABLE = 860;
    int V1_VARIABLE = 450;


    // Methods
    /**
     * This method outputs all visuals on screen for the JavaFX end screen
     * Loads the FXML file and then creates a scene using this file to output the end screen
     * @param stage
     * @throws IOException
     */
    @Override
    public void start(Stage stage) throws IOException {

        this.setM_stage(stage);
        SetL(new FXMLLoader(StartScreenView.class.getResource(
                "/FXMLModel/EndScreen.fxml")));
        SetScene(new Scene(GetL().load(), V_VARIABLE, V1_VARIABLE));

        getM_stage().setTitle("Snake Yipee Highscore");
        getM_stage().setScene(GetScene());
        getM_stage().show();

    }

    /**
     * This method launches main() method so the game can run
     * @param args
     */
    public static void main(String[] args) {
        launch(args);
    }

}