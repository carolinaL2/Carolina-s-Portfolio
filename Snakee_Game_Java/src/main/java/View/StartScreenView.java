package View;

import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.stage.Stage;

import java.io.IOException;

/**
 * @Project snakee_2022
 * @Author Carolina Louro
 * @File StartScreenView.java
 * @Description This class is a View of the
 * Responsible for outputting the JavaFX start screen out on screen so the player can see it
 * @Version 19
 */

public class StartScreenView extends Application {

    // Getters and Setters
    /**
     * This method is a getter for the Scene m_scene
     * @return m_scene
     */
    public static Scene GetM_scene() {
        return m_scene;
    }

    /**
     * This method is a setter for the Scene m_scene
     * @param m_scene
     */
    public static void SetM_scene(Scene m_scene) {
        StartScreenView.m_scene = m_scene;
    }

    /**
     * This method is a getter for the FXMLLoader m_loader
     * @return m_loader
     */
    public FXMLLoader GetLoader() {
        return m_loader;
    }

    /**
     * This method is a setter for the FXMLLoader m_loader
     * @param m_loader
     */
    public void SetLoader(FXMLLoader m_loader) {
        this.m_loader = m_loader;
    }


    // Variables
    private static Scene m_scene;
    private FXMLLoader m_loader;
    int V_VARIABLE = 860;
    int V1_VARIABLE = 450;


    //Methods
    /**
     * This method obtains a scene by calling the FXML file being used to display that scene
     * This will then be shows on screen
     * @param stage
     * @throws IOException
     */
    @Override
    public void start(Stage stage) throws IOException {

        SetLoader(new FXMLLoader(StartScreenView.class.getResource(
                "/FXMLModel/JavaFX.fxml")));
        SetM_scene(new Scene(GetLoader().load(), V_VARIABLE, V1_VARIABLE));

        stage.setTitle("Snake Yipee");
        stage.setScene(GetM_scene());
        stage.show();

    }

    /**
     * This method launches the main() method so the game can run
     * @param args
     */
    public static void main(String[] args) {
        launch(args);
    }

}