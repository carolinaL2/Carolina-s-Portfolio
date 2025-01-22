package Models;

import javafx.beans.property.SimpleIntegerProperty;
import javafx.beans.property.SimpleStringProperty;

import java.io.*;
import java.util.Map;

/**
 * @Project snakee_2022
 * @Author Carolina Louro
 * @File EndScreenModel.java
 * @Description This class is the Model class (MVC) for the JavaFX end screen
 * This class stores the player's name and score and writes it out to the highscore.csv file
 * @Version 19
 */

public class EndScreenModel {

    // Getters and Setters
    /**
     * This method is a getter for the String m_name
     * @return
     */
    public String getName() {
        return m_name.get();
    }

    /**
     * This method is a setter for the String m_name
     * @param name
     */
    public void setName(String name) {
        this.m_name = new SimpleStringProperty(name);
    }

    /**
     * This method is a getter for the Integer m_score
     * @return
     */
    public Integer getScore() {
        return m_score.get();
    }

    /**
     * This method is a setter for the Integer m_score
     * @param score
     */
    public void setScore(Integer score) {
        this.m_score = new SimpleIntegerProperty(score);
    }


    // Variables
    private SimpleStringProperty m_name;
    private SimpleIntegerProperty m_score;


    // Methods
    /**
     * This method is the class's constructor
     * @param name
     * @param score
     */
    public EndScreenModel(String name, Integer score) {

        this.m_name = new SimpleStringProperty(name);
        this.m_score = new SimpleIntegerProperty(score);

    }

    /**
     * This method opens a new csv file
     * Writes the player's name and score to the csv file using a buffer
     * @param playerScore
     */
    public static void WriteCSV(Map<String, Integer> playerScore) {

        File file = new File("highscore.csv");
        BufferedWriter bf = null;

        try {

            // Creates new BufferedWriter for the output file
            bf = new BufferedWriter(new FileWriter(file, true));

            // Iterate map entries
            for (Map.Entry<String, Integer> entry :
                    playerScore.entrySet()) {

                // put key and value separated by a colon
                bf.write(entry.getKey() + ","
                        + entry.getValue());

                // new line
                bf.newLine();

            }

            bf.flush();

        } catch (IOException e) {

            throw new RuntimeException(e);

        }

    }

}
