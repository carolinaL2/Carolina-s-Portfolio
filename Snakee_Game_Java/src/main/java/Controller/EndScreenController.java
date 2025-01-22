package Controller;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.net.URL;
import java.util.ArrayList;
import java.util.ResourceBundle;

import Models.EndScreenModel;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.scene.control.TableColumn;
import javafx.scene.control.TableView;
import javafx.scene.control.cell.PropertyValueFactory;

/**
 * @Project snakee_2022
 * @Author Carolina Louro
 * @File EndScreenController.java
 * @Description This class is the Controller class (MVC) for the EndScreenModel and EndScreenView class
 * Responsible for reacting to input from user/state of GUI about the JavaFX end screen
 * This class calls the main in the EndScreenView() class to run the game using JavaFX
 * @Version 19
 */

public class EndScreenController {

    // Variables
    @FXML
    private ResourceBundle resources;
    @FXML
    private URL location;
    @FXML
    private TableView<EndScreenModel> table;
    @FXML
    private TableColumn<EndScreenModel, String> name;

    @FXML
    private TableColumn<EndScreenModel, Integer> score;

    private static String[] values;
    private static ArrayList<String> nameA = new ArrayList<>();
    private static ArrayList<Integer> scoreA = new ArrayList<>();
    private static ObservableList<EndScreenModel> list = FXCollections
            .observableArrayList();

    /**
     * This method initialises the table view to fill the data from the csv file in the table columns
     * @throws IOException
     */
    @FXML
    void initialize() throws IOException {
        assert name != null : "fx:id=\"name\" was not injected: check your FXML file 'EndScreen.fxml'.";
        assert score != null : "fx:id=\"score\" was not injected: check your FXML file 'EndScreen.fxml'.";
        assert table != null : "fx:id=\"table\" was not injected: check your FXML file 'EndScreen.fxml'.";

        name.setCellValueFactory(new PropertyValueFactory<EndScreenModel, String>
                ("Name"));
        score.setCellValueFactory(new PropertyValueFactory<EndScreenModel, Integer>
                ("Score"));
        score.setSortType(TableColumn.SortType.DESCENDING);

        try { 
            list = results();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        table.setItems(list);
        table.getSortOrder().add(score);
        table.sort();

    }

    /**
     * This method reads from the csv file and stores the data in an ArrayList()
     */
    public static void ReadCSV() {

        String csvFile = "highscore.csv";
        String line = "";
        String splitBy = ",";
        int m;

        try (BufferedReader br = new BufferedReader(new FileReader(csvFile))) {

            while ((line = br.readLine()) != null) {

                values = line.split(splitBy);
                nameA.add(values[0]);
                m = Integer.parseInt(values[1].trim());
                scoreA.add(m);

            }

        } catch (IOException e) {

            throw new RuntimeException(e);

        }

    }

    /**
     * This method converts the ArrayList() to an ObservableList() so data can be put inside the table columns
     * @return
     * @throws IOException
     */
    public static ObservableList<EndScreenModel> results() throws IOException {

        ReadCSV();
        for (int i = 0; i < nameA.size(); i++) {

            list.add(new EndScreenModel(nameA.get(i).toString(), scoreA.get(i)));

        }

        return list;

    }

}