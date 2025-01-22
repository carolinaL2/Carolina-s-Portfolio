module module_Info {

    requires javafx.controls;
    requires javafx.fxml;
    requires java.desktop;
    requires jlayer;
    requires opencsv;

    exports Controller;
    opens Controller to javafx.fxml;

    exports View;
    opens View to javafx.fxml;

    exports Models;
    opens Models to javafx.fxml;

    exports Singleton;
    opens Singleton to javafx.fxml;

    exports main;
    opens main to javafx.fxml;

    exports Factory_pattern;
    opens Factory_pattern to javafx.fxml;

    opens FXMLModel to javafx.fxml;


}