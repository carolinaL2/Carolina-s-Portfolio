package Factory_pattern;

import Models.GameObjectsModel;

/**
 * @Project snakee_2022
 * @Author Carolina Louro
 * @File ObstaclesFFactory.java
 * @Description This is part of the Factory design pattern used within the Obstacles() class
 * @Version 19
 */

public class ObstaclesFFactory {

    // Implementing a design factory pattern
    // Private variable
    private static ObstaclesFFactory obstaclesFFactoryInstance;

    // Methods
    /**
     * This method is the class's private constructor
     */
    private ObstaclesFFactory() {
    }

    /**
     * This method gets an instance of this class
     * @return obstaclesFFactoryInstance
     */
    public static ObstaclesFFactory GetInstance() {
        if (obstaclesFFactoryInstance == null) {
            obstaclesFFactoryInstance = new ObstaclesFFactory();
        }
        return obstaclesFFactoryInstance;
    }

    /**
     * This method checks if ObstaclesFEnum() class is null
     * @param obstaclesFEnum
     * @return
     */
    public GameObjectsModel GetGameObjectsModel(ObstaclesFEnum obstaclesFEnum) {
        switch (obstaclesFEnum) {
            case OBSTACLES_VIEW: {
                return new Obstacles();
            }
            default: {
                return null;
            }
        }
    }
}