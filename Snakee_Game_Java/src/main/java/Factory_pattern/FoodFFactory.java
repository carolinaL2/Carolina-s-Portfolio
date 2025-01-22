package Factory_pattern;

import Models.GameObjectsModel;

/**
 * @Project snakee_2022
 * @Author Carolina Louro
 * @File FoodFFactory.java
 * @Description This is part of the Factory design pattern used within the Food() class
 * @Version 19
 */

public class FoodFFactory {

    // Implementing a design factory pattern
    // Private variable
    private static FoodFFactory foodFFactoryInstance;

    // Methods
    /**
     * This method is the class's private constructor
     */
    private FoodFFactory() {
    }

    /**
     * This method gets an instance of this class
     * @return foodFFactoryInstance
     */
    public static FoodFFactory GetInstance() {
        if (foodFFactoryInstance == null) {
            foodFFactoryInstance = new FoodFFactory();
        }
        return foodFFactoryInstance;
    }

    /**
     * This method checks if FoodFEnum() class is null
     * @param foodFEnum
     * @return
     */
    public GameObjectsModel GetGameObjectsModel(FoodFEnum foodFEnum) {
        switch (foodFEnum) {
            case FOOD_VIEW: {
                return new Food();
            }
            default: {
                return null;
            }
        }
    }
}