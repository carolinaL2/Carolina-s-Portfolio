package Factory_pattern;

import Models.GameObjectsModel;

/**
 * @Project snakee_2022
 * @Author Carolina Louro
 * @File BonusFFactory.java
 * @Description This is part of the Factory design pattern used within the Bonus() class
 * @Version 19
 */

public class BonusFFactory {

    // Implementing a design factory pattern
    // Private variable
    private static BonusFFactory bonusFFactoryInstance;

    // Methods
    /**
     * This method is the class's private constructor
     */
    private BonusFFactory() {
    }

    /**
     * This method gets an instance of this class
     * @return bonusFFactoryInstance
     */
    public static BonusFFactory GetInstance() {
        if (bonusFFactoryInstance == null) {
            bonusFFactoryInstance = new BonusFFactory();
        }
        return bonusFFactoryInstance;
    }

    /**
     * This method checks if BonusFEnum() class is null
     * @param bonusFEnum
     * @return
     */
    public GameObjectsModel GetGameObjectsModel(BonusFEnum bonusFEnum) {
        switch (bonusFEnum) {
            case BONUS: {
                return new Bonus();
            }
            default: {
                return null;
            }
        }
    }
}