package Factory_pattern;

import Models.GameObjectsModel;
import Models.ImagesModel;
import View.SnakeView;

import java.awt.*;
import java.util.Random;

/**
 * @Project snakee_2022
 * @Author Carolina Louro
 * @File Food.java
 * @Description This class is responsible for outputting the food items for the snake to eat
 * Acts as the view (MVC) for the food item - visualisation of the Food items
 * It's model is the GameObjectsModel()
 * @Version 19
 */

public class Food extends GameObjectsModel {

    // Variables
    private final long serialVersionUID = -3641221053272056036L;
    int VARIABLE_ONE = 870;
    int VARIABLE_TWO = 10;
    int VARIABLE_THREE = 560;
    int VARIABLE_FOUR = 40;
    int POINTS = 521;


    // Methods

    /**
     * This is the class's constructor
     * This method gets the food items randomly and places them in random locations
     */
    public Food() {

        // Places the food on the board
        this.SetL(true);

        ImagesModel images = new ImagesModel();
        this.SetImg(images.images.get(String.valueOf(new Random().nextInt(16))));

        this.SetW(GetImg().getWidth(null));
        this.SetH(GetImg().getHeight(null));

        // Rule 9: Symbolic Constants
        // Puts the food items in random locations (coordinates) on the board
        this.SetX((int) (Math.random() * (VARIABLE_ONE - GetW() + VARIABLE_TWO)));
        this.SetY((int) (Math.random() * (VARIABLE_THREE - GetH() - VARIABLE_FOUR)));

    }

    /**
     * This method checks if snake is eating food, and if so then increase snake's length
     * @param mySnake
     */
    public void Eaten(SnakeView mySnake)	{

        Rectangle k = mySnake.GetRectangle();
        if (k.intersects(this.GetRectangle()) && GetL() && mySnake.GetL())
        {
            int sc;
            sc = mySnake.GetScore();

            this.l = false;
            mySnake.SetLength(mySnake.GetLength() + 1);
            sc += POINTS;
            mySnake.SetScore(sc);

        }

    }

    /**
     * This method draws the food items onto the screen by calling the draw() method
     * @param g
     */
    @Override
    public void Draw(Graphics g)
    {
        g.drawImage(GetImg(), GetX(), GetY(), null);
    }

}
