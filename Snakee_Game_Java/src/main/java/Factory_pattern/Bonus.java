package Factory_pattern;

import Models.GameObjectsModel;
import Models.ImagesModel;
import View.SnakeView;

import java.awt.*;

/**
 * @Project snakee_2022
 * @Author Carolina Louro
 * @File Bonus.java
 * @Description This class is responsible for outputting the bonus items for the player to obtain extra points
 * Acts as the view (MVC) for the bonus items - visualisation of the Bonus items
 * It's model is the GameObjectsModel()
 * @Version 19
 */

public class Bonus extends GameObjectsModel {

    // Variables
    private final long serialVersionUID = -3641221053272056036L;
    int VARIABLE_ONE = 870;
    int VARIABLE_TWO = 10;
    int VARIABLE_THREE = 560;
    int VARIABLE_FOUR = 40;
    int POINTS = 200;


    // Methods
    /**
     * This method is the class's constructor
     * This method gets the bonus obstacles and places them in random locations
     */
    public Bonus() {

        // Places the obstacle on the panel
        this.SetL(true);

        // Gets the obstacle to be outputted on screen
        ImagesModel images = new ImagesModel();
        // In this case gets image with the String "19"
        this.SetImg(images.images.get(String.valueOf(20)));

        this.SetW(GetImg().getWidth(null));
        this.SetH(GetImg().getHeight(null));

        this.SetX((int) (Math.random() * (VARIABLE_ONE - GetW() + VARIABLE_TWO)));
        this.SetY((int) (Math.random() * (VARIABLE_THREE - GetH() - VARIABLE_FOUR)));

    }

    /**
     * This method checks if snake has touched an bonus item
     * If yes then add extra points to final score
     * @param mySnake
     */
    public void Touched(SnakeView mySnake)	{

        // Checks if snake has touched an obstacle
        if (mySnake.GetRectangle().intersects(this.GetRectangle()) && GetL()
                && mySnake.GetL())
        {

            int sc;
            sc = mySnake.GetScore();

            this.l = false;

            sc = sc + POINTS;

            mySnake.SetScore(sc);

        }

    }

    /**
     * This method draws the bonus items onto the screen by calling the draw() method
     * @param g
     */
    @Override
    public void Draw(Graphics g)
    {

        g.drawImage(GetImg(), GetX(), GetY(), null);
    }

}
