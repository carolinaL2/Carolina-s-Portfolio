package Factory_pattern;

import Models.GameObjectsModel;
import Models.ImagesModel;
import View.SnakeView;

import java.awt.*;

/**
 * @Project snakee_2022
 * @Author Carolina Louro
 * @File Obstacles.java
 * @Description This class acts as the View class (MVC) for the GameObjectsModel class
 * Responsible for outputting the obstacles used to make game more interesting
 * This class also checks if the snake intersected with the obstacle and if so then an event occurs
 * @Version 19
 */

public class Obstacles extends GameObjectsModel {

    // Getters and Setters
    /**
     * This method is a getter for the boolean m_t
     * The variable m_t is used to check if snake touched obstacle
     * @return m_t
     */
    public static boolean GetTouched() {
        return m_t;
    }

    /**
     * This method is a setter for the boolean m_t
     * The variable m_t is used to check if snake touched obstacle
     * @param t
     * @return t
     */
    public void SetTouched(boolean t) {
        this.m_t = t;
    }


    // Variables
    private final long serialVersionUID = -3641221053272056036L;
    private static boolean m_t = false;
    int VARIABLE_ONE = 870;
    int VARIABLE_TWO = 10;
    int VARIABLE_THREE = 560;
    int VARIABLE_FOUR = 40;
    int POINTS = 200;


    // Methods
    /**
     * This acts as the class's constructor
     * This method gets the obstacles randomly and places them in random locations
     */
    public Obstacles() {

        // Places the obstacle on the panel
        this.SetL(true);
        this.GetTouched();

        // Gets the obstacle to be outputted on screen
        ImagesModel images = new ImagesModel();
        // In this case gets image with the String "19"
        this.SetImg(images.images.get(String.valueOf(19)));

        this.SetW(GetImg().getWidth(null));
        this.SetH(GetImg().getHeight(null));

        this.SetX((int) (Math.random() * (VARIABLE_ONE - GetW() + VARIABLE_TWO)));
        this.SetY((int) (Math.random() * (VARIABLE_THREE - GetH() - VARIABLE_FOUR)));

    }

    /**
     * This method checks if snake has touched the obstacle
     * If so, then points are subtracted and lives are reduced
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
            SetTouched(true);

            if (sc != 0) {

                sc = sc - POINTS;

            }

            mySnake.SetScore(sc);

        }

    }

    /**
     * This method draws the obstacles onto the screen by calling the draw() method
     * setTouched() sets the boolean m_t back to false so it resets every time the snake touches the obstacle
     * @param g
     */
    // Rule 8: Method Naming
    @Override
    public void Draw(Graphics g)
    {
        SetTouched(false);
        g.drawImage(GetImg(), GetX(), GetY(), null);
    }

}