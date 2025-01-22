package View;

import Models.GameObjectsModel;
import Models.ImagesModel;


import java.awt.*;
import java.awt.event.KeyEvent;
import java.awt.image.BufferedImage;

/**
 * @Project snakee_2022
 * @Author Carolina Louro
 * @File SnakeView.java 
 * @Description This class is the view class (MVC) of the GameObjectsModel class
 * Responsible for outputting the snake visualisation used within the game on screen
 * This class is also responsible for the Snake's movements (listens to input to provide output)
 * @Version 19
 */

public class SnakeView extends GameObjectsModel  {

    // Getters and Setters

    /**
     * This method is a getter for the BufferedImage m_IMG_SNAKE_HEAD
     * @return m_IMG_SNAKE_HEAD
     */
    public BufferedImage GetIMG_SNAKE_HEAD() {
        return m_IMG_SNAKE_HEAD;
    }

    /**
     * This method is a setter for the BufferedImage m_IMG_SNAKE_HEAD
     * @param IMG_SNAKE_HEAD
     */
    public void SetIMG_SNAKE_HEAD(BufferedImage IMG_SNAKE_HEAD) {
        this.m_IMG_SNAKE_HEAD = IMG_SNAKE_HEAD;
    }

    /**
     * This method is a getter for the BufferedImage m_newImgSnakeHead
     * @return m_newImgSnakeHead
     */
    public BufferedImage GetNewImgSnakeHead() {
        return m_newImgSnakeHead;
    }

    /**
     * This method is a setter for the BufferedImage m_newImgSnakeHead
     * @param newImgSnakeHead
     */
    public void SetNewImgSnakeHead(BufferedImage newImgSnakeHead) {
        this.m_newImgSnakeHead = newImgSnakeHead;
    }


    // Variables
    private BufferedImage m_IMG_SNAKE_HEAD;
    private BufferedImage m_newImgSnakeHead;

    // Local variable
    int sn;
    int SN_2 = 2;
    int SN_3 = 3;
    int SET_SPEED = 5;
    int ROTATION = 90;
    int ROTATION1 = 180;
    int INT_W = 870;
    int INT_H = 560;


    // Methods
    /**
     * This method is the constructor of this class
     * Outputs the snake body onto the screen using the rectangle measurements
     * But it also checks speed of snake
     *
     * @param x
     * @param y
     * @param s
     * @param sn
     */
    public SnakeView(int x, int y, int s, int sn)
    {
        this.SetL(true);
        this.SetX(x);
        this.SetY(y);
        this.SetS(s);
        this.sn = sn;

        ImagesModel images = new ImagesModel();

        if (sn == 1) {

            this.SetImg(images.images.get("snakeBody2"));
            this.SetIMG_SNAKE_HEAD((BufferedImage) images.images.get("snakeHead2"));

        } else if (sn == SN_2 || sn == 0) {

            this.SetImg(images.images.get("snakeBody"));
            this.SetIMG_SNAKE_HEAD((BufferedImage) images.images.get("snakeHead"));

        } else if (sn == SN_3) {

            this.SetImg(images.images.get("snakeBody3"));
            this.SetIMG_SNAKE_HEAD((BufferedImage) images.images.get("snakeHead3"));

        }

        this.SetW(GetImg().getWidth(null));
        this.SetH(GetImg().getHeight(null));

        this.SetLength(1);

        this.SetSpeed_XY(SET_SPEED);
        SetNewImgSnakeHead(GetIMG_SNAKE_HEAD());
        this.SetNum(GetW() / GetSpeed_XY());

    }

    /**
     * This method listens for when a key is pressed (input)
     * Then makes the snake move according to the key which has been pressed
     * @param e
     */
    public void KeyPressed(KeyEvent e)
    {
        // Checks the key being pressed
        switch (e.getKeyCode())
        {
            case KeyEvent.VK_UP:
                if (!GetDown())
                {
                    SetUp(true);
                    SetDown(false);
                    SetLeft(false);
                    SetRight(false);

                    SetNewImgSnakeHead((BufferedImage) new ImagesView().RotateImage(
                            GetIMG_SNAKE_HEAD(), -ROTATION));

                }
                break;

            case KeyEvent.VK_DOWN:
                if (!GetUp())
                {
                    SetUp(false);
                    SetDown(true);
                    SetLeft(false);
                    SetRight(false);

                    SetNewImgSnakeHead((BufferedImage) new ImagesView().RotateImage(
                            GetIMG_SNAKE_HEAD(), ROTATION));
                }
                break;

            case KeyEvent.VK_LEFT:
                if (!GetRight())
                {
                    SetUp(false);
                    SetDown(false);
                    SetLeft(true);
                    SetRight(false);

                    SetNewImgSnakeHead((BufferedImage) new ImagesView().RotateImage(
                            GetIMG_SNAKE_HEAD(), -ROTATION1));

                }
                break;

            case KeyEvent.VK_RIGHT:
                if (!GetLeft())
                {
                    SetUp(false);
                    SetDown(false);
                    SetLeft(false);
                    SetRight(true);

                    SetNewImgSnakeHead(GetIMG_SNAKE_HEAD());
                }

            default:
                break;
        }
    }

    /**
     * This method increases the speed of the snake's movement
     */
    public void Move()
    {

        if (GetUp())
        {

            y -= GetSpeed_XY();

        } else if (GetDown())
        {

            y += GetSpeed_XY();

        } else if (GetLeft())
        {

            x -= GetSpeed_XY();

        } else if (GetRight())
        {

            x += GetSpeed_XY();

        }

    }

    /**
     * This method is responsible for drawing the snake onto the screen and managing its length throughout the entire game
     * It also adds the rest of the snake's body as it eats food by increasing its length
     * Checks if the snake is out of bounds or if it ate itself (if so game ends)
     * Makes the snake move so game can start
     * @param g
     */
    @Override
    public void Draw(Graphics g)
    {
        // Checks if snake is out of bounds or if it has eaten its body
        // If so then game will end
        outofBounds();
        EatBody();

        // bodyPoints adds 5 everytime a new food item is eaten by the snake
        GetList().add(new Point(GetX(), GetY()));

        if (GetList().size() == (this.GetLength() + 1) * GetNum())
        {
            GetList().remove(0);
        }

        // Inserts the snake head
        g.drawImage(GetNewImgSnakeHead(), GetX(), GetY(),null);
        // Adds more to the snake's body as it eats food
        DrawBody(g);

        // Makes the snake move otherwise game would end instantly
        Move();

    }

    /**
     *  This method checks if snake has eaten its body, if yes then game ends
     */
    public void EatBody()
    {
        // Whilst snake has not eaten its body continue
        for (Point point : GetList())
        {
            for (Point point2 : GetList())
            {
                // This if statement checks if snake ate its body
                if (point.equals(point2) && point != point2)
                {
                    this.SetL(false);
                }
            }
        }
    }

    /**
     * This method checks the snake's length and then adds more of the body on screen (length increases)
     * @param g
     */
    public void DrawBody(Graphics g)
    {

        int length = GetList().size() - 1 - GetNum();

        // As the length increases then add more of the snake's body using drawImage()
        for (int i = length; i >= GetNum(); i -= GetNum())
        {
            Point point = GetList().get(i);

            g.drawImage(this.GetImg(), point.x, point.y, null);
        }
    }

    /**
     * This method checks if snake is out of bounds
     * It does this by checking all corners (using coordinates)
     * If snake is out of bounds then game ends
     */
    private void outofBounds()
    {
        // Rule 9: Symbolic Constant
        boolean xOut = (GetX() <= 0 || GetX() >= (INT_W - GetW()));
        // Change y <= 40 to y <= 0 to make sure the snake goes all the way up
        boolean yOut = (GetY() <= 0 || GetY() >= (INT_H - GetH()));
        // If snake is out of bounds end game
        if (xOut || yOut)
        {
            SetL(false);
        }
    }

}
