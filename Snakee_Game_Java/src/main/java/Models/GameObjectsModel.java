package Models;

import java.awt.*;
import java.util.LinkedList;
import java.util.List;

/**
 * @Project snakee_2022
 * @Author Carolina Louro
 * @File GameObjectsModel.java
 * @Description This class acts as the Model class (MVC model) for the Snake, Food, Obstacle objects
 * Contains all the getters and setters used to create the these objects within the game
 * Calls the draw() method to draw any element out on screen using rectangle measurements called by the getRectangle() method
 * @Version 19
 */

public abstract class GameObjectsModel {

    // Getters and Setters
    /**
     * This method is a getter for the integer x
     * Variable x is used to define the rectangle measurements for any object
     * @return x
     */
    public int GetX() {
        return this.x;
    }

    /**
     * This method is a setter for the integer x
     * Variable x is used to define the rectangle measurements for any object
     * @param x
     */
    public void SetX(int x) {
        this.x = x;
    }

    /**
     * This method is a getter for the integer y
     * Variable y is used to define the rectangle measurements for any object
     * @return y
     */
    public int GetY() {
        return this.y;
    }

    /**
     * This method is a setter for the integer y
     * Variable y is used to define the rectangle measurements for any object
     * @param y
     */
    public void SetY(int y) {
        this.y = y;
    }

    /**
     * This method is a getter for the integer m_s
     * @return m_s
     */
    public int GetS() {
        return this.m_s;
    }

    /**
     * This method is a setter for the integer m_s
     * @param s
     */
    public void SetS(int s) {
        this.m_s = s;
    }

    /**
     * This method is a getter for the integer m_w
     * Variable m_w is used to define the rectangle measurements for any object
     * @return m_w
     */
    public int GetW() {
        return this.m_w;
    }

    /**
     * This method is a setter for the integer m_w
     * Variable m_w is used to define the rectangle measurements for any object
     * @param w
     */
    public void SetW(int w) {
        this.m_w = w;
    }

    /**
     * This method is a getter for the integer m_h
     * Variable m_h is used to define the rectangle measurements for any object
     * @return m_h
     */
    public int GetH() {
        return this.m_h;
    }

    /**
     * This method is a setter for the integer m_h
     * Variable m_h is used to define the rectangle measurements for any object
     * @param h
     */
    public void SetH(int h) {
        this.m_h = h;
    }

    /**
     * This method is a getter for the boolean l
     * @return l
     */
    public boolean GetL() {
        return this.l;
    }


    /**
     * This method is a setter for the boolean l
     * @param l
     */
    public void SetL(boolean l) {
        this.l = l;
    }

    /**
     * This method is a getter for the Image m_i
     * @return m_i
     */
    public Image GetImg() {
        return this.m_i;
    }

    /**
     * This method is a setter for the Image m_i
     * @param img
     */
    public void SetImg(Image img) {
        this.m_i = img;
    }

    /**
     * This method is a getter for the integer m_speed_XY
     * The variable m_speed_XY gets the speed of the snake object in the game
     * @return m_speed_XY
     */
    public int GetSpeed_XY() {
        return this.m_speed_XY;
    }

    /**
     * This method is a setter for the integer m_speed_XY
     * The variable m_speed_XY sets the speed of the snake object in the game
     * @param s
     */
    public void SetSpeed_XY(int s) {
        this.m_speed_XY = s;
    }

    /**
     * This method is a getter for the integer m_length
     * The variable m_length gets the length of snake's body
     * @return m_length
     */
    public int GetLength()
    {
        return m_length;
    }

    /**
     * This method is a setter for the integer m_length
     * The variable m_length sets the length of snake's body
     * @param length
     */
    public void SetLength(int length)
    {
        this.m_length = length;
    }

    /**
     * This method is a getter for the integer m_num
     * @return m_num
     */
    protected int GetNum() {
        return this.m_num;
    }

    /**
     * This method is a setter for the integer m_num
     * @param n
     */
    protected void SetNum(int n) {
        this.m_num = n;
    }

    /**
     * This method is a getter for the integer m_score
     * @return m_score
     */
    public int GetScore() {
        return this.m_score;
    }

    /**
     * This method is a setter for the integer m_score
     * @param sc
     */
    public void SetScore(int sc) {
        this.m_score = sc;
    }

    /**
     * This method is a getter for the List<Point> m_bodyPoints
     * @return m_bodyPoints
     */
    protected List<Point> GetList() {
        return this.m_bodyPoints;
    }

    /**
     * This method is a setter for the List<Point> m_bodyPoints
     * @param bp
     */
    protected void SetList(List<Point> bp) {
        this.m_bodyPoints = bp;
    }

    /**
     * This method is a getter for the boolean m_up
     * The variable m_up is used to indicate directions for the snake
     * @return m_up
     */
    protected boolean GetUp() {
        return this.m_up;
    }

    /**
     * This method is a setter for the boolean m_up
     * The variable m_up is used to indicate directions for the snake
     * @param u
     */
    protected void SetUp(boolean u) {
        this.m_up = u;
    }

    /**
     * This method is a getter for the boolean m_down
     * The variable m_down is used to indicate directions for the snake
     * @return m_down
     */
    protected boolean GetDown() {
        return this.m_down;
    }

    /**
     * This method is a setter for the boolean m_down
     * The variable m_down is used to indicate directions for the snake
     * @param d
     */
    protected void SetDown(boolean d) {
        this.m_down = d;
    }

    /**
     * This method is a getter for the boolean m_right
     * The variable m_right is used to indicate directions for the snake
     * @return m_right
     */
    protected boolean GetRight() {
        return this.m_right;
    }

    /**
     * This method is a setter for the boolean m_right
     * The variable m_right is used to indicate directions for the snake
     * @param r
     */
    protected void SetRight(boolean r) {
        this.m_right = r;
    }

    /**
     * This method is a getter for the boolean m_left
     * The variable m_left is used to indicate directions for the snake
     * @return m_left
     */
    protected boolean GetLeft() {
        return this.m_left;
    }

    /**
     * This method is a getter for the boolean m_left
     * The variable m_left is used to indicate directions for the snake
     * @param lf
     */
    protected void SetLeft(boolean lf) {
        this.m_left = lf;
    }


    // Variables
    private int m_speed_XY;
    private int m_length;
    private int m_num;
    private int m_score = 0;
    private List<Point> m_bodyPoints = new LinkedList<>();
    boolean m_up, m_down, m_left, m_right = true;
    protected int x;
    protected int y;
    protected int m_s;
    protected Image m_i;
    protected int m_w;
    protected int m_h;

    // This variable is not a class member so can not be private
    public boolean l;


    // Methods
    /**
     * This method draws any object's graphics onto screen
     * Objects include: snake (head and body), food elements and obstacles
     * @param g
     */
    public abstract void Draw(Graphics g);

    /**
     * This method uses a rectangle measurement to measure the size of any object
     * This is useful to help with collisions/objects touching
     * @return Rectangle()
     */
    public Rectangle GetRectangle()
    {

        return new Rectangle(GetX(), GetY(), GetW(), GetH());

    }

}
