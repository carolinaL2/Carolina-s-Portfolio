package Models;

import View.ImagesView;

import javax.imageio.ImageIO;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.net.URL;
import java.util.HashMap;
import java.util.Map;

/**
 * @Project snakee_2022
 * @Author Carolina Louro
 * @File ImagesModel.java
 * @Description This class acts as the class model (MVC) and obtains all images needed to run the game
 * Includes all getters and setters needed to obtain these images
 * As well as a class initializer with all the links to the images required by the game
 * @Version 19
 */

public class ImagesModel {

    // Getters and Setters
    /**
     * This method is a getter for the Map images
     * @return images
     */
    public Map GetMap() {
        return this.images;
    }

    /**
     * This method is a setter for the Map images
     * @param img
     */
    public void SetMap(Map img) {
        this.images = img;
    }


    // Variables
    public Map<String, Image> images = new HashMap<>();


    // Methods
    /**
     * This method looks for the location of the image needed
     * If image not found then an error message is printed onto the screen
     * @param imagePath
     * @return i
     */
    public Image getImage(String imagePath)
    {
        URL u = ImagesView.class.getClassLoader().getResource(imagePath);
        BufferedImage i = null;

        // Checks if image was found if not then an error message is printed on screen
        try
        {
            i = ImageIO.read(u);
        } catch (Exception e)
        {
            System.err.println("ERROR: Image not found!\n");
            e.printStackTrace();
        }

        return i;
    }

    /**
     * This class initializer obtains the location of all image files' required and puts them in a Map
     */
    {
        // Snake Images
        GetMap().put("snakeHead", getImage("snakeHeadMain.png"));
        GetMap().put("snakeBody", getImage("snakeBody.png"));
        // Food Items
        GetMap().put("0", getImage("kiwi.png"));
        GetMap().put("1", getImage("lemon.png"));
        GetMap().put("2", getImage("litchi.png"));
        GetMap().put("3", getImage("mango.png"));
        GetMap().put("4", getImage("apple.png"));
        GetMap().put("5", getImage("banana.png"));
        GetMap().put("6", getImage("blueberry.png"));
        GetMap().put("7", getImage("cherry.png"));
        GetMap().put("8", getImage("durian.png"));
        GetMap().put("9", getImage("grape.png"));
        GetMap().put("10", getImage("grapefruit.png"));
        GetMap().put("11", getImage("peach.png"));
        GetMap().put("12", getImage("pear.png"));
        GetMap().put("13", getImage("orange.png"));
        GetMap().put("14", getImage("pineapple.png"));
        GetMap().put("15", getImage("strawberry.png"));
        GetMap().put("16", getImage("watermelon.png"));
        // My own additions
        GetMap().put("19", getImage("bomb.png"));
        GetMap().put("20", getImage("bonus.png"));
        GetMap().put("lives", getImage("lives.png"));
        GetMap().put("snakeHead2", getImage("snakeHeadMain2.png"));
        GetMap().put("snakeHead3", getImage("snakeHeadMain3.png"));
        GetMap().put("snakeBody2", getImage("snakeBody2.png"));
        GetMap().put("snakeBody3", getImage("snakeBody3.png"));
        // Backgrounds
        GetMap().put("UI-background", getImage("CloudsBg.png"));
        GetMap().put("new", getImage("SunCloudsBg.png"));
        GetMap().put("endGame", getImage("endGameImg.png"));
    }

}
