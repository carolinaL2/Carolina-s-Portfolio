package View;

import Models.ImagesModel;

import java.awt.*;
import java.awt.image.BufferedImage;

/**
 * @Project snakee_2022
 * @Author Carolina Louro
 * @File ImagesView.java
 * @Description This class is a view class (MVC) for the ImagesModel class
 * This class is responsible for rotating images according to a specific degree
 * @Version 19
 */

public class ImagesView extends ImagesModel {

    // Methods
    /**
     * This method is responsible for rotating the snake's body according to a degree provided
     * @param b
     * @param degree
     * @return i
     */
    public Image RotateImage(final BufferedImage b, final int degree)
    {

        int w = b.getWidth();
        int h = b.getHeight();
        int t = b.getColorModel().getTransparency();

        BufferedImage i;
        Graphics2D graphics2d;

        i = new BufferedImage(w, h, t);
        graphics2d = i.createGraphics();

        graphics2d.setRenderingHint(RenderingHints.KEY_INTERPOLATION,
                RenderingHints.VALUE_INTERPOLATION_BILINEAR);

        graphics2d.rotate(Math.toRadians(degree), w / 2, h / 2);
        graphics2d.drawImage(b, 0, 0, null);
        graphics2d.dispose();

        return i;

    }

}
