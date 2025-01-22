package Models;

import javax.swing.*;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

/**
 * @Project snakee_2022
 * @Author Carolina Louro
 * @File StartScreenModel.java
 * @Description This class is the view class (MVC) of the FrameModel class
 * Responsible for the frame/panel of the game and outputs all game functionality on screen
 * @Version 19
 */

public class StartScreenModel extends JPanel implements KeyListener
{

	// Getters and Setters
	/**
	 * This method is a getter for the JFrame m_jFrame
	 * @return m_jFrame
	 */
	protected JFrame GetFrame() {
		return this.m_jFrame;
	}

	/**
	 * This method is a setter for the JFrame m_jFrame
	 * @param frame
	 */
	private void SetFrame(JFrame frame) {
		this.m_jFrame = frame;
	}


	// Variables
	private final long serialVersionUID = -3149926831770554380L;

	private JFrame m_jFrame = new JFrame();


	// Methods
	/**
	 * Constructor of this class
	 * @param t
	 * @param backg
	 * @param s
	 */
	public StartScreenModel(String t, int backg, int s) {
		super();
	}

	/**
	 * This method loads the game panel on screen using a new thread
	 * Joins all functionality so all elements can be put together to make the game work
	 */
	public void LoadFrame()
	{

		this.setDoubleBuffered(true);
		GetFrame().add(this);
		GetFrame().addKeyListener(this);

		// Shows the name of the game at the top of screen
		GetFrame().setTitle("Snakee Yipee");
		// Sets the size of the game on screen
		GetFrame().setSize(870, 560);
		GetFrame().setLocationRelativeTo(null);

		GetFrame().addWindowListener(new WindowAdapter()
		{
			@Override
			public void windowClosing(WindowEvent e)
			{
				super.windowClosing(e);
				System.exit(0);
			}
		});

		GetFrame().setVisible(true);
		GetFrame().setDefaultCloseOperation(GetFrame().EXIT_ON_CLOSE);

		// Starts a new game when previous one ends
		new MyThread().start();

	}


	/**
	 * This method runs the game by calling a new thread
	 */
	public class MyThread extends Thread
	{
		@Override
		public void run()
		{
			super.run();

			while (true)
			{
				repaint();
				try
				{
					sleep(30);

				} catch (Exception e)
				{
					e.printStackTrace();
				}
			}
		}
	}

	/**
	 * This method listens for when a key is typed (input)
	 * @param e
	*/
	@Override
	public void keyTyped(KeyEvent e)
	{
		// TODO Auto-generated method stub

	}

	/**
	 * This method listens for when a key is pressed (input)
	 * This method is overwritten in the GameController() class
	 * @param e
	 */
	@Override
	public void keyPressed(KeyEvent e)
	{
		// TODO Auto-generated method stub

	}

	/**
	 * This method listens for when a key is released after being pressed (input)
	 * @param e
	 */
	@Override
	public void keyReleased(KeyEvent e)
	{
		// TODO Auto-generated method stub

	}

}
