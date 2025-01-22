package Controller;

import Factory_pattern.*;
import Models.EndScreenModel;
import Models.ImagesModel;
import Models.StartScreenModel;
import Singleton.MusicPlayer;
import View.*;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.control.Alert;

import javax.swing.*;
import java.awt.*;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import java.awt.event.KeyEvent;

import static Factory_pattern.Obstacles.GetTouched;
import static View.EndScreenView.*;

/**
 * @Project snakee_2022
 * @Author Carolina Louro
 * @File GameController.java
 * @Description This class outputs all graphics seen on screen, shows a visualisation of the all items
 * This class also take in input and passes it on to the other classes
 * @Version 19
 */

public class GameController extends StartScreenModel
{

	// Getters and Setters for class variables
	/**
	 * This method is a getter for the MusicPlayer m_music
	 * @return m_music
	 */
	public static MusicPlayer GetMusic() {
		return m_music;
	}

	/**
	 * This method is a setter for the MusicPlayer m_music
	 * @param m
	 */
	public static void SetMusic(MusicPlayer m) {
		m_music = m;
	}

	/**
	 * This method is a getter for the ImagesModel images
	 * @return m_images
	 */
	public ImagesModel GetImage() {
		return this.images;
	}

	/**
	 * This method is a setter for the ImagesModel images
	 * @param i
	 */
	public void SetImage(ImagesModel i) {
		this.images = i;
	}

	/**
	 * This method is a getter for the integer m_speed
	 * @return m_speed
	 */
	public int GetSpeed()
	{
		return m_speed;
	}

	/**
	 * This method is a setter for the integer m_speed
	 * @param speed
	 */
	public void SetSpeed(int speed)
	{
		this.m_speed = speed;
	}

	/**
	 * This method is a getter for the string m_pString
	 * @return m_pString
	 */
	public String GetM_pString() {
		return m_pString;
	}

	/**
	 * This method is a setter for the string m_pString
	 * @param m_pString
	 */
	public void SetM_pString(String m_pString) {
		this.m_pString = m_pString;
	}

	/**
	 * This method is a getter for the integer m_b
	 * @return m_b
	 */
	public int GetB() {
		return m_b;
	}

	/**
	 * This method is a setter for the integer m_b
	 * @param b
	 */
	public void SetB(int b) {
		this.m_b = b;
	}

	/**
	 * This method is a getter for the integer m_sn
	 * @return m_sn
	 */
	public int GetSn() {
		return m_sn;
	}

	/**
	 * This method is a setter for the integer m_sn
	 * @param sn
	 */
	public void SetSn(int sn) {
		this.m_sn = sn;
	}

	/**
	 * This method is a setter for the integer m_counter
	 * @return m_counter
	 */
	public int GetCounter() {
		return m_counter;
	}

	/**
	 * This method is a setter for the integer m_counter
	 * @param counter
	 */
	public void SetCounter(int counter) {
		this.m_counter = counter;
	}

	/**
	 * This method is a getter for the boolena m_check
	 * @return m_check
	 */
	public boolean GetCheck() {
		return m_check;
	}

	/**
	 * This method is a setter for the boolean m_check
	 * @param check
	 */
	public void SetCheck(boolean check) {
		this.m_check = check;
	}

	/**
	 * This method is a getter for the boolean m_medium
	 * @return m_medium
	 */
	public boolean GetMedium() {
		return m_medium;
	}

	/**
	 * This method is a setter for the boolean m_medium
	 * @param medium
	 */
	public void SetMedium(boolean medium) {
		this.m_medium = medium;
	}

	/**
	 * This method is a getter for the boolean m_hard
	 * @return m_hard
	 */
	public boolean GetHard() {
		return m_hard;
	}

	/**
	 * This method is a setter for the boolean m_hard
	 * @param hard
	 */
	public void SetHard(boolean hard) {
		this.m_hard = hard;
	}


	// Variables
	private final long serialVersionUID = -3641221053272056036L;
	public SnakeView mySnake;
	private static MusicPlayer m_music = MusicPlayer.GetInstance();
	private Food food = (Food) FoodFFactory.GetInstance().GetGameObjectsModel(
			FoodFEnum.FOOD_VIEW);
	private Bonus bonus = (Bonus) BonusFFactory.GetInstance().GetGameObjectsModel(
			BonusFEnum.BONUS);
	private Obstacles obstacles = (Obstacles) ObstaclesFFactory.GetInstance()
			.GetGameObjectsModel(ObstaclesFEnum.OBSTACLES_VIEW);
	private Obstacles obstacles2 = (Obstacles) ObstaclesFFactory.GetInstance()
			.GetGameObjectsModel(ObstaclesFEnum.OBSTACLES_VIEW);
	private Obstacles obstacles3 = (Obstacles) ObstaclesFFactory.GetInstance()
			.GetGameObjectsModel(ObstaclesFEnum.OBSTACLES_VIEW);
	private Obstacles obstacles4 = (Obstacles) ObstaclesFFactory.GetInstance()
			.GetGameObjectsModel(ObstaclesFEnum.OBSTACLES_VIEW);
	private Obstacles obstacles5 = (Obstacles) ObstaclesFFactory.GetInstance()
			.GetGameObjectsModel(ObstaclesFEnum.OBSTACLES_VIEW);
	private Obstacles obstacles6 = (Obstacles) ObstaclesFFactory.GetInstance()
			.GetGameObjectsModel(ObstaclesFEnum.OBSTACLES_VIEW);
	private ImagesModel images = new ImagesModel();
	private Image background = images.images.get("UI-background");

	private Image lives = images.images.get("lives");

	private Image fail = images.images.get("endGame");

	private String m_pString;
	private int m_b;
	private int m_sn;
	private int m_counter = 4;
	private int m_speed;
	private boolean m_check;
	private boolean m_medium;
	private boolean m_hard;
	// Symbolic variables
	int XY_VARIABLE = 100;
	int DRAWX_VARIABLE = 20;
	int DRAWY_VARIABLE = 40;
	int DRAWX2_VARIABLE = 390;
	int Y_VARIABLE = 5;
	int DRAW_IMG1 = 230;
	int DRAW_IMG2 = 265;
	int DRAW_IMG3 = 300;


	// Methods

	/**
	 * This is the class's constructor
	 * This method makes changes according to all user input, such as level chosen (which affects speed, etc)
	 * @param s
	 * @param b
	 * @param speed
	 * @param m
	 * @param h
	 * @param snakeColour
	 */
	public GameController(String s, int b, int speed, boolean m, boolean h, int snakeColour) {

		super(s, b, speed);
		this.SetM_pString(s);
		this.SetB(b);
		this.SetSpeed(speed);
		this.SetMedium(m);
		this.SetHard(h);
		this.SetSn(snakeColour);
		SetCheck(true);
		this.mySnake = new SnakeView(XY_VARIABLE, XY_VARIABLE, GetSpeed(), GetSn());

		// Changes speed according to the button pressed (mode chosen) by the player
		if (speed == 1) {

			speed = 5;
			mySnake.SetSpeed_XY(speed);

		} else if (speed == 2) {

			speed = 10;
			mySnake.SetSpeed_XY(speed);

		} else if (speed == 3) {

			speed = 10;
			mySnake.SetSpeed_XY(speed);

		}

		if (GetMedium() == true ||  GetHard() == true) {

			GetMusic().play("src/main/resources/warning.mp3");
			Alert alert = new Alert(Alert.AlertType.WARNING);
			alert.setTitle("Warning Dialog");
			alert.setHeaderText("Look out for the Bombs!");
			alert.setContentText("If you catch a bomb you loose a life (max lives = 3).");
			alert.showAndWait();

		}

	}

	/**
	 * This method listens for the keyboard input
	 * When a key is pressed, the snake moves accordingly
	 * @param e the event to be processed
	 */
	@Override
	public void keyPressed(KeyEvent e)
	{
		super.keyPressed(e);
		mySnake.KeyPressed(e);
	}

	/**
	 * This method adds all graphics to the game screen according to input
	 * This method also determines the state of the game by checking:
	 * whilst game is still on then draw snake's body and all obstacles/items on screen
	 * if snake has touched another object then an event will occur accordingly
	 * responsible for ending game if either all lives are lost or snake hits the wall
	 * also responsible for setting the background images and levels' elementss according to the user input
	 * @param g
	 */
	@Override
	public void paint(Graphics g)
	{
		int i = 0, y = 0, scorep;
		super.paint(g);
		ImagesModel images = new ImagesModel();

		// Changes background depending on the player's choice in the Play_JavaFX. java class
		if (GetB() == 0 || GetB() == 1) {

			g.drawImage(background, 0, 0, null);

		} else {

			g.drawImage(images.images.get("new"), 0, 0, null);

		}

		// Determine the state of the game
		if (mySnake.l) {
			// Draws the snake so user can see it on screen when playing
			mySnake.Draw(g);

			if (food.l) {
				// Draws the food on screen so snake can eat it
				food.Draw(g);
				food.Eaten(mySnake);

				if (bonus.l) {

					if (GetMedium() == true) {

						bonus.Draw(g);
						bonus.Touched((mySnake));

					} else if (GetHard() == true) {

						bonus.Draw(g);
						bonus.Touched((mySnake));

					}

				}

				if (obstacles.l) {

					if (GetMedium() == true) {

						obstacles.Draw(g);
						obstacles.Touched((mySnake));

						if (GetTouched() == true) {

							SetCounter(GetCounter() - 1);

						}

					} else if (GetHard() == true) {

						obstacles.Draw(g);
						obstacles2.Draw(g);
						obstacles3.Draw(g);
						obstacles4.Draw(g);
						obstacles5.Draw(g);
						obstacles6.Draw(g);

						obstacles.Touched((mySnake));
						obstacles2.Touched((mySnake));
						obstacles3.Touched((mySnake));
						obstacles4.Touched((mySnake));
						obstacles5.Touched((mySnake));
						obstacles6.Touched((mySnake));

						if (GetTouched() == true) {

							SetCounter(GetCounter() - 1);

						}

					}

				}

			} else {

				food = (Food) FoodFFactory.GetInstance().GetGameObjectsModel(
						FoodFEnum.FOOD_VIEW);

				if (GetMedium() == true) {

					bonus = (Bonus) BonusFFactory.GetInstance().GetGameObjectsModel(
							BonusFEnum.BONUS);
					obstacles = (Obstacles) ObstaclesFFactory.GetInstance().GetGameObjectsModel(
							ObstaclesFEnum.OBSTACLES_VIEW);

				} else if (GetHard() == true) {

					bonus = (Bonus) BonusFFactory.GetInstance().GetGameObjectsModel(
							BonusFEnum.BONUS);
					obstacles = (Obstacles) ObstaclesFFactory.GetInstance()
							.GetGameObjectsModel(ObstaclesFEnum.OBSTACLES_VIEW);
					obstacles2 = (Obstacles) ObstaclesFFactory.GetInstance()
							.GetGameObjectsModel(ObstaclesFEnum.OBSTACLES_VIEW);
					obstacles3 = (Obstacles) ObstaclesFFactory.GetInstance()
							.GetGameObjectsModel(ObstaclesFEnum.OBSTACLES_VIEW);
					obstacles4 = (Obstacles) ObstaclesFFactory.GetInstance()
							.GetGameObjectsModel(ObstaclesFEnum.OBSTACLES_VIEW);
					obstacles5 = (Obstacles) ObstaclesFFactory.GetInstance()
							.GetGameObjectsModel(ObstaclesFEnum.OBSTACLES_VIEW);
					obstacles6 = (Obstacles) ObstaclesFFactory.GetInstance()
							.GetGameObjectsModel(ObstaclesFEnum.OBSTACLES_VIEW);

				}

			}

			// This method shows the score at the top of the screen when playing
			DrawScore(g);

		} else
		{

			EndGame();

		}

		if (mySnake.l == false) {

			if (GetCheck() == true) {

				// Stores the player's score to be output in the highscore board
				Map map = new HashMap();
				scorep = mySnake.GetScore();
				map.put(GetM_pString(), scorep);
				// Calls the HighScore class to output the player's score and store it
				EndScreenModel.WriteCSV(map);
				SetCheck(false);

			}

		}

	}

	/**
	 * This method outputs a pop-up message with the player's score
	 */
	public void EndGame() {

		JFrame frame = new JFrame();
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.setTitle("Score");
		JLabel textLabel = new JLabel("Your Score = " + mySnake.GetScore(),SwingConstants.CENTER);
		textLabel.setPreferredSize(new Dimension(400, 200));
		frame.getContentPane().add(textLabel, BorderLayout.CENTER);
		frame.setLocationRelativeTo(null);
		frame.pack();
		frame.setVisible(true);

	}

	/**
	 * This method is responsible for choosing all the visual for the Player's name and the score at the top of game screen
	 * Also, keeps count of when the snake touched the obstacle so that it can output lives out on the screen
	 * @param g
	 */
	// Rule 8: Method Naming
	public void DrawScore(Graphics g)
	{
		int sc, i = 0;
		sc = mySnake.GetScore();

		g.setFont(new Font(Font.SANS_SERIF, Font.BOLD, 30));
		g.setColor(Color.MAGENTA);
		g.drawString("SCORE : " + sc, DRAWX_VARIABLE, DRAWY_VARIABLE);

		// Displays player's name whilst game is running
		g.drawString(GetM_pString(), DRAWX2_VARIABLE, DRAWY_VARIABLE);

		if (GetCounter() == 3) {

			g.drawImage(lives, DRAW_IMG1, Y_VARIABLE, null);
			g.drawImage(lives, DRAW_IMG2, Y_VARIABLE, null);
			g.drawImage(lives, DRAW_IMG3, Y_VARIABLE, null);


		} else if (GetCounter() == 2) {

			g.drawImage(lives, DRAW_IMG1, Y_VARIABLE, null);
			g.drawImage(lives, DRAW_IMG2, Y_VARIABLE, null);

		} else if (GetCounter() == 1) {

			g.drawImage(lives, DRAW_IMG1, Y_VARIABLE, null);

		} else if (GetCounter() == 0) {

			SetCounter(4);
			mySnake.l = false;

		}

	}

}
