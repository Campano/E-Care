using UnityEngine;
using UnityEngine.UI;
using System.Collections;

public class UIManager : MonoBehaviour {

	public float difficulty = 1f;
	private bool isSettingsUIActive;
	GameObject panelSettings; 
	Slider sliderDifficulty;

	// Called at initialisation
	public void Start()
	{
		// Get the Settings panel and hide it
		panelSettings = GameObject.Find ("Panel Settings"); 
		panelSettings.SetActive (false);
		isSettingsUIActive = false;

		// Get the difficulty slider
		GameObject slider = GameObject.Find ("Difficulty Slider");
		sliderDifficulty = slider.GetComponent<Slider>();

	}

	// Load the game
	public void startGame()
	{
		Application.LoadLevel ("Game");
	}

	// Quit he application
	public void quit()
	{
		Application.Quit ();
	}

	// Show/Hide Settings panel
	public void showSettings()
	{
		if (isSettingsUIActive) 
		{
			panelSettings.SetActive (false);
			isSettingsUIActive = false;
		}
		else
		{
			panelSettings.SetActive (true);
			isSettingsUIActive = true;
		}
	}

	// Getter for (float)difficulty
	public float getDifficulty()
	{
		return difficulty;
	}

	// Setter for (float)difficulty
	public void setDifficulty(float d)
	{
		difficulty = sliderDifficulty.value;
	}
}
