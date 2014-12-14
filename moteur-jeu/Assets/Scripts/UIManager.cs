using UnityEngine;
using UnityEngine.UI;
using System.Collections;

public class UIManager : MonoBehaviour {

	public int difficulty = 50;
	private bool isSettingsUIActive;
	GameObject panelSettings; 
	Slider sliderDifficulty;
	Text sliderDifficultyText;

	// Called at initialisation
	public void Start()
	{
		// Get the Settings panel
		panelSettings = GameObject.Find ("Panel Settings"); 

		// Get the difficulty slider and get and set the difficulty text
		GameObject slider = GameObject.Find ("Difficulty Slider");
			sliderDifficulty = slider.GetComponent<Slider>();
		GameObject text = GameObject.Find ("Difficulty Text");
			sliderDifficultyText = text.GetComponent<Text>();
			sliderDifficultyText.text = "50";

		// Hide the settings panel
		panelSettings.SetActive (false);
		isSettingsUIActive = false;


	}

	// Load the game
	public void changeLevel(string level)
	{
		if(level == "Quit")
			Application.Quit();
		else
			Application.LoadLevel (level);
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
	public void setDifficulty()
	{
		difficulty = Mathf.RoundToInt(sliderDifficulty.value);
		sliderDifficultyText.text = difficulty.ToString();
	}
}
