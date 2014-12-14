using UnityEngine;
using System.Collections;

public class UIManager : MonoBehaviour {

	public float difficulty = 1f;

	public void startGame()
	{
		Application.LoadLevel ("Game");
	}

	public void quit()
	{
		Application.Quit ();
	}

	public void showSettings()
	{
		GameObject panelSettings = GameObject.FindGameObjectWithTag("Panel Settings");

		panelSettings.SetActive (true);
	}

	public float getDifficulty()
	{
		return difficulty;
	}
}
