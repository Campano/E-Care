using UnityEngine;
using UnityEngine.UI;
using System.Collections;

public class UIManagerMainMenu : MonoBehaviour {

	// Load the game
	public void changeLevel(string level)
	{
		if(level == "Quit")
			Application.Quit();
		else
			Application.LoadLevel (level);
	}
}
