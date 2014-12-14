using UnityEngine;
using System.Collections;

public class UIManager : MonoBehaviour {
	
	public void startGame()
	{
		Application.LoadLevel ("Game");
	}

	public void quit()
	{
		Application.Quit ();
	}
}
