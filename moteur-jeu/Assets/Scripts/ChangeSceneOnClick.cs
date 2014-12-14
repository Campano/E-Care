using UnityEngine;
using System.Collections;

public class ChangeSceneOnClick : MonoBehaviour {

	public string nextScene = "";
	
	// Update is called once per frame
	public void start () 
	{
		if (Input.GetMouseButtonDown (0) || (Input.touches != null && Input.touches.Length > 0)) 
		{
			if (nextScene != "Quit")
				Application.LoadLevel (nextScene);
			else 
				Application.Quit ();
		}
	}
}
