using UnityEngine;
using System.Collections;

public class ChangeSceneOnClick : MonoBehaviour {
	
	// Update is called once per frame
	public void Update () 
	{
		if (Input.GetMouseButtonDown (0) || (Input.touches != null && Input.touches.Length > 0))
						Application.Quit ();
	}
}
