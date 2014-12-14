using UnityEngine;
using UnityEngine.UI;
using System.Collections;

public class ConnexionManagerScript : MonoBehaviour {

	Text text;
	GameObject connexionButton;


	// Use this for initialization
	public void Start ()
	{
		// Get the GameObjects and the text component
		GameObject connexionText = GameObject.Find("Connexion Status");
			text = connexionText.GetComponent<Text>();
		connexionButton = GameObject.Find("Connexion Button");

		// Hide retry button
		connexionButton.SetActive (false);

		// Launch connexion process
		connectSensors ();
	}


	// Manage the connection
	public void connectSensors()
	{

		updateStatus ("connecting");

		// INTEGRATION ====================================================================================
		//						 		LANCER CONNEXION
		//						
		//						
		//								if(CONNEXION FAILED)
		//									updateStatus ("error");
		//						
		//								if(CONNEXION SUCCESS)
		//									updateStatus ("connected");
		// ==================================================================================== INTEGRATION
	}


	// Update the text and the retry button according to the status
	private void updateStatus (string status) {
		switch (status) 
		{
			case "connecting": 
				text.text = "Connexion des capteurs en cours ...";
			break;

			case "connected": 
				text.text = "Capteurs connectes";
			break;

			case "error": 
				text.text = "Erreur lors de la connexion : INTEGRATION";
				connexionButton.SetActive (true);
			break;
		}
	}
}
