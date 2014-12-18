using UnityEngine;
using UnityEngine.UI;
using System.Collections;
 
public class OpenDoor : MonoBehaviour {
	
	[SerializeField]
	public float moveSpeedDoor = 2; 
	public Rigidbody _door;
	
	private Vector3 initialPositionDoor;
	private GameObject _GUI;
	private float ouverture; 

	bool isInside;
	bool isCompleted;
	
/*
*	Use this for initialization
*/
	void Start () 
	{
		isInside = false;
		isCompleted = false;
		initialPositionDoor = _door.transform.position;

		// Get and hide the acquisition UI
		_GUI = GameObject.Find ("GUI Acquisition");
		_GUI.SetActive(false);

	}
	
	
/*
*	UnTriggerStay is called at every frame while the player is in the trigger
*/
	public void OnTriggerStay (Collider col) 
	{
		print ("is Inside = " + isInside);
		isInside = true;
		print ("is Inside = " + isInside);
		// Move door and lever if movement was succesfully acquired
		if (acquireMovement() != 0) 
		{
			// Determination of the door final position depending on ouverture
			Vector3 finalPositionDoor = new Vector3 (initialPositionDoor.x, 
                                    initialPositionDoor.y + (ouverture * _door.renderer.bounds.size.y), 
                                    initialPositionDoor.z);

			// Move the door depending on ouverture
			_door.transform.position = Vector3.Lerp (_door.transform.position, 
                                finalPositionDoor, 
                                  Time.deltaTime * moveSpeedDoor);
		}
	}


/*
*	Manage the acquisition of the movement's datas
*/
	private float acquireMovement()
	{
		// Show acquirement GUI
		_GUI.SetActive(true);

		// GameObject that will get the difficulty set in the setting's panel
		GameObject UIgetter = GameObject.Find ("UI Manager");

		//Calcule le coef de corrélation entre deux mouvements
		float analysisResult =GameObject.Find("Connexion Manager").GetComponent<ConnexionManagerScript>().getCoef();
		print (analysisResult);
			
		// Serialize difficulty so difficulty<50 => serializedDifficulty<1 & difficulty>50 => serializedDifficulty>1
		float serializedDifficulty = UIgetter.GetComponent<UIManager>().getDifficulty() / 50;

		return ouverture = analysisResult * serializedDifficulty; 

	}


/*
*	UnTriggerExit is called once when the player leave the trigger
*/
	public void OnTriggerExit (Collider col)
	{	isInside = false;
		_GUI.SetActive(false);
	}


	public bool getisInside(){
		return isInside;
	}

	public void setIsCompleted (bool value){
		isCompleted = value;
	}
}


