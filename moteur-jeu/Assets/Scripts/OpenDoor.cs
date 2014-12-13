using UnityEngine;
using UnityEngine.UI;
using System.Collections;
 
public class OpenDoor : MonoBehaviour {
	
	[SerializeField]
	public float moveSpeedDoor = 2; 
	public float moveSpeedLever = 4; 
	public Rigidbody _door;
	public Rigidbody _lever;
	
	private Vector3 initialPositionDoor;
	private Vector3 initialPositionLever;
	private GameObject _GUI;
	private GameObject _Counter;
	private float ouverture; 
	private Text textCounter;
	float startTime;
	
/*
*	Use this for initialization
*/
	void Start () 
	{
		initialPositionDoor = _door.transform.position;
		initialPositionLever = _lever.transform.position; 

		_GUI = GameObject.Find("GUI Acquisition");
		_Counter = GameObject.Find("Counter");
		textCounter = _Counter.GetComponent <Text> ();

		_GUI.SetActive(false);

	}
	
	
/*
*	UnTriggerStay is called at every frame while the player is in the trigger
*/
	public void OnTriggerStay (Collider col) 
	{
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

			// Determination of the lever final position
			Vector3 finalPositionLever = new Vector3 (initialPositionLever.x + _lever.renderer.bounds.size.x / 2, 
                                  initialPositionLever.y, 
                                  initialPositionLever.z);

			// Move the lever 
			_lever.transform.position = Vector3.Lerp (_lever.transform.position, 
                                  finalPositionLever, 
                                  Time.deltaTime * moveSpeedLever);
		}
	}


/*
*	Manage the acquisition of the movement's datas
*/
	private float acquireMovement()
	{
		// Show acquirement GUI
		_GUI.SetActive(true);
		countDown ();
		
		// INTEGRATION ====================================================================================
		return ouverture = 0.5f; 
		// ==================================================================================== INTEGRATION
	}


/*
*	UnTriggerExit is called once when the player leave the trigger
*/
	public void OnTriggerExit (Collider col)
	{
		_GUI.SetActive(false);
	}


/*
*	Manage the countdown in the acquisition GUI
*/
	private IEnumerator countDown()
	{
		for(int counter = 3; counter >=0; counter--)
		{	
			print (counter+ "  "+Time.time);
			yield return StartCoroutine(wait(0.5f));
			textCounter.text = counter.ToString();
		}
	}

	private IEnumerator wait(float waitTime)
	{
		float endTime = Time.realtimeSinceStartup + waitTime;
		
		while (Time.realtimeSinceStartup < endTime) 
			yield return null;
	}
}
