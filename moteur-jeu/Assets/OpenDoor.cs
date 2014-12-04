using UnityEngine;
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
	
	/*
		Use this for initialization
	*/
	void Start () 
	{
		initialPositionDoor = _door.transform.position;
		initialPositionLever = _lever.transform.position; 

		_GUI = GameObject.Find("GUI Acquisition");
		_Counter = GameObject.Find("Counter");

		_GUI.SetActive(false);

	}
	
	
	/*
		UnTriggerStay is called at every frame while the player is in the trigger
	*/
	public void OnTriggerStay (Collider col) 
	{
		acquireMovement();

		// Determination of the final position depending on ouverture
		Vector3 finalPositionDoor = new Vector3(initialPositionDoor.x, 
		                                            initialPositionDoor.y + (ouverture * _door.renderer.bounds.size.y), 
		                                            initialPositionDoor.z);

		// Move the door depending on ouverture
		_door.transform.position = Vector3.Lerp(_door.transform.position, 
		                                        finalPositionDoor, 
		                                          Time.deltaTime * moveSpeedDoor);
	}

	public void OnTriggerExit (Collider col)
	{
		_GUI.SetActive(false);
	}

	/*
		Manage the acquisition of the movement's datas
	*/
	private void acquireMovement()
	{
		_GUI.SetActive(true);
		countDown ();

		// INTEGRATION ====================================================================================
		ouverture = 0.5f; 
		// ==================================================================================== INTEGRATION

		Vector3 finalPositionLever = new Vector3 (initialPositionLever.x + _lever.renderer.bounds.size.x / 2, 
		                                              initialPositionLever.y, 
		                                              initialPositionLever.z);


		_lever.transform.position = Vector3.Lerp (_lever.transform.position, 
		                                          finalPositionLever, 
                  								Time.deltaTime * moveSpeedLever);				
	}

	/*
		Manage the countdown in the acquisition GUI
	*/
	private void countDown()
	{
		float startTime = Time.time;
		float counter = Time.time - startTime;
		_Counter.text = "bla";


	}
}
