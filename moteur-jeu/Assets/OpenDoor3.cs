using UnityEngine;
using System.Collections;
 
public class OpenDoor3 : MonoBehaviour {
	
	[SerializeField]
	public float moveSpeedPorte; // INTEGRATION : fixer au max en constante
	public float moveSpeedLevier; //
	public Rigidbody _porte;
	public Rigidbody _levier;
	public float seuil = 0.7f; // 
	
	private Vector3 position_initiale_porte;
	private Vector3 position_initiale_levier;
	private float ouverture = 0f; 
	
	/*
		Use this for initialization
	*/
	void Start () 
	{
		position_initiale_porte = _porte.transform.position;
		position_initiale_levier = _levier.transform.position; 
	}
	
	
	/*
		UnTriggerStay is called at every frame while the player is in the trigger
	*/
	public void OnTriggerStay (Collider col) 
	{
		acquireMovement();

		// Deplacer la porte selon ouverture
		Vector3 position_finale_porte = new Vector3(position_initiale_porte.x, 
		                                             position_initiale_porte.y + (ouverture * _porte.renderer.bounds.size.y), 
		                                             position_initiale_porte.z);

		_porte.transform.position = Vector3.Lerp(_porte.transform.position, 
		                                          position_finale_porte, 
		                                          Time.deltaTime * moveSpeedPorte);
	}

	private void acquireMovement()
	{
		ouverture = 0.5f; // INTEGRATION ====================================================================================

		/*
		 * OUVRIR FENETRE DIALOGUE POUR INPUT 
		 * _levier.transform.position = Vector3.Lerp (_levier.transform.position, 
                  								position_intiale_levier, 
                  								Time.deltaTime * moveSpeedLevier * 5);	
		 */

		Vector3 position_finale_levier = new Vector3 (position_initiale_levier.x + _levier.renderer.bounds.size.x / 2, 
                  			 						  position_initiale_levier.y, 
                    								  position_initiale_levier.z);


		_levier.transform.position = Vector3.Lerp (_levier.transform.position, 
                  								position_finale_levier, 
                  								Time.deltaTime * moveSpeedLevier);				
	}
}
