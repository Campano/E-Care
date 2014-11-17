using UnityEngine;
using System.Collections;

public class OpenDoor2 : MonoBehaviour {
	
	[SerializeField]
	public float moveSpeed;
	public Rigidbody _porte;
	public float ouverture; 
	
	private float position_initiale;
	
	/*
		Use this for initialization
	*/
	void Start () 
	{
		if(_porte != null)
			position_initiale = _porte.transform.position.y;
	}
	
	
	/*
		UnTriggerStay is called at every frame while the player is in the trigger
	*/
	public void OnTriggerStay (Collider col) 
	{
		if(_porte != null)
		{
			if(Input.GetKey(KeyCode.P)) // INTEGRATION : récupérer le % d'ouverture et le stocker dans "long ouverture" On peut afficher un popup : "start movmnt now" pour freezer le jeu et acquerir les cpateurs
			{
				while(_porte.transform.position.y < position_initiale + (ouverture * _porte.renderer.bounds.size.y)) // tant qu'on n'a pas atteint la position finale
					_porte.transform.Translate(new Vector3(0, 1, 0) * moveSpeed * Time.deltaTime); // déplacer la porte vers le haut
			}
		}
	}
}
