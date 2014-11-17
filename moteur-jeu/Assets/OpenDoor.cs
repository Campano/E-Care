using UnityEngine;
using System.Collections;

public class OpenDoor : MonoBehaviour {
	
	[SerializeField]
	public float moveSpeed;
	public Rigidbody _porte;
	public float ouverture; 

	private float position_initiale;
	private char orientation;
	
	/*
		Use this for initialization
	*/
	void Start () 
	{
		if(_porte != null)
		{
			if(_porte.renderer.bounds.size.x > _porte.renderer.bounds.size.z) // Seulement les cas de portes coulissants sur l'axe des x ou z est pris en compte
			{
				orientation = 'x';
				position_initiale = _porte.transform.position.x;
			}
			else 
			{
				orientation = 'z';
				position_initiale = _porte.transform.position.z;
			}
		}
    }


	/*
		UnTriggerStay is called at every frame while the player is in the trigger
	*/
	public void OnTriggerStay (Collider col) 
	{
		 if(_porte != null)
		 {
			if(Input.GetKey(KeyCode.P)) // INTEGRATION : récupérer le % d'ouverture et le stocker dans "long ouverture"
			{
				Debug.Log("getkey ");
				if(orientation == 'x')
				{
					Debug.Log("DANS IF :"+orientation);
					while(_porte.transform.position.x < position_initiale + (ouverture * _porte.renderer.bounds.size.x)) // tant qu'on n'a pas atteint la position finale
					{	
						Debug.Log("DANS WHILE :"+orientation);
						_porte.transform.Translate(new Vector3(1, 0, 0) * moveSpeed * Time.deltaTime);
					}
					Debug.Log("APRES WHILE :"+orientation);
				}

				else if(orientation == 'z')
				{
					Debug.Log("DANS IF :"+orientation);
					while(_porte.transform.position.z < position_initiale + (ouverture * _porte.renderer.bounds.size.z)) // tant qu'on n'a pas atteint la position finale
					{	
						Debug.Log("DANS WHILE :"+orientation);
						_porte.transform.Translate(new Vector3(0, 0, 1) * moveSpeed * Time.deltaTime);
					}
				}

				else 
					Debug.LogError("Erreur d'orientation de la porte : doit etre orienté selon x OU z");
			}
		 }
	}
}
