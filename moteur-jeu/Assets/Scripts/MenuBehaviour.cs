using UnityEngine;
using System.Collections;
using UnityEngine.UI;
using UnityEngine.EventSystems;

public class MenuBehaviour : MonoBehaviour {

	private Text text;
	private Color basicColor;
	public Color overColor;

	private int basicSize;
	public float ratioSize = 0.5f;


	// Use this for initialization
	void Awake () {
		text = GetComponent<Text>();
	
		basicColor = text.color;
		basicSize = text.fontSize;
		print ("Color = " + basicColor);
		print ("Size = " + basicSize);
	}

	void OnMouseEnter () {
		print ("Coucou");
		text.color = overColor;
		text.fontSize = (int)(basicSize * ratioSize + basicSize);
	}

	void OnMouseExit () {
		text.color = basicColor;
		text.fontSize = basicSize;
	}
}
