using UnityEngine;
using System.Collections;

public class Menu : MonoBehaviour {

	private Color basicColor;
	public Color overColor;

	private float basicSize;
	public float ratioSize = 0.05f;


	// Use this for initialization
	void Start () {
		basicColor = guiText.material.color;
		basicSize = guiText.fontSize;
	}
	
	// Update is called once per frame
	void OnMouseEnter () {
		guiText.material.color = overColor;
		guiText.fontSize = basicSize * ratioSize + basicSize;
	}

	void OnMouseExit () {
		guiText.material.color = basicColor;
		guiText.fontSize = basicSize;
	}
}
