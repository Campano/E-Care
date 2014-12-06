using UnityEngine;
using UnityEngine.UI;
using System.Collections;

public class ChangeSceneAfterDelayScript : MonoBehaviour {

	public string _nextScene = "";
	public float fadeSpeed = 1.5f;

	public float _delay = 5f;

	public IEnumerator Start()
	{


		GameObject panelObject = GameObject.Find("Fading");
		Image panel = panelObject.GetComponent <Image> ();

		panel.CrossFadeAlpha (255, fadeSpeed * Time.deltaTime, false);

		yield return new WaitForSeconds (_delay); 

		Application.LoadLevel (_nextScene);
	}
}
