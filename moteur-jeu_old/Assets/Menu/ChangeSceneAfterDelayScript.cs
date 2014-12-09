using UnityEngine;
using UnityEngine.UI;
using System.Collections;

public class ChangeSceneAfterDelayScript : MonoBehaviour {

	public string _nextScene = "";

	public float _delay = 5f;

	//public IEnumerator Start()
	public IEnumerator Start()
	{
		yield return new WaitForSeconds (_delay); 

		Application.LoadLevel (_nextScene);
	}
}
