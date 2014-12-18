using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using System.Collections.Generic;
using ShimmerAPI;
using System.IO;
using System;

public class ConnexionManagerScript : MonoBehaviour {

	Text text;
	GameObject connexionButton;
	GameObject connexionCanvas;

	
	public List<double>[] Cur;
	public List<double>[] Ref;
	int count=0;
	Data_analysis Analyse;
	
	public Shimmer Shim = new Shimmer ("Shimmer","COM5"); //Initialise the sensor for the connection
	public bool connect;
	public bool stream;
	public string labelText="";
	public string status="";
	
	//Liste stockant les données
	List <double> Cur_X;
	List <double> Cur_Y;
	List <double> Cur_Z;
	
	Boolean read;
	float Coef;

	// Use this for initialization
	public void Start ()
	{
		// Get the GameObjects and the text component
		GameObject connexionText = GameObject.Find("Connexion Status");
			text = connexionText.GetComponent<Text>();
		connexionButton = GameObject.Find("Connexion Button");
		connexionCanvas = GameObject.Find ("GUI Connexion");

		// Hide retry button
		connexionButton.SetActive (false);

		// Launch connexion process
		connectSensors ();
	}


	// Manage the connection
	public void connectSensors()
	{

		updateStatus ("connecting");

		// Hide retry button
		connexionButton.SetActive (false);

		// INTEGRATION ====================================================================================
		//						 		LANCER CONNEXION
			labelText = "Start";
			Shim.UICallback += this.HandleEvent;
			connect = false;
			stream = false;
			read = false;
			this.Connect ();
			Cur = new List<double>[3];
			Ref = new List<double>[3];
			count = 0;
		//						
		//								if(CONNEXION FAILED)
		//									updateStatus ("error");
		//						
		//								if(CONNEXION SUCCESS)
		//									updateStatus ("connected");
		// ==================================================================================== INTEGRATION
	}

	public void Connect(){
		if (Shim.GetState () != Shimmer.SHIMMER_STATE_CONNECTED) {
			if (!connect) {
				Shim.StartConnectThread ();
				//Shim.Connect();
				labelText="connecting...";
				print ("connecting...");
				connect = true;
			}
		}
	}
	
	
	
	public void Disconnect(){
		if (Shim != null) {
			if (Shim.GetState () == (int)Shimmer.SHIMMER_STATE_STREAMING) {
				print ("stop stream");
				StopStreaming ();
			}
			print ("disconnecting");
			Shim.Disconnect ();
			//			if (Orientation3DForm != null)
			//			{
			//				Orientation3DForm.Close();
			//			}
		}
	}
	
	public void StartStreaming(){
		Shim.StartStreaming ();
	}
	
	public void StopStreaming(){
		Shim.StopStreaming ();
	}

	public void HandleEvent(object sender, EventArgs args){
		
		CustomEventArgs eventArgs = (CustomEventArgs)args;
		int indicator = eventArgs.getIndicator();
		//print ("HandleEvent "+indicator + "vs "+(int)Shimmer.ShimmerIdentifier.MSG_IDENTIFIER_DATA_PACKET);
		
		switch (indicator)
		{
		case (int)Shimmer.ShimmerIdentifier.MSG_IDENTIFIER_STATE_CHANGE:
			//print(((Shimmer)sender).GetDeviceName() + " State = " + ((Shimmer)sender).GetStateString() + System.Environment.NewLine);
			int state = (int)eventArgs.getObject();
			if(Shim.GetStateString().Contains("Connecting")){
				status="connecting";
			}
			if(Shim.GetStateString().Contains("Connected")){
				status="connected";
			}
			if(Shim.GetStateString().Contains("None")){
				status="error";
			}
			if(Shim.GetStateString().Contains("Streaming")){
				status="connected";
			}
			//labelText="State "+Shim.GetStateString();
			print ("State "+Shim.GetStateString());
			//GUI.Label(new Rect(0, 0, 100, 20), Shim.GetStateString());
			if (state == (int)Shimmer.SHIMMER_STATE_CONNECTED)
			{

				if(!stream){
					Shim.StartStreaming();

				}
			}
			else if (state == (int)Shimmer.SHIMMER_STATE_CONNECTING)
			{

			}
			else if (state == (int)Shimmer.SHIMMER_STATE_NONE)
			{

			}
			else if (state == (int)Shimmer.SHIMMER_STATE_STREAMING)
			{
			
			}
			break;
		case (int)Shimmer.ShimmerIdentifier.MSG_IDENTIFIER_NOTIFICATION_MESSAGE:
			string message = (string)eventArgs.getObject();
			if(message.Contains("Unable"))

				status="error";
			labelText= message;
			print(message);
			break;
		case (int)Shimmer.ShimmerIdentifier.MSG_IDENTIFIER_DATA_PACKET:
			
			if(Shim.GetEnabledSensors() >0){
				// this is essential to ensure the object is not a reference
				ObjectCluster objectCluster = new ObjectCluster((ObjectCluster)eventArgs.getObject());
				List<String> names = objectCluster.GetNames();
				List<String> formats = objectCluster.GetFormats();
				List<String> units = objectCluster.GetUnits();
				List<Double> data = objectCluster.GetData();
				labelText=objectCluster.GetData ("Low Noise Accelerometer X", "CAL").GetData ().ToString ();
				if(read==true)
				{
					Cur_X.Add(double.Parse((objectCluster.GetData ("Low Noise Accelerometer X", "CAL")).GetData ().ToString ()));
					Cur_Y.Add(double.Parse((objectCluster.GetData ("Low Noise Accelerometer Y", "CAL")).GetData ().ToString ()));
					Cur_Z.Add(double.Parse((objectCluster.GetData ("Low Noise Accelerometer Z", "CAL")).GetData ().ToString ()));
				}//print ("value "+(objectCluster.GetData ("Gyroscope X", "RAW")).GetData ().ToString ());
				//GUI.Label(new Rect(0, 0, 100, 20), ""+(objectCluster.GetData ("Gyroscope X", "RAW")).GetData ().ToString ());
			}
						else if (status.Equals("connected")&& Shim.GetEnabledSensors() ==0){
						status="error";
					}
			break;
		case (int)Shimmer.ShimmerIdentifier.MSG_IDENTIFIER_PACKET_RECEPTION_RATE:
			/*double prr = (double)eventArgs.getObject();
						print (""+prr);*/
			break;
			
		}
	}

	public void GetList(int i)
	{
		int j = i;
		if ( j >= 0 && j < 2) {
			if (j == 0)
			{
				Cur[0]=Cur_X;
				Cur[1]=Cur_Y;
				Cur[2]=Cur_Z;
			}
			else {
				if (j == 1)
				{
					Ref[0]=Cur_X;
					Ref[1]=Cur_Y;
					Ref[2]=Cur_Z;
				}
				
			}
			
		}
	}

	// Update is called once per frame
	void Update () {

		updateStatus (status);
		if (Input.GetKeyDown("p")){
			Disconnect ();
		}
		if (Input.GetKeyDown("a") && GameObject.Find("Zone detection").GetComponent<OpenDoor>().getisInside()){
			startAcquisition();			
		}
		if (Input.GetKeyDown("z")){
			stopAcquisition();
		}
	}
	// Update the text and the retry button according to the status. Hide the connexion canvas if connected
	private void updateStatus (string status) {
		switch (status) 

		{
			case "connecting": 
				text.text = "Connexion des capteurs en cours ...";
			break;

			case "connected": 
				text.text = "Capteurs connectes";
			break;

			case "error": 
				text.text = "Erreur lors de la connexion : Unable to connect";
				Disconnect();
				connexionButton.SetActive (true);
			break;
		}
	}

	public void hideConnexionCanvas()
	{
		connexionCanvas.SetActive (false);
	}

	public void startAcquisition(){

		Cur_X=new List<double>();
		Cur_Y=new List<double>();
		Cur_Z=new List<double>();
		read=true;
	}

	public void stopAcquisition(){
		
		read=false;
		this.GetList(count);
		count++;
		
		if(count==2)
		{
			//				foreach(double red in Ref[0])
			//				{
			//					printConsole(red.ToString());
			//				}
			float delai=0.1f;
			
			Analyse = new Data_analysis();
			this.Analyse.Analysis(delai,Ref[0],Ref[1],Ref[2],Cur[0],Cur[1],Cur[2]);
			if (this.Analyse.getcoef() == -1){
				Disconnect();
			}
			else
				Coef = this.Analyse.getcoef();

			print(Coef.ToString());
			count=0;
		}
		GameObject.Find("Zone detection").GetComponent<OpenDoor>().setIsCompleted(true);
	}
	
	public float getCoef(){
		return Coef;
		}
}
