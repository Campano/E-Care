using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using ShimmerAPI;
using System.IO;
using System;

public class ReadData : MonoBehaviour {
	
	public Shimmer Shim = new Shimmer ("Shimmer","COM4"); //Initialise the sensor for the connection
	public bool connect;
	public bool stream;
	public string labelText="";

	//Liste stockant les données
	List <double> Cur_X;
	List <double> Cur_Y;
	List <double> Cur_Z;
	
	// Use this for initialization
	void Start () {
		labelText = "Start";
		Shim.UICallback += this.HandleEvent;
		connect = false;
		stream = false;
		this.Connect ();
		
	}
	
	// Update is called once per frame
	void Update () {
		
		if (Input.GetKeyDown("p")){
			//print ("pause");
			Disconnect ();
		}
	}
	
	void OnGUI () 
	{
		float x = 310;
		float y = 110;
		GUI.color = Color.white;
		GUI.skin.box.fontSize = 32;
		GUI.Box(new Rect(x, y, 230, 50),""+labelText );
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
				StopStreaming ();
			}
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
			labelText="State "+Shim.GetStateString();
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
				Cur_X.Add(double.Parse((objectCluster.GetData ("Low Noise Accelerometer X", "CAL")).GetData ().ToString ()));
				Cur_Y.Add(double.Parse((objectCluster.GetData ("Low Noise Accelerometer Y", "CAL")).GetData ().ToString ()));
				Cur_Z.Add(double.Parse((objectCluster.GetData ("Low Noise Accelerometer Z", "CAL")).GetData ().ToString ()));
				//print ("value "+(objectCluster.GetData ("Gyroscope X", "RAW")).GetData ().ToString ());
				//GUI.Label(new Rect(0, 0, 100, 20), ""+(objectCluster.GetData ("Gyroscope X", "RAW")).GetData ().ToString ());
			}
			break;
		case (int)Shimmer.ShimmerIdentifier.MSG_IDENTIFIER_PACKET_RECEPTION_RATE:
			/*double prr = (double)eventArgs.getObject();
						print (""+prr);*/
			break;
			
		}
	}
	
	public void printConsole(String toBePrinted){
		print (toBePrinted);
	}
}
