using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using System.Collections.Generic;
using ShimmerAPI; //API à intégrer dans le projet
using System.IO;
using System;

public class ConnexionManagerScript : MonoBehaviour {

	//Variables de classe
	Text text;
	GameObject connexionButton;
	GameObject connexionCanvas;

	//Listes regroupant les données du mouvement
	public List<double>[] Cur; //mouvement du patient
	public List<double>[] Ref; //mouvement de référence
	int count=0; //représente qui fait le mouvement: 1=Kiné 2=Patient
	Data_analysis Analyse; 

	//Initialisation du shimmer: Shimmer(nom,PortCOM);
	public Shimmer Shim = new Shimmer ("Shimmer","COM9");
	//isConnected ?
	public bool connect;
	//isStreaming?
	public bool stream;
	//variable qui enregistre les messages envoyés par le capteur
	public string labelText="";
	//Statut du capteur
	public string status="";
	
	//Liste stockant les données
	List <double> Cur_X; //mouvement sur X
	List <double> Cur_Y; //mouvement sur Y
	List <double> Cur_Z; //mouvement sur Z

	//Are we reading the data ?
	Boolean read;
	//Coefficient de corrélation, pourcentage de réussite du mouvement 0<coef<1
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

		//						 		LANCER CONNEXION
			//Début de la connexion
			labelText = "Start";
			//Attribution du HandleEvent au Capteur Shimmer
			Shim.UICallback += this.HandleEvent;
			//not connected
			connect = false;
			//not streaming
			stream = false;
			//not reading
			read = false;
			//===>Connexion<===
			this.Connect ();

			//Initialisation des listes de mouvements
			Cur = new List<double>[3];
			Ref = new List<double>[3];
			//Le premier mouvement à effectuer est celui de référence
			count = 0;
	}

	//Méthode de connexion au capteur
	public void Connect(){
		//On vérifie que le capteur n'est pas connecté
		if (Shim.GetState () != Shimmer.SHIMMER_STATE_CONNECTED) {
			if (!connect) {
				//Lancement de la connexion
				Shim.StartConnectThread ();
				//Affichage de l'état
				labelText="connecting...";
				print ("connecting...");
				//Shimmer connecté
				connect = true;
			}
		}
	}
	
	
	//Méthode pour se déconnecter du Shimmer
	public void Disconnect(){
		//S'il existé un capteur
		if (Shim != null) {
			//Si se catpeur envoie des données
			if (Shim.GetState () == (int)Shimmer.SHIMMER_STATE_STREAMING) {
				print ("stop stream");
				//On stoppe la communication
				StopStreaming ();
			}
			print ("disconnecting");
			//On se déconnecte
			Shim.Disconnect ();
		}
	}

	//Début de l'envoi des données
	public void StartStreaming(){
		Shim.StartStreaming ();
	}

	//Arret de l'envoi des données
	public void StopStreaming(){
		Shim.StopStreaming ();
	}

	/* Fonction qui permet de lire automatiquement les messages envoyés depuis le capteur
	 * Object Sender: Shimmer
	 * EventArgs: les evennements envoyés par le capteur
	 * 
	 * Cette méthode est appelée automatiquement lorsqu'un capteur envoie des données
	 */
	public void HandleEvent(object sender, EventArgs args){

		//Réception de l'évennement
		CustomEventArgs eventArgs = (CustomEventArgs)args;
		int indicator = eventArgs.getIndicator();

		//Selon l'évennement reçu
		switch (indicator)
		{
			//Evennement de changement d'état
		case (int)Shimmer.ShimmerIdentifier.MSG_IDENTIFIER_STATE_CHANGE:
			int state = (int)eventArgs.getObject();

			//Réception de l'état du Shimmer: 4 valeurs possibles
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
			print ("State "+Shim.GetStateString());
			//Si le capteur est connecté
			if (state == (int)Shimmer.SHIMMER_STATE_CONNECTED)
			{
				//Si le capteur n'envoie pas de données
				if(!stream){
					//On demande au capteur d'envoyer ses données
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

			//Evennement de notification --> souvent des messages d'erreur
		case (int)Shimmer.ShimmerIdentifier.MSG_IDENTIFIER_NOTIFICATION_MESSAGE:
			//On récupère le message
			string message = (string)eventArgs.getObject();
			//Si c'est une erreur de type "unable to connect"
			if(message.Contains("Unable"))
				//Renvoi de l'erreur
				status="error";
				labelText= message;
				print(message);
				break;

			//Evennement de réception de données: le capteur nous envoie ses données
		case (int)Shimmer.ShimmerIdentifier.MSG_IDENTIFIER_DATA_PACKET:

			//Si les capteurs sont actifs
			if(Shim.GetEnabledSensors() >0){
				// this is essential to ensure the object is not a reference
				ObjectCluster objectCluster = new ObjectCluster((ObjectCluster)eventArgs.getObject());
				List<String> names = objectCluster.GetNames();
				List<String> formats = objectCluster.GetFormats();
				List<String> units = objectCluster.GetUnits();
				List<Double> data = objectCluster.GetData();
				labelText=objectCluster.GetData ("Low Noise Accelerometer X", "CAL").GetData ().ToString ();

				//Si on souhaite lire la données
				if(read==true)
				{
					//lecture des données
					Cur_X.Add(double.Parse((objectCluster.GetData ("Low Noise Accelerometer X", "CAL")).GetData ().ToString ()));
					Cur_Y.Add(double.Parse((objectCluster.GetData ("Low Noise Accelerometer Y", "CAL")).GetData ().ToString ()));
					Cur_Z.Add(double.Parse((objectCluster.GetData ("Low Noise Accelerometer Z", "CAL")).GetData ().ToString ()));
				}
			}
						//Si aucun capteur n'est actif
						else if (status.Equals("connected")&& Shim.GetEnabledSensors() ==0){
						//Envoie erreur
						status="error";
					}
			break;
			//Evennement de notification du taux de reception
		case (int)Shimmer.ShimmerIdentifier.MSG_IDENTIFIER_PACKET_RECEPTION_RATE:

			break;
			
		}
	}

	//Retourne la liste de mouvements demandée
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

		//Mise à jour du statut
		updateStatus (status);

		//Déconnecte manuellement le capteur
		if (Input.GetKeyDown("p")){
			Disconnect ();
		}

		//Lance l'acquisitionn
		if (Input.GetKeyDown("a") && GameObject.Find("Zone detection").GetComponent<OpenDoor>().getisInside()){
			startAcquisition();			
		}

		//Arrete l'acquisition
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

			//Dans le cas d'une erreur
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

	//Initialise les variables utilisées pendant l'acquisition
	public void startAcquisition(){
		Cur_X=new List<double>();
		Cur_Y=new List<double>();
		Cur_Z=new List<double>();
		read=true;
	}

	//Arrete l'acquisition
	public void stopAcquisition(){

		//Le capteur ne lit plus
		read=false;
		this.GetList(count);
		count++;
		
		if(count==2)
		{
			//On s'autorise un délai
			float delai=0.1f;

			//Lancement de l'algorithme qui compare les mouvements
			Analyse = new Data_analysis();
			this.Analyse.Analysis(delai,Ref[0],Ref[1],Ref[2],Cur[0],Cur[1],Cur[2]);

			//Si l'algorithme renvoie -1; nous avons une erreur de connexion
			if (this.Analyse.getcoef() == -1){
				Disconnect();
			}
			//Sinon, renvoi du coef
			else
				Coef = this.Analyse.getcoef();

			print(Coef.ToString());
			//remise à zéro
			count=0;
		}
		//Le mouvement est fini, la porte peut bouger
		GameObject.Find("Zone detection").GetComponent<OpenDoor>().setIsCompleted(true);
	}
	
	public float getCoef(){
		return Coef;
		}
}
