using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using ShimmerAPI;
using System.IO;
using System;


class Data_analysis
{
	//listes de reference
	List<double>[] Ref;
	List<double> Ref_Axe_X;
	List<double> Ref_Axe_Y;
	List<double> Ref_Axe_Z;
	
	//listes du mouvement
	List<double>[] Cur;
	List<double> Cur_Axe_X;
	List<double> Cur_Axe_Y;
	List<double> Cur_Axe_Z;
	
	int Delai;
	
	
	//Constructeur ,  necessite les 6 listes originale
	public void Analysis(float DelaiMax, List<double>Ref_Axe_X,List<double>Ref_Axe_Y,List<double>Ref_Axe_Z,List<double>Cur_Axe_X,List<double>Cur_Axe_Y,List<double>Cur_Axe_Z)
	{
		Ref = new List<double>[3];
		Ref[0]=Ref_Axe_X;
		Ref[1]=Ref_Axe_Y;
		Ref[2]=Ref_Axe_Z;
		
		Cur = new List<double>[3];
		Cur[0]=Cur_Axe_X;
		Cur[1]=Cur_Axe_Y;
		Cur[2]=Cur_Axe_Z;
		
		Delai = (int)(DelaiMax*Math.Min (Cur [0].Count, Ref [0].Count));
	}
	
	//Verifie que les listes sont de même tailles et récupère le coeficient de corrélation
	public float getcoef()
	{
		bool erreur = false;
		float[] r = new float[] {0,0,0};
		if (Ref[0].Count == 0 || Cur[1].Count == 0)
		{
			erreur = true;
		}
		else
		{
			if (Ref[0].Count != Ref[1].Count || Ref[0].Count != Ref[2].Count || Ref[1].Count != Ref[2].Count)
			{
				erreur = true;
			}
			else
			{
				if (Cur[0].Count != Cur[1].Count || Cur[0].Count != Cur[2].Count || Cur[1].Count != Cur[2].Count)
				{
					erreur = true;
				}
				
			}
		}
		
		if (erreur == false)
		{

			r = Cross_Corelation();
			float Corelation =0 ;
			Corelation = (r[0] + r[1] + r[2]) / 3;
			return Corelation;
		}
		else
		{
			return -1;
		}
		
		
		
		
	}
	
	//Calcul la moyenne, le dénominateur et la corrélation, retourne la corrélation
	private float[] Cross_Corelation()
	{
		float[] Corelation = new float[] { 0, 0, 0 };
		float[] moyenne = new float[] { 0, 0, 0, 0, 0, 0};
		float[] denominateur = new float[] { 0, 0, 0,0,0,0 };
		int longueur = Math.Min (Cur [0].Count, Ref [0].Count);
		moyenne = getmoyenne();
		denominateur = getdenominator(moyenne);
		Corelation = getCore(moyenne, denominateur);
		return (Corelation);
	}
	
	//Calcul et retourne la moyenne
	private float[] getmoyenne()
	{
		float[] Moy = new float [] {0,0,0,0,0,0};
		for (int i = 0; i < 3; i++)
		{                
			for (int j = 0; j < Math.Min (Cur [i].Count, Ref [i].Count); j++)
			{
				Moy[i] = Moy[i] + (float)Cur[i][j];
			}
			for (int j = 0; j < Math.Min (Cur [i].Count, Ref [i].Count); j++)
			{
				Moy[i + 3] = Moy[i + 3] + (float)Ref[i][j];
			}
		}
		
		for (int i = 0; i < Moy.Length; i++)
		{
			Moy[i]=Moy[i]/Math.Min (Cur [0].Count, Ref [0].Count);
		}
		return (Moy);
	}
	
	//Calcul et retourne le dénominateur
    //Tableau contenant les moyennes récupérées 
    //Renvoit les dénominateur calculés
	private float[] getdenominator(float[] Moy)
	{
		float[] deno = new float[] { 0, 0, 0, 0, 0, 0};
		
		float[] Transmit = new float[] { 0, 0, 0 ,0,0,0};
		
		
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < Math.Min (Cur [i].Count, Ref [i].Count); j++)
			{
				float tmp = float.Parse((Cur[i][j] - (double)Moy[i]).ToString());
				float tmp2 = (float)Math.Pow(tmp,2);
				deno[i] = deno[i] + tmp2;
			}
			for (int j = 0; j < Math.Min (Cur [i].Count, Ref [i].Count); j++)
			{
				float tmp= float.Parse((Ref[i][j] - (double)Moy[i+3]).ToString());
				float tmp2 = (float)Math.Pow(tmp,2);
				deno[i+3] = deno[i+3] + tmp2;
			}
		}
		
		
		
		for (int i = 0; i < Transmit.Length; i++)
		{
			Debug.Log("déno "+deno[i]);
			Transmit[i] = (float)Math.Sqrt(deno[i]);
			Debug.Log(Transmit[i]);
		}
		
		return (Transmit);
	}
	
	//Calcul la corrélation, 
    //Necessite les moyennes et les dénominateur 
    //retourne la corrélation calculée
	private float[] getCore(float[] moyenne, float[] denom)
	{
		
		float[] Cor = new float[] {0,0,0};
		
	
			int j = 0;

				List <double>[] nom = new List<double>[3];
				for (int retard = -Delai; retard < Delai; retard++) {
			
						for (int i = 0; i < Ref[0].Count; i++) {
								j = i + retard;
								if (j >= 0 && j < Math.Min (Ref [0].Count, Cur [0].Count)) {
										for (int k =0; k<3; k++) {

												nom [k].Add (((float)Cur [k] [i] - moyenne [k]) * ((float)Ref [k] [j] - moyenne [k + 3]));

										}
								}
						}

				}
			for(int l =0; l<3;l++)
			{
				double value=nom[l][0];
				for(int m=0;m<nom[l].Count;m++)
				{
					if(value<nom[l][m])
						value=nom[l][m];
				}
				Cor[l]=(float)(value/(denom[l]*denom[l+3]));
			}


		
		return (Cor);
	}

}

