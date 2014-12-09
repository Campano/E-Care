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
	public void Analysis(int DelaiMax, List<double>Ref_Axe_X,List<double>Ref_Axe_Y,List<double>Ref_Axe_Z,List<double>Cur_Axe_X,List<double>Cur_Axe_Y,List<double>Cur_Axe_Z)
	{
		Ref = new List<double>[3];
		Ref[0]=Ref_Axe_X;
		Ref[1]=Ref_Axe_Y;
		Ref[2]=Ref_Axe_Z;
		
		Cur = new List<double>[3];
		Cur[0]=Cur_Axe_X;
		Cur[1]=Cur_Axe_Y;
		Cur[2]=Cur_Axe_Z;
		
		Delai = DelaiMax;
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
			double Corelation = (r[0] + r[1] + r[2]) / 3;
			return ((r[0] + r[1] + r[2]) / 3);
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
		float[] denominateur = new float[] { 0, 0, 0 };
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
			for (int j = 0; j < Cur[i].Count; j++)
			{
				Moy[i] = Moy[i] + (float)Cur[i][j];
			}
			for (int j = 0; j < Ref[i].Count; j++)
			{
				Moy[i + 3] = Moy[i + 3] + (float)Ref[i][j];
			}
		}
		
		for (int i = 0; i < Moy.Length; i++)
		{
			if (i < 3)
			{
				Moy[i] = Moy[i] / Cur[0].Count;
			}
			else
			{
				Moy[i] = Moy[i] / Ref[0].Count;
			}
		}
		return (Moy);
	}
	
	//Calcul et retourne le dénominateur
    //Tableau contenant les moyennes récupérées 
    //Renvoit les dénominateur calculés
	private float[] getdenominator(float[] Moy)
	{
		float[] deno = new float[] { 0, 0, 0, 0, 0, 0};
		
		float[] Transmit = new float[] { 0, 0, 0 };
		
		
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < Cur[i].Count; j++)
			{
				deno[i] = deno[i] + (float)Math.Sqrt(((Cur[i][j] - (double)Moy[i])));
			}
			for (int j = 0; j < Ref[i].Count; j++)
			{
				deno[i+3] = deno[i+3] + (float)Math.Sqrt(((Ref[i][j] - (double)Moy[i+3])));
			}
		}
		
		
		
		for (int i = 0; i < Transmit.Length; i++)
		{
			Transmit[i] = (float)Math.Sqrt(deno[i] * deno[i + 3]);
		}
		
		return (Transmit);
	}
	
	//Calcul la corrélation, 
    //Necessite les moyennes et les dénominateur 
    //retourne la corrélation calculée
	private float[] getCore(float[] moyenne, float[] denom)
	{
		
		float[] Cor = new float[] {0,0,0};
		
		for(int k=0; k<3; k++)
		{
			int j = 0;
            if (Delai > 0)
            {
                for (int retard = -Delai; retard < Delai; retard++)
                {
                    for (int i = 0; i < Ref[0].Count; i++)
                    {
                        j = i + retard;
                        if (j >= 0 && j < Ref[0].Count)
                        {
                            if (k == 0)
                            {
                                Cor[k] = Cor[k] + ((float)Cur[0][i] - moyenne[k]) * ((float)Ref[0][j] - moyenne[k + 3]);
                            }
                            if (k == 1)
                            {
                                Cor[k] = Cor[k] + ((float)Cur[1][i] - moyenne[k]) * ((float)Ref_Axe_Y[j] - moyenne[k + 3]);
                            }
                            if (k == 2)
                            {
                                Cor[k] = Cor[k] + ((float)Cur[2][i] - moyenne[k]) * ((float)Ref[2][j] - moyenne[k + 3]);
                            }
                        }
                    }
                }
            }
            else
            {
                for (int i = 0; i < Math.Min(Ref[0].Count,Cur[0].Count); i++)
                {
					if (j >= 0 && j < Math.Min(Ref[0].Count,Cur[0].Count))
                    {
                        if (k == 0)
                        {
                            Cor[k] = Cor[k] + ((float)Cur[0][i] - moyenne[k]) * ((float)Ref[0][j] - moyenne[k + 3]);
                        }
                        if (k == 1)
                        {
                            Cor[k] = Cor[k] + ((float)Cur[1][i] - moyenne[k]) * ((float)Ref[1][j] - moyenne[k + 3]);
                        }
                        if (k == 2)
                        {
                            Cor[k] = Cor[k] + ((float)Cur[2][i] - moyenne[k]) * ((float)Ref[2][j] - moyenne[k + 3]);
                        }
                    }
                }
            }
		}
		
		for (int i = 0; i < 3; i++)
		{
			Debug.Log (Cor[i]+"--"+denom[i]);
			Cor[i] = Cor[i] / denom[i];
		}
		
		return (Cor);
	}

}

