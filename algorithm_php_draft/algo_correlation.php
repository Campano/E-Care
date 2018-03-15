<?php
	function getCoef($mouvement_1, $mouvement_2, $delaiMaxPourcent = 0.25){
		$erreur = NULL;
		$r = 0;
		$nb_axes = 3;
		
		// À ADAPTER : VÉRIFS SUR TABLEAUX
		if(is_array($mouvement_1) && is_array($mouvement_2) && count($mouvement_1)==$nb_axes && count($mouvement_2)==$nb_axes){
			$axes = array_keys($mouvement_1);
			foreach($axes as $axe)
				if(!array_key_exists($axe, $mouvement_1))
					$erreur = 'Tableaux différents';
		} else {
			$axes = NULL;
			$erreur = 'Problème sur les paramètres';
		}
		
		if($erreur)
			die($erreur);
			
		
		// Moyenne du coef de corrélation sur les trois axes
		foreach($axes as $axe)
			$r += cross_correlation($mouvement_1[$axe], $mouvement_2[$axe], $delaiMaxPourcent);
		
		$r /= $nb_axes;
	
		return round($r, 2);
	}
	
	function cross_correlation($s1, $s2, $delaiMaxPourcent){
		// http://paulbourke.net/miscellaneous/correlate/
		
		//Réindexation
		$s1 = array_values($s1);
		$s2 = array_values($s2);
		
		//On détermine d'abord la taille de l'échantillon
		$n = getTaille($s1, $s2);
		$delaiMax = ceil($n * $delaiMaxPourcent);
		
		
		
		$m1 = getMoyenne($s1, $n);
		$m2 = getMoyenne($s2, $n);
		
		$dn1 = getDenominateur($s1, $m1, $n);
		$dn2 = getDenominateur($s2, $m2, $n);
		
		$r = array();
		
		for($delai = -$delaiMax ; $delai < $delaiMax ; $delai++){
			$nominateur = 0;
			
			for($i=0; $i<$n; $i++){
				$j = $i + $delai;
				if($j>=0 && $j<$n)
					$nominateur += ($s1[$i] - $m1) * ($s2[$j] - $m2);
			}
			
			$r[$delai] = $nominateur / ($dn1 * $dn2);
		}
		
		return round(max($r),2);
	}
	
	function getTaille($s1, $s2){
		// ARBITRAIRE : on choisit la série la plus courte
		$n1 = sizeof($s1);
		$n2 = sizeof($s2);
		return $n1 < $n2 ? $n1 : $n2;
	}
	
	function getMoyenne($s, $n){
		$m=0;
		
		for($i=0; $i<$n; $i++)
				$m += $s[$i];
		return $m / $n;
	}
	
	function getDenominateur($s, $m, $n){
		$dn=0;
		for($i=0; $i<$n; $i++)
			$dn += pow( $s[$i] - $m , 2);
		return sqrt($dn);
	}
?>