<?php
	include('algo_correlation.php');
	
	function pre($var){
		print("<pre>".print_r($var,true)."</pre>");
	}
	
	function csv_to_array($filename='', $delimiter=',')
	{
	    if(!file_exists($filename) || !is_readable($filename))
	        return FALSE;
	
	    $header = array();
	    $data = array();
	    $i=0;
	    
	    
	    if (($handle = fopen($filename, 'r')) !== FALSE)
	    {
	        while (($row = fgetcsv($handle, 0, $delimiter)) !== FALSE)
	        {	
	        	if($i < 4)
		        	foreach($row as $key => $value){
						if(!array_key_exists($key, $header))
		        			$header[$key] = $value;
		        		else
		        			$header[$key] .= ' - '.$value;
			        }        
	        	else 
		        	foreach($row as $key => $value){
		        		if(!array_key_exists($key, $data))
		        			$data[$key] = array();
			        	array_push($data[$key], $value);
			        }
	                
				$i++;
	        }
	        
	        fclose($handle);
	    }
	    return array_combine($header, $data);
	}
	
	
	function print_key_csharp($data, $key){
		echo '// Liste '.$key.'</br/>';
		echo 'List<float> L = new List<float>(new float[] { ';
		$continue = true;
		
		for($j=0; $continue; $j++){
			echo strtr($data[$key][$j], ",", ".");
			if(array_key_exists($j+1, $data[$key]))
				echo ',';
			else 
				$continue = false;
			echo ' ';
		}
		
		echo '}); <br/><br/>';
	}
	
	function getDiffT($array_timestamp){
		return (end($array_timestamp) - $array_timestamp[0]) * 0.001;
	}
	
	function getAxes($cal = false){
		if($cal){
			$x = 'Shimmer - Low Noise Accelerometer X - CAL - m/(sec^2)*';
			$y = 'Shimmer - Low Noise Accelerometer Y - CAL - m/(sec^2)*';
			$z = 'Shimmer - Low Noise Accelerometer Z - CAL - m/(sec^2)*';
		} else{
			$x = 'Shimmer - Low Noise Accelerometer X - RAW - no units';
			$y = 'Shimmer - Low Noise Accelerometer Y - RAW - no units';
			$z = 'Shimmer - Low Noise Accelerometer Z - RAW - no units';
		}

		return array($x, $y, $z);
	}
	

	// EXTRACTION CSV
	/* 
	array_keys(csv_to_array('csv/Mouvement 1.2.csv', "\t"))
	(
	    [0] => Shimmer - Timestamp - RAW - no units
	    [1] => Shimmer - Timestamp - CAL - mSecs
	    [2] => Shimmer - Low Noise Accelerometer X - RAW - no units
	    [3] => Shimmer - Low Noise Accelerometer X - CAL - m/(sec^2)*
	    [4] => Shimmer - Low Noise Accelerometer Y - RAW - no units
	    [5] => Shimmer - Low Noise Accelerometer Y - CAL - m/(sec^2)*
	    [6] => Shimmer - Low Noise Accelerometer Z - RAW - no units
	    [7] => Shimmer - Low Noise Accelerometer Z - CAL - m/(sec^2)*
	    [8] => Shimmer - Wide Range Accelerometer X - RAW - no units
	    [9] => Shimmer - Wide Range Accelerometer X - CAL - m/(sec^2)*
	    [10] => Shimmer - Wide Range Accelerometer Y - RAW - no units
	    [11] => Shimmer - Wide Range Accelerometer Y - CAL - m/(sec^2)*
	    [12] => Shimmer - Wide Range Accelerometer Z - RAW - no units
	    [13] => Shimmer - Wide Range Accelerometer Z - CAL - m/(sec^2)*
	    [14] => Shimmer - Gyroscope X - RAW - no units
	    [15] => Shimmer - Gyroscope X - CAL - deg/sec*
	    [16] => Shimmer - Gyroscope Y - RAW - no units
	    [17] => Shimmer - Gyroscope Y - CAL - deg/sec*
	    [18] => Shimmer - Gyroscope Z - RAW - no units
	    [19] => Shimmer - Gyroscope Z - CAL - deg/sec*
	    [20] => Shimmer - Magnetometer X - RAW - no units
	    [21] => Shimmer - Magnetometer X - CAL - local*
	    [22] => Shimmer - Magnetometer Y - RAW - no units
	    [23] => Shimmer - Magnetometer Y - CAL - local*
	    [24] => Shimmer - Magnetometer Z - RAW - no units
	    [25] => Shimmer - Magnetometer Z - CAL - local*
	    [26] =>  -  -  - 
	)
	*/
	
	
	
	
/*
	//PRINT C#
	//$mouvement[$i][$j] = csv_to_array('csv/Mouvement '.$i.'.'.$j.'.csv', "\t");
	$axes = getAxes(true);
	for($i=1; $i<3; $i++)
		for($j=1; $j<4; $j++){
			echo '// -------- Mouvement '.$i.'.'.$j.'<br/>';
			foreach($axes as $axe){
				echo '//'.$axe.'<br/>';
				print_key_csharp(csv_to_array('csv/Mouvement '.$i.'.'.$j.'.csv', "\t"), $axe);
			}
			echo '<hr/>';
		}
*/
			
	
	//COMPARER MOUVEMENTS
	$mouvements = array(1, 2);
	$instances = array(1, 2, 3);
	
	$axes = getAxes();
	
	$done = array();
	
	foreach($mouvements as $mvt1)
		foreach($mouvements as $mvt2)
			foreach($instances as $instance1)
				foreach($instances as $instance2){
					
					//On évite de faire les inverses
					if(!in_array($mvt2.'.'.$instance2.'-'.$mvt1.'.'.$instance1, $done)){
						array_push($done, $mvt1.'.'.$instance1.'-'.$mvt2.'.'.$instance2);
						
								
						echo 'Corrélation '.$mvt1.'.'.$instance1.' - '.$mvt2.'.'.$instance2.' = &nbsp;&nbsp;&nbsp;';
						
						$tab1 = csv_to_array('csv/Mouvement '.$mvt1.'.'.$instance1.'.csv', "\t");
						$tab2 = csv_to_array('csv/Mouvement '.$mvt2.'.'.$instance2.'.csv', "\t");
						
						//On ne conserve que les 3 axes
						foreach($tab1 as $key => $value){
							if(!in_array($key, $axes)){
								unset($tab1[$key]);
								unset($tab2[$key]);
							}
						}
						
						echo getCoef($tab1, $tab2);
						echo '<br/>';
					}
				}
	



	
	
	//echo getDiffT($mouvement_1_1['Shimmer - Timestamp - CAL - mSecs']);


?>





