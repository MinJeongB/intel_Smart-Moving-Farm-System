<?php
    $conn = mysqli_connect("localhost", "iot", "pwiot");
    mysqli_select_db($conn, "TempHumidb");

    // Sensor table data
    $query_sensor = "SELECT name, date, time, temp, humi FROM sensor";
    $result_sensor = mysqli_query($conn, $query_sensor);
    $data_sensor = array(array('DateTime', 'Temp', 'Humi'));

    if ($result_sensor) {
        while ($row = mysqli_fetch_array($result_sensor)) {
            array_push($data_sensor, array($row['date'] . "\n" . $row['time'], intval($row['temp']), intval($row['humi'])));
        }
    }

    $options_sensor = array(
        'title' => 'Temperature Humidity',
        'width' => 1000, 'height' => 400,
        'curveType' => 'function'
    );

    // IlluSensor table data
    $query_illu = "SELECT name, date, time, illu FROM illu";
    $result_illu = mysqli_query($conn, $query_illu);
    $data_illu = array(array('DateTime', 'Illu'));

    if ($result_illu) {
        while ($row = mysqli_fetch_array($result_illu)) {
            array_push($data_illu, array($row['date'] . "\n" . $row['time'], floatval($row['illu'])));
        }
    }

    $options_illu = array(
        'title' => 'Illumination',
        'width' => 1000, 'height' => 400,
        'curveType' => 'function'
    );
?>

<script src="//www.google.com/jsapi"></script>
<script>
	var data_sensor = <?= json_encode($data_sensor) ?>;
	var options_sensor = <?= json_encode($options_sensor) ?>;

	var data_illu = <?= json_encode($data_illu) ?>;
	var options_illu = <?= json_encode($options_illu) ?>;

	google.load('visualization', '1.0', {'packages': ['corechart']});

	google.setOnLoadCallback(function () {
		var chart_sensor = new google.visualization.LineChart(document.querySelector('#chart_sensor_div'));
		chart_sensor.draw(google.visualization.arrayToDataTable(data_sensor), options_sensor);

		var chart_illu = new google.visualization.LineChart(document.querySelector('#chart_illu_div'));
		chart_illu.draw(google.visualization.arrayToDataTable(data_illu), options_illu);
	});
</script>

<div id="chart_sensor_div"></div>
<div id="chart_illu_div"></div>



