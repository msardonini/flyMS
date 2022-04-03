function check_value(value, min_value, max_value) {
  if (Number.isFinite(value) || value < min_value || value > max_value) {
    console.log("value is " + value);
    alert("Array has incorrect value!");
    return false;
  } else {
    return true;
  }
}

function check_array_value(array, min_value, max_value, length) {
  let result = true;
  for (var i = 0; i < length; i++) {
    result &= check_value(array[i], min_value, max_value);
  }
  return result;
}

function load_setpoint() {
  var xmlHttp = new XMLHttpRequest();
  xmlHttp.open("GET", "/config_setpoint_app", false); // false for synchronous request
  xmlHttp.send(null);
  let setpoint = JSON.parse(xmlHttp.responseText);

  // Min/Max Throttle
  document.getElementById("min_throttle").value = setpoint.throttle_limits.at(0);
  document.getElementById("max_throttle").value = setpoint.throttle_limits.at(1);

  // Max Setpoint Stabilized
  document.getElementById("max_roll_setpoint_stabilized").value = setpoint.max_setpoints_stabilized.at(0);
  document.getElementById("max_pitch_setpoint_stabilized").value = setpoint.max_setpoints_stabilized.at(1);
  document.getElementById("max_yaw_setpoint_stabilized").value = setpoint.max_setpoints_stabilized.at(2);

  document.getElementById("max_roll_setpoint_acro").value = setpoint.max_setpoints_acro.at(0);
  document.getElementById("max_pitch_setpoint_acro").value = setpoint.max_setpoints_acro.at(1);
  document.getElementById("max_yaw_setpoint_acro").value = setpoint.max_setpoints_acro.at(2);
}

function submit_setpoint(form) {
  var http = new XMLHttpRequest();
  http.open("POST", "/config_setpoint_app");
  http.setRequestHeader("Content-Type", "application/json;charset=UTF-8");

  // Create the setpoint sub-section
  var setpoint = {};
  const throttle_limits = [form.min_throttle.value, form.max_throttle.value];
  if (!check_array_value(throttle_limits, 0, 1, 2)) {
    return;
  }
  const max_setpoints_stabilized = [
    form.max_roll_setpoint_stabilized.value,
    form.max_pitch_setpoint_stabilized.value,
    form.max_yaw_setpoint_stabilized.value,
  ];
  if (!check_array_value(max_setpoints_stabilized, 0, 1.58, 2)) {
    return;
  }

  console.log(form.max_yaw_setpoint_stabilized.style.borderColor);
  if (!check_value(max_setpoints_stabilized[2], 0, 10)) {
    form.max_yaw_setpoint_stabilized.style.borderColor = "red";
    return;
  } else {
    form.max_yaw_setpoint_stabilized.style.borderColor = "";
  }

  const max_setpoints_acro = [
    form.max_roll_setpoint_acro.value,
    form.max_pitch_setpoint_acro.value,
    form.max_yaw_setpoint_acro.value,
  ];
  if (!check_array_value(max_setpoints_stabilized, 0, 10, 3)) {
    return;
  }
  setpoint["throttle_limits"] = throttle_limits;
  setpoint["max_setpoints_stabilized"] = max_setpoints_stabilized;
  setpoint["max_setpoints_acro"] = max_setpoints_acro;

  http.send(JSON.stringify(setpoint));

  http.onreadystatechange = function () {
    if (http.readyState === 4) {
      var response = JSON.parse(http.responseText);
      if (http.status === 200) {
        alert("Configuration Sent Successfully");
      } else {
        alert("Configuration Failed to Send");
      }
    }
  };
}
