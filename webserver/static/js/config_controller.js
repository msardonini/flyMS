function check_value(value, min_value, max_value) {
  if (Number.isFinite(value) || value < min_value || value > max_value) {
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

const varToString = (varObj) => Object.keys(varObj)[0];

function submit_controller(form) {
  var http = new XMLHttpRequest();
  http.open("POST", "/config_controller_app");
  http.setRequestHeader("Content-Type", "application/json;charset=UTF-8");

  // Create the controller sub-section
  var controller = {};
  const roll_inner = [form.Kp_roll_inner.value, form.Ki_roll_inner.value, form.Kd_roll_inner.value];
  if (!check_array_value(roll_inner, 0, Number.MAX_VALUE, 3)) {
    const displayName = varToString({ roll_inner });
    alert(displayName + " has an incorrect value of " + roll_inner);
    return;
  }
  const roll_outer = [form.Kp_roll_outer.value, form.Ki_roll_outer.value, form.Kd_roll_outer.value];

  if (!check_array_value(roll_outer, 0, Number.MAX_VALUE, 3)) {
    const displayName = varToString({ roll_outer });
    alert(displayName + " has an incorrect value of " + roll_outer);
    return;
  }
  const pitch_inner = [form.Kp_pitch_inner.value, form.Ki_pitch_inner.value, form.Kd_pitch_inner.value];
  if (!check_array_value(pitch_inner, 0, Number.MAX_VALUE, 3)) {
    const displayName = varToString({ pitch_inner });
    alert(displayName + " has an incorrect value of " + pitch_inner);
    return;
  }
  const pitch_outer = [form.Kp_pitch_outer.value, form.Ki_pitch_outer.value, form.Kd_pitch_outer.value];
  if (!check_array_value(pitch_outer, 0, Number.MAX_VALUE, 3)) {
    const displayName = varToString({ pitch_outer });
    alert(displayName + " has an incorrect value of " + pitch_outer);
    return;
  }
  const yaw_PID = [form.Kp_yaw.value, form.Ki_yaw.value, form.Kd_yaw.value];
  if (!check_array_value(yaw_PID, 0, Number.MAX_VALUE, 3)) {
    const displayName = varToString({ yaw_PID });
    alert(displayName + " has an incorrect value of " + yaw_PID);
    return;
  }
  controller["roll_PID_inner"] = roll_inner;
  controller["roll_PID_outer"] = roll_outer;
  controller["pitch_PID_inner"] = pitch_inner;
  controller["pitch_PID_outer"] = pitch_outer;
  controller["yaw_PID"] = yaw_PID;

  http.send(JSON.stringify(controller));

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

function load_controller() {
  var xmlHttp = new XMLHttpRequest();
  xmlHttp.open("GET", "/config_controller_app", false); // false for synchronous request
  xmlHttp.send(null);
  let controller = JSON.parse(xmlHttp.responseText);

  // Roll PID Controller
  document.getElementById("Kp_roll_inner").value = controller.roll_PID_inner.at(0);
  document.getElementById("Ki_roll_inner").value = controller.roll_PID_inner.at(1);
  document.getElementById("Kd_roll_inner").value = controller.roll_PID_inner.at(2);
  document.getElementById("Kp_roll_outer").value = controller.roll_PID_outer.at(0);
  document.getElementById("Ki_roll_outer").value = controller.roll_PID_outer.at(1);
  document.getElementById("Kd_roll_outer").value = controller.roll_PID_outer.at(2);

  // Roll PID Controller
  document.getElementById("Kp_pitch_inner").value = controller.pitch_PID_inner.at(0);
  document.getElementById("Ki_pitch_inner").value = controller.pitch_PID_inner.at(1);
  document.getElementById("Kd_pitch_inner").value = controller.pitch_PID_inner.at(2);
  document.getElementById("Kp_pitch_outer").value = controller.pitch_PID_outer.at(0);
  document.getElementById("Ki_pitch_outer").value = controller.pitch_PID_outer.at(1);
  document.getElementById("Kd_pitch_outer").value = controller.pitch_PID_outer.at(2);

  // Yaw Controller
  document.getElementById("Kp_yaw").value = controller.yaw_PID.at(0);
  document.getElementById("Ki_yaw").value = controller.yaw_PID.at(1);
  document.getElementById("Kd_yaw").value = controller.yaw_PID.at(2);
}
