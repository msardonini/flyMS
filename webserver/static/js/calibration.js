
function update_term(message) {
  var li = document.createElement("li");
  li.appendChild(document.createTextNode(message));

  if (terminal_list.getElementsByTagName("li").length > 15) {
    terminal_list.removeChild(terminal_list.firstElementChild);
  }
  terminal_list.appendChild(li);
  var container = document.getElementById("container");
  container.appendChild(terminal_list);
}

function run_calibration(calibration_type) {
  // TODO add check if user really wants to do it

  // Show the terminal div if not already showing
  var terminal_window = document.getElementById("calibration_terminal");
  terminal_window.style.display = 'block';


  var params = 'calibration_type=' + calibration_type;

  var xmlHttp = new XMLHttpRequest();
  xmlHttp.open("GET", '/calibration?' + params, false); // false for synchronous request
  xmlHttp.send(null);


  fetch('/calibrate_gyro_output')
  .then(response => response.body)
  .then(rb => {
    const reader = rb.getReader();

    return new ReadableStream({
      start(controller) {
        // The following function handles each data chunk
        function push() {
          // "done" is a Boolean and value a "Uint8Array"
          reader.read().then( ({done, value}) => {
            // If there is no more data to read
            if (done) {
              console.log('done', done);
              controller.close();
              return;
            }
            // Get the data and send it to the browser via the controller
            // controller.enqueue(value);
            // Check chunks by logging to the console
            const value_str = new TextDecoder().decode(value);
            update_term(value_str)

            push();
          })
        }
        push();
      }
    });
  })
  .then(stream => {
    // Respond with our stream
    return new Response(stream, { headers: { "Content-Type": "text/html" } }).text();
  })
  .then(result => {
    // Do things with result
    // console.log(result);
  });
}

window.addEventListener( "load", function () {
  // Access the form element...
  const form = document.getElementById( "form_calibration" );

  // ...and take over its submit event.
  form.addEventListener( "submit", function ( event ) {
    event.preventDefault();

    var elements = document.getElementsByName("calibration_routines");

    for(i = 0; i < elements.length; i++) {
      if(elements[i].checked) {
        run_calibration(elements[i].value)
      }
  }
  } );
} );

function press_a_key() {
  var xmlHttp = new XMLHttpRequest();
  xmlHttp.open("POST", '/calibrate_gyro_output', false); // false for synchronous request
  xmlHttp.send(null);
}
