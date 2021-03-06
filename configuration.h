
String WEB_HOME_PAGE = "<!DOCTYPEhtml><html><head><title>DronotiqueSignal</title><metacharset=\"UTF-8\"><meta http-equiv=\"refresh\" content=\"1\"/></head><body><h1>Dispositif-[ID_DISPOSITIF]</h1><h2>Version-[VERSION]</h2><h3>[PROTOCOLE_MODE]</h3><h3>[MESSAGE]</h3><table><tr><th>FixGPS</th><td>[FIX_GPS]</td></tr><tr><th>NbSatellite</th><td>[NB_SAT]</td></tr><tr><th>HDOP</th><td>[HDOP]</td></tr><tr><th>Lattitude</th><td>[LAT]</td></tr><tr><th>Longitude</th><td>[LON]</td></tr><tr><th>Altitude</th><td>[Alt]</td></tr><tr><th>Vitesse</th><td>[SPEED]</td></tr><tr><th>Route</th><td>[ROUTE]</td></tr></table><a href=\"update\">Mise à jour</a></body></html>";
/*
 * Server Index Page
 */
 
const char* WEB_UPDATE_PAGE = "<form method='POST' action='update_go' enctype='multipart/form-data' id='upload_form'><input type='file' name='update'><input type='submit' value='Update'></form>";
/*"<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
"<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
   "<input type='file' name='update'>"
        "<input type='submit' value='Update'>"
    "</form>"
 "<div id='prg'>progress: 0%</div>"
 "<script>"
  "$('form').submit(function(e){"
  "e.preventDefault();"
  "var form = $('#upload_form')[0];"
  "var data = new FormData(form);"
  " $.ajax({"
  "url: '/update_go',"
  "type: 'POST',"
  "data: data,"
  "contentType: false,"
  "processData:false,"
  "xhr: function() {"
  "var xhr = new window.XMLHttpRequest();"
  "xhr.upload.addEventListener('progress', function(evt) {"
  "if (evt.lengthComputable) {"
  "var per = evt.loaded / evt.total;"
  "$('#prg').html('progress: ' + Math.round(per*100) + '%');"
  "}"
  "}, false);"
  "return xhr;"
  "},"
  "success:function(d, s) {"
  "console.log('success!')" 
 "},"
 "error: function (a, b, c) {"
 "}"
 "});"
 "});"
 "</script>";*/
