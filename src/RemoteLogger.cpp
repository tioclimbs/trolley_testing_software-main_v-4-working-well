#include "RemoteLogger.h"
#include "arduinosecrets.h"

WiFiServer server(80);
String lastRFIDTag = "--";
unsigned long lastRFIDTime = 0;

void updateRFIDData(const String &tagID, unsigned long timestamp)
{
  lastRFIDTag = tagID;
  lastRFIDTime = timestamp;
}

void connectToWiFi()
{
  WiFi.begin(SECRET_SSID, SECRET_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi connected. IP:");
  Serial.println(WiFi.localIP());
  server.begin();
}

void handleClient(TFMPlus *frontLidar, TFMPlus *rearLidar)
{
  WiFiClient client = server.available();
  if (!client)
    return;

  String request = "";
  while (client.connected())
  {
    if (client.available())
    {
      char c = client.read();
      request += c;
      if (request.endsWith("\r\n\r\n"))
        break;
    }
  }

  if (request.indexOf("GET /data") >= 0)
  {
    uint16_t front_dist = frontLidar->data.dist;
    uint16_t front_str = frontLidar->data.flux;
    uint16_t rear_dist = rearLidar->data.dist;
    uint16_t rear_str = rearLidar->data.flux;
    unsigned long ts = millis();

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: application/json");
    client.println("Connection: close");
    client.println();
    client.print("{\"front_distance\":");
    client.print(front_dist);
    client.print(",\"front_strength\":");
    client.print(front_str);
    client.print(",\"rear_distance\":");
    client.print(rear_dist);
    client.print(",\"rear_strength\":");
    client.print(rear_str);
    client.print(",\"rfid_tag\":\"");
    client.print(lastRFIDTag);
    client.print("\",\"rfid_time\":");
    client.print(lastRFIDTime);
    client.print(",\"timestamp\":");
    client.print(ts);
    client.println("}");
  }

  else if (request.indexOf("GET /reset") >= 0)
  {
    Serial.println("🧹 Log reset triggered.");
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/plain");
    client.println("Connection: close");
    client.println();
    client.println("Reset OK");
  }

  else if (request.indexOf("GET /clear_rfid") >= 0)
  {
    Serial.println("🧹 RFID cleared.");
    lastRFIDTag = "--";
    lastRFIDTime = 0;
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/plain");
    client.println("Connection: close");
    client.println();
    client.println("RFID cleared");
  }

  else
  {
    // Serve HTML UI
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close");
    client.println();
    client.println(R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Trolley Testing Dashboard</title>
  <meta charset="UTF-8">
  <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
  <style>
    body {
      margin: 0;
      font-family: 'Segoe UI', sans-serif;
      background: linear-gradient(135deg, #007BFF, #00BFFF);
      color: #fff;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      height: 100vh;
    }
    .container {
      background-color: rgba(0, 0, 50, 0.2);
      border-radius: 20px;
      padding: 30px 50px;
      box-shadow: 0 8px 16px rgba(0,0,0,0.3);
      text-align: center;
    }
    h1 {
      margin-bottom: 20px;
      font-size: 2em;
      color: #E0F7FF;
    }
    .data p {
      font-size: 1.5em;
      margin: 10px 0;
    }
    .label { color: #B0DFFF; font-weight: bold; }
    .value { color: #fff; font-weight: bold; }
    button {
      margin-top: 20px;
      padding: 10px 20px;
      font-size: 1em;
      border: none;
      border-radius: 10px;
      background-color: #005fbb;
      color: white;
      cursor: pointer;
    }
  </style>
</head>
<body>
  <div class="container">
    <h1>Trolley Testing Dashboard</h1>
    <div class="data">
      <p><span class="label">Front Distance:</span> <span class="value" id="front_distance">--</span> cm</p>
      <p><span class="label">Front Strength:</span> <span class="value" id="front_strength">--</span></p>
      <p><span class="label">Rear Distance:</span> <span class="value" id="rear_distance">--</span> cm</p>
      <p><span class="label">Rear Strength:</span> <span class="value" id="rear_strength">--</span></p>
      <p><span class="label">Last Updated:</span> <span class="value" id="timestamp">--</span> ms</p>
      <p><span class="label">RFID Tag:</span> <span class="value" id="rfid_tag">--</span></p>
      <p><span class="label">Last RFID Time:</span> <span class="value" id="rfid_time">--</span></p>


    </div>
    <canvas id="distanceChart" width="400" height="200"></canvas>
    
    <button onclick="clearRFID()">Clear RFID</button>

    <button onclick="resetLog()">Reset Log</button>
  </div>

  <script>
    let distanceChart = new Chart(document.getElementById("distanceChart"), {
      type: 'line',
      data: {
        labels: [],
        datasets: [
          {
            label: 'Front Distance (cm)',
            data: [],
            borderColor: 'rgba(255, 255, 255, 0.8)',
            borderWidth: 2,
            fill: false
          },
          {
            label: 'Rear Distance (cm)',
            data: [],
            borderColor: 'rgba(255, 255, 0, 0.8)',
            borderWidth: 2,
            fill: false
          }
        ]
      },
      options: {
        scales: {
          x: { title: { display: true, text: 'Time (ms)' } },
          y: { beginAtZero: true, title: { display: true, text: 'Distance (cm)' } }
        }
      }
    });

    function fetchData() {
      fetch("/data")
        .then(response => response.json())
        .then(data => {
          document.getElementById("front_distance").textContent = data.front_distance;
          document.getElementById("rear_distance").textContent = data.rear_distance;
          document.getElementById("front_strength").textContent = data.front_strength;
          document.getElementById("rear_strength").textContent = data.rear_strength;
          document.getElementById("timestamp").textContent = data.timestamp;
          document.getElementById("rfid_tag").textContent = data.rfid_tag;
          document.getElementById("rfid_time").textContent = data.rfid_time;


          distanceChart.data.labels.push(data.timestamp);
          distanceChart.data.datasets[0].data.push(data.front_distance);
          distanceChart.data.datasets[1].data.push(data.rear_distance);

          if (distanceChart.data.labels.length > 30) {
            distanceChart.data.labels.shift();
            distanceChart.data.datasets[0].data.shift();
            distanceChart.data.datasets[1].data.shift();
          }

          distanceChart.update();
        });
    }

    function resetLog() {
      fetch("/reset").then(() => {
        // Clear chart data
        distanceChart.data.labels = [];
        distanceChart.data.datasets[0].data = [];
        distanceChart.data.datasets[1].data = [];
        distanceChart.update();

        // Reset displayed values
        document.getElementById("front_distance").textContent = "--";
        document.getElementById("rear_distance").textContent = "--";
        document.getElementById("front_strength").textContent = "--";
        document.getElementById("rear_strength").textContent = "--";
        document.getElementById("timestamp").textContent = "--";
        document.getElementById("rfid_tag").textContent = "--";
        document.getElementById("rfid_time").textContent = "--";

        alert("Log reset!");
      });
    }

    
    function clearRFID() {
      fetch("/clear_rfid").then(() => {
        document.getElementById("rfid_tag").textContent = "--";
        document.getElementById("rfid_time").textContent = "--";

      // Optional: Clear LiDAR data too (if treating Clear RFID like a full UI reset)
        document.getElementById("front_distance").textContent = "--";
        document.getElementById("rear_distance").textContent = "--";
        document.getElementById("front_strength").textContent = "--";
        document.getElementById("rear_strength").textContent = "--";
        document.getElementById("timestamp").textContent = "--";
        distanceChart.data.labels = [];
        distanceChart.data.datasets[0].data = [];
        distanceChart.data.datasets[1].data = [];
        distanceChart.update();

        alert("RFID tag cleared");
      });
    }



    setInterval(fetchData, 1000);
    fetchData();
  </script>
</body>
</html>
)rawliteral");
  }

  delay(1);
  client.stop();
}
