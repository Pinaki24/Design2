#include <WiFi.h>
#include <WebServer.h>
#include "esp_camera.h"
#include "esp32-hal.h" // Include the ESP32 hardware abstraction layer

// Replace with your new network credentials
const char *ssid = "ESP32_AP";    // New WiFi network name (SSID)
const char *password = "ESP32_AP_TEAM2"; // New WiFi password
const char *hostname = "ESP32_QUANTUM"; // Optional hostname for the ESP32

// Create a web server on port 80
WebServer server(80);

// Camera configuration (specific to AI-Thinker ESP32-CAM)
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h" // Pin definitions for the AI-Thinker camera

String telemetryData = ""; // Variable to store telemetry data

void startCameraServer(); // Function to start the camera server

void setup() {
  Serial.begin(115200); // Initialize serial communication at 115200 baud rate

  // Set Wi-Fi mode to Access Point
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password); // Start WiFi in Access Point mode with the provided credentials
  WiFi.setHostname(hostname); // Set the device hostname

  Serial.print("Access Point IP address: ");
  Serial.println(WiFi.softAPIP()); // Print the Access Point IP address

  // Initialize the camera
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM; // Power down pin
  config.pin_reset = RESET_GPIO_NUM; // Reset pin
  config.xclk_freq_hz = 20000000; // XCLK frequency
  config.pixel_format = PIXFORMAT_JPEG; // Use JPEG format for camera output

  // Optional settings to adjust frame size and quality based on PSRAM availability
  if(psramFound()){
    config.frame_size = FRAMESIZE_QVGA; // Set lower resolution (QVGA) for better performance
    config.jpeg_quality = 10; // Set JPEG quality (lower means higher quality)
    config.fb_count = 2; // Use double buffer
  } else {
    config.frame_size = FRAMESIZE_SVGA; // Higher resolution if no PSRAM
    config.jpeg_quality = 12; // Lower JPEG quality for better performance
    config.fb_count = 1; // Single buffer
  }

  // Initialize the camera and check for errors
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  // Adjust camera settings
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA); // Set frame size to QVGA
  s->set_quality(s, 10); // Set quality (lower value = better quality)
  s->set_brightness(s, 0); // Default brightness
  s->set_contrast(s, 0); // Default contrast
  s->set_saturation(s, 0); // Default saturation
  s->set_gainceiling(s, (gainceiling_t)0); // Default gain ceiling
  s->set_special_effect(s, 2); // 2 = grayscale effect

  // Start the web server
  startCameraServer();
}

void loop() {
  server.handleClient(); // Handle client requests
  updateTelemetryData(); // Update telemetry data
  if (Serial.available() > 0) {
    telemetryData = Serial.readStringUntil('\n'); // Read telemetry data from serial input
    Serial.println("Received telemetry data: " + telemetryData);
  }
}

void updateTelemetryData() {
  // Example telemetry data (can be modified to read real sensors)
  int sensorValue = 20; // Simulated analog sensor value
  float temperature = InternaltemperatureRead(); // Read internal temperature sensor
  float batteryLevel = 3.7; // Simulated battery level

  // Format telemetry data as a string
  telemetryData = "Sensor Value: " + String(sensorValue) + "\n";
  telemetryData += "Temperature: " + String(temperature) + " C\n";
  telemetryData += "Battery Level: " + String(batteryLevel) + " V\n";
  telemetryData += "Vivian gonna have to learn ESP32 now\n"; // Custom message
}

void handleCameraPage() {
  // HTML page to display the camera stream
  String html = "<html><head><style>body { font-family: Arial, sans-serif; } #cameraImage { width: 640px; height: 480px; }</style></head><body><h1>ESP32 Camera</h1><img src=\"/capture\" id=\"cameraImage\"><script>setInterval(function() {document.getElementById('cameraImage').src = '/capture?rand=' + Math.random();}, 250);</script></body></html>";
  server.send(200, "text/html", html); // Send HTML page
}

void handleTelemetryPage() {
  // HTML page to display telemetry data
  String html = "<html><head><style>";
  html += "body { font-family: Arial, sans-serif; }";
  html += "pre { font-size: 20px; }"; // Style telemetry data
  html += "</style></head><body>";
  html += "<h1>Telemetry Data</h1>";
  html += "<pre id='telemetry'>" + telemetryData + "</pre>";
  html += "<script>setInterval(function() {fetch('/telemetryData').then(response => response.text()).then(data => {document.getElementById('telemetry').innerText = data;});}, 500);</script>";
  html += "</body></html>";
  server.send(200, "text/html", html); // Send HTML page
}

void handleTelemetryData() {
  // Send the current telemetry data as plain text
  updateTelemetryData();
  server.send(200, "text/plain", telemetryData);
}

void handleCapture() {
  // Capture an image from the camera
  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get(); // Get the camera framebuffer
  if (!fb) {
    Serial.println("Camera capture failed");
    server.send(500, "text/plain", "Camera capture failed");
    return;
  }
  server.send_P(200, "image/jpeg", (const char *)fb->buf, fb->len); // Send the captured image
  esp_camera_fb_return(fb); // Return the framebuffer
}

void handleStream() {
  // Start MJPEG stream
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "multipart/x-mixed-replace; boundary=frame");

  while (server.client().connected()) {  // Check if the client is still connected
    camera_fb_t * fb = esp_camera_fb_get(); // Capture frame buffer
    if (!fb) {
      Serial.println("Camera capture failed");
      break;
    }

    // Send the MJPEG frame
    server.sendContent("--frame\r\n");
    server.sendContent("Content-Type: image/jpeg\r\n\r\n");
    server.sendContent((const char *)fb->buf, fb->len);
    server.sendContent("\r\n");
    esp_camera_fb_return(fb); // Return the frame buffer

    // Add a small delay to avoid overwhelming the client
    delay(100);
  }
}


void startCameraServer() {
  // Set up URL handlers for the web server 
  server.on("/", HTTP_GET, handleCameraPage); // Handle root URL for camera page
  server.on("/telemetry", HTTP_GET, handleTelemetryPage); // Handle telemetry page
  server.on("/telemetryData", HTTP_GET, handleTelemetryData); // Handle telemetry data requests
  server.on("/capture", HTTP_GET, handleCapture); // Handle camera capture requests
  server.on("/stream", HTTP_GET, handleStream); // Handle MJPEG stream requests
  server.begin(); // Start the web server
}

float InternaltemperatureRead() {
  // Read the internal temperature sensor (convert to Celsius)
  return (temperatureRead() - 32) / 1.8; //
}
