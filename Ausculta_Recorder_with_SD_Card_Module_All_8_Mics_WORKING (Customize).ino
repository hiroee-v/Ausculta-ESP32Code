#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <driver/i2s.h>
#include <SPI.h>
#include <SD.h>
#include "esp_timer.h"

const char* AP_SSID = "AUSCULTA-RECORDER";
const char* AP_PASSWORD = "12345678";

WebServer server(80);

// --- EXACT PIN MAPPING MATCHING YOUR WIRING ---
#define I2S_PORT            I2S_NUM_0
#define I2S_BCLK_PIN        41  // SCK (Orange) -> Shared SCK
#define I2S_WS_PIN          42  // WS (Yellow) -> Shared WS
#define I2S_SD_PIN          17  // Z Pin of 74HC4051 Multiplexer (Black)

// 74HC4051 Multiplexer Control Pins
#define MUX_S0              5   // S0 (White)
#define MUX_S1              6   // S1 (Grey)
#define MUX_S2              7   // S2 (Purple)

// MicroSD Card Pin Assignments
#define SD_CS_PIN           15  // CS (Blue)
#define SPI_SCK_PIN         2   // SCK (Green)
#define SPI_MOSI_PIN        4   // MOSI (Yellow)
#define SPI_MISO_PIN        16  // MISO (Orange)

#define SAMPLE_RATE         16000
#define RECORD_SECONDS      20                  
#define MAX_SAMPLES         (SAMPLE_RATE * RECORD_SECONDS) // 320,000 samples per mic (640 KB)

int16_t* audioBuffer = NULL;

volatile bool recording = false;
volatile bool scanningAll = false;
volatile bool recordingFinished = false;
volatile uint32_t sampleCount = 0;
volatile bool recordingTaskRunning = false;
TaskHandle_t recordingTaskHandle = NULL;
uint8_t selectedMic = 1;
String statusMessage = "Ready";

// --- DSP Filter Settings ---
#define FILTER_LOW_HZ          80.0f
#define FILTER_HIGH_HZ       1200.0f
#define FILTER_NOTCH_HZ        60.0f
#define SIGNAL_GAIN             4.0f
#define TRANSIENT_SPIKE_LIMIT 30000.0f

volatile int16_t recordedMin = 0;
volatile int16_t recordedMax = 0;
volatile uint32_t rmsLevel = 0;
uint32_t fileCounter = 0;

struct Biquad { float b0, b1, b2, a1, a2, x1, x2, y1, y2; };
// Explicit declarations avoid Arduino generating prototypes before this type.
void setupLowPass(Biquad &f, float frequency, float Q);
void setupHighPass(Biquad &f, float frequency, float Q);
void setupNotch(Biquad &f, float frequency, float Q);
float processBiquad(Biquad &f, float input);
Biquad highPass, notch60, lowPass;

void setupLowPass(Biquad &f, float frequency, float Q) {
  float w0 = 2.0f * PI * frequency / SAMPLE_RATE;
  float cosw0 = cosf(w0), sinw0 = sinf(w0);
  float alpha = sinw0 / (2.0f * Q);
  float b0 = (1.0f - cosw0) / 2.0f, b1 = 1.0f - cosw0, b2 = (1.0f - cosw0) / 2.0f;
  float a0 = 1.0f + alpha, a1 = -2.0f * cosw0, a2 = 1.0f - alpha;
  f.b0 = b0 / a0; f.b1 = b1 / a0; f.b2 = b2 / a0; f.a1 = a1 / a0; f.a2 = a2 / a0;
  f.x1 = f.x2 = f.y1 = f.y2 = 0;
}

void setupHighPass(Biquad &f, float frequency, float Q) {
  float w0 = 2.0f * PI * frequency / SAMPLE_RATE;
  float cosw0 = cosf(w0), sinw0 = sinf(w0);
  float alpha = sinw0 / (2.0f * Q);
  float b0 = (1.0f + cosw0) / 2.0f, b1 = -(1.0f + cosw0), b2 = (1.0f + cosw0) / 2.0f;
  float a0 = 1.0f + alpha, a1 = -2.0f * cosw0, a2 = 1.0f - alpha;
  f.b0 = b0 / a0; f.b1 = b1 / a0; f.b2 = b2 / a0; f.a1 = a1 / a0; f.a2 = a2 / a0;
  f.x1 = f.x2 = f.y1 = f.y2 = 0;
}

void setupNotch(Biquad &f, float frequency, float Q) {
  float w0 = 2.0f * PI * frequency / SAMPLE_RATE;
  float cosw0 = cosf(w0), sinw0 = sinf(w0);
  float alpha = sinw0 / (2.0f * Q);
  float b0 = 1.0f, b1 = -2.0f * cosw0, b2 = 1.0f;
  float a0 = 1.0f + alpha, a1 = -2.0f * cosw0, a2 = 1.0f - alpha;
  f.b0 = b0 / a0; f.b1 = b1 / a0; f.b2 = b2 / a0; f.a1 = a1 / a0; f.a2 = a2 / a0;
  f.x1 = f.x2 = f.y1 = f.y2 = 0;
}

float processBiquad(Biquad &f, float input) {
  float output = f.b0 * input + f.b1 * f.x1 + f.b2 * f.x2 - f.a1 * f.y1 - f.a2 * f.y2;
  f.x2 = f.x1; f.x1 = input; f.y2 = f.y1; f.y1 = output;
  return output;
}

void resetFilters() {
  highPass.x1 = highPass.x2 = highPass.y1 = highPass.y2 = 0;
  notch60.x1 = notch60.x2 = notch60.y1 = notch60.y2 = 0;
  lowPass.x1 = lowPass.x2 = lowPass.y1 = lowPass.y2 = 0;
}

void selectMicrophone(uint8_t micIndex) {
  if (micIndex < 1) micIndex = 1;
  if (micIndex > 8) micIndex = 8;
  selectedMic = micIndex;

  uint8_t channel = micIndex - 1;
  digitalWrite(MUX_S0, (channel & 0x01) ? HIGH : LOW);
  digitalWrite(MUX_S1, (channel & 0x02) ? HIGH : LOW);
  digitalWrite(MUX_S2, (channel & 0x04) ? HIGH : LOW);
  delay(15);
}

void initI2S() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK_PIN,
    .ws_io_num = I2S_WS_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD_PIN
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
  i2s_start(I2S_PORT);
}

void initSDCard() {
  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SD_CS_PIN);
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("[SD] Initialization failed! Check card wiring or FAT32 format.");
    return;
  }
  Serial.println("[SD] MicroSD Card initialized successfully.");
}

template <typename T>
void writeWavHeader(T &stream, uint32_t dataSize) {
  uint32_t fileSize = 36 + dataSize, fmtSize = 16;
  uint16_t audioFormat = 1, channels = 1, bitsPerSample = 16, blockAlign = channels * bitsPerSample / 8;
  uint32_t sampleRate = SAMPLE_RATE, byteRate = sampleRate * blockAlign;
  stream.write((const uint8_t*)"RIFF", 4); stream.write((uint8_t*)&fileSize, 4); stream.write((const uint8_t*)"WAVE", 4);
  stream.write((const uint8_t*)"fmt ", 4); stream.write((uint8_t*)&fmtSize, 4); stream.write((uint8_t*)&audioFormat, 2);
  stream.write((uint8_t*)&channels, 2); stream.write((uint8_t*)&sampleRate, 4); stream.write((uint8_t*)&byteRate, 4);
  stream.write((uint8_t*)&blockAlign, 2); stream.write((uint8_t*)&bitsPerSample, 2); stream.write((const uint8_t*)"data", 4);
  stream.write((uint8_t*)&dataSize, 4);
}

const char* SENSOR_NAMES[] = {
  "Sensor 1 (Upper Right Chest)",
  "Sensor 2 (Upper Left Chest)",
  "Sensor 3 (Lower Right Chest)",
  "Sensor 4 (Lower Left Chest)",
  "Sensor 5 (Upper Right Back)",
  "Sensor 6 (Upper Left Back)",
  "Sensor 7 (Lower Right Back)",
  "Sensor 8 (Lower Left Back)",
};

String sensorName(uint8_t sensor) {
  if (sensor < 1 || sensor > 8) return "Unknown Sensor";
  return SENSOR_NAMES[sensor - 1];
}

String nextSensorFilename(uint8_t sensor) {
  const String base = "/" + sensorName(sensor);
  String filename = base + ".wav";
  uint32_t number = 2;
  // Inspect the SD card so numbering also survives a restart.
  while (SD.exists(filename.c_str())) {
    filename = base + " (" + String(number++) + ").wav";
  }
  return filename;
}

void saveWavToSD(const char* filename, int16_t* buffer, uint32_t count) {
  if (SD.exists(filename)) {
    Serial.printf("[SD] Refusing to overwrite existing recording: %s\n", filename);
    return;
  }

  File file = SD.open(filename, FILE_WRITE);
  if (!file) {
    Serial.println("[SD] Failed to open file for writing.");
    return;
  }

  uint32_t dataSize = count * sizeof(int16_t);
  writeWavHeader(file, dataSize);

  const size_t chunkSize = 512;
  uint8_t* bytePtr = (uint8_t*)buffer;
  uint32_t remainingBytes = dataSize;

  while (remainingBytes > 0) {
    size_t toWrite = (remainingBytes < chunkSize) ? remainingBytes : chunkSize;
    file.write(bytePtr, toWrite);
    bytePtr += toWrite;
    remainingBytes -= toWrite;
  }

  file.close();
  Serial.printf("[SD] Saved recording to %s (%u bytes)\n", filename, dataSize + 44);
}

const char MAIN_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta name="viewport" content="width=device-width, initial-scale=1"><title>AUSCULTA Electronic Stethoscope</title><style>body{font-family:Arial,sans-serif;background:#101820;color:white;margin:0}.container{max-width:500px;margin:auto;padding:30px 20px}button,select{width:100%;padding:14px;margin:6px 0;border:none;border-radius:12px;font-size:16px;font-weight:bold}select{background:#222;color:white}.record{background:#e53935;color:white}.scan{background:#7b1fa2;color:white}.stop{background:#555;color:white}.play{background:#2196f3;color:white}audio{width:100%;margin-top:15px}</style></head><body><div class="container"><h1>AUSCULTA Stethoscope</h1><div class="subtitle">ESP32-S3 + Mux + SD Card</div><div class="status"><div class="statusText" id="status">Ready</div></div><div class="timer" id="timer">0.0s</div>
<label>Select Sensor Zone (Manual):</label>
<select id="micSelect">
  <option value="1">Sensor 1 (Upper Right Chest)</option><option value="2">Sensor 2 (Upper Left Chest)</option>
  <option value="3">Sensor 3 (Lower Right Chest)</option><option value="4">Sensor 4 (Lower Left Chest)</option>
  <option value="5">Sensor 5 (Upper Right Back)</option><option value="6">Sensor 6 (Upper Left Back)</option>
  <option value="7">Sensor 7 (Lower Right Back)</option><option value="8">Sensor 8 (Lower Left Back)</option>
</select>
<button onclick="setMic()">Set Channel</button>

<button class="record" id="recordBtn" onclick="startRecording()">RECORD SINGLE (20s)</button>
<!-- Restore this button to show auto-scan again; the function and endpoint remain.
<button class="scan" id="scanBtn" onclick="recordAllMics()">AUTO-SCAN ALL 8 ZONES (20s + 5s Break)</button>
-->
<button class="stop" id="stopBtn" onclick="stopRecording()" disabled>STOP</button>

<label style="margin-top:15px;display:block">Select Recorded File:</label>
<select id="fileSelect"></select>
<button class="play" id="playBtn" onclick="playRecording()" disabled>PLAY SELECTED</button>
<button class="play" id="downloadBtn" onclick="downloadRecording()" disabled>DOWNLOAD SELECTED (.wav)</button>
<audio id="audioPlayer" controls></audio>
</div>
<script>
let timerInterval;let startTime;
function setScanButtonDisabled(disabled){
  const button = document.getElementById("scanBtn");
  if(button) button.disabled = disabled;
}
async function loadFileList(){
  try{
    // Added cache-busting timestamp (?t=...) so the browser never caches the file list
    let res = await fetch("/list?t=" + Date.now());
    let files = await res.json();
    let select = document.getElementById("fileSelect");
    select.innerHTML = "";
    files.forEach(f => {
      let opt = document.createElement("option");
      opt.value = f;
      opt.innerText = f;
      select.appendChild(opt);
    });
    if(files.length > 0) document.getElementById("playBtn").disabled = false;
    document.getElementById("downloadBtn").disabled = files.length === 0;
  }catch(e){console.error("Failed to load file list");}
}
async function setMic(){
  let mic = document.getElementById("micSelect").value;
  let res = await fetch("/mic?channel="+mic);
  let data = await res.text();
  alert(data);
}
async function startRecording(){
  document.getElementById("recordBtn").disabled=true;
  setScanButtonDisabled(true);
  document.getElementById("stopBtn").disabled=false;
  document.getElementById("playBtn").disabled=true;
  document.getElementById("status").innerText="Recording Single (20s)...";
  document.getElementById("timer").innerText="0.0s";
  try{
    let mic = document.getElementById("micSelect").value;
    let response=await fetch("/record/start?channel="+mic);
    if(!response.ok){throw new Error("Start failed")}
    startTime=Date.now();
    timerInterval=setInterval(updateTimer,100)
  }catch(error){
    document.getElementById("status").innerText="Connection error";
    document.getElementById("recordBtn").disabled=false;
    setScanButtonDisabled(false);
    document.getElementById("stopBtn").disabled=true;
  }
}
async function recordAllMics(){
  document.getElementById("recordBtn").disabled=true;
  setScanButtonDisabled(true);
  document.getElementById("stopBtn").disabled=false;
  document.getElementById("playBtn").disabled=true;
  document.getElementById("status").innerText="Auto-scanning 8 zones (20s rec + 5s break)...";
  document.getElementById("timer").innerText="Automated Sequence Active";
  try{
    let res = await fetch("/record/all");
    let data = await res.text();
    document.getElementById("status").innerText=data;
    document.getElementById("recordBtn").disabled=false;
    setScanButtonDisabled(false);
    document.getElementById("stopBtn").disabled=true;
    document.getElementById("timer").innerText="Sequence Complete";
    loadFileList();
  }catch(e){
    document.getElementById("status").innerText="Scan failed or interrupted";
    document.getElementById("recordBtn").disabled=false;
    setScanButtonDisabled(false);
    document.getElementById("stopBtn").disabled=true;
  }
}
function updateTimer(){
  let elapsed=(Date.now()-startTime)/1000;
  if(elapsed>20){elapsed=20}
  document.getElementById("timer").innerText=elapsed.toFixed(1)+"s";
  if(elapsed>=20){clearInterval(timerInterval)}
}
async function stopRecording(){
  clearInterval(timerInterval);
  document.getElementById("status").innerText="Stopping...";
  document.getElementById("stopBtn").disabled=true;
  try{
    await fetch("/record/stop");
    document.getElementById("recordBtn").disabled=false;
    setScanButtonDisabled(false);
    document.getElementById("playBtn").disabled=false;
    document.getElementById("status").innerText="Stopped & Ready";
    loadFileList();
  }catch(error){
    document.getElementById("status").innerText="Connection error";
    document.getElementById("recordBtn").disabled=false;
    setScanButtonDisabled(false);
  }
}
function downloadRecording(){
  const filename = document.getElementById("fileSelect").value;
  if(!filename) return;
  const link = document.createElement("a");
  link.href = "/audio?file=" + encodeURIComponent(filename) + "&download=1";
  link.download = filename.split("/").pop();
  document.body.appendChild(link);
  link.click();
  link.remove();
}
function playRecording(){
  let filename = document.getElementById("fileSelect").value;
  if(!filename) return;
  let audio=document.getElementById("audioPlayer");
  audio.src="/audio?file="+encodeURIComponent(filename)+"&"+Date.now();
  audio.load();
  audio.play();
  document.getElementById("status").innerText="Playing " + filename + "...";
}
loadFileList();
</script></body></html>)rawliteral";

void recordingTask(void*) {
  recordingTaskRunning = true;
  int16_t localMin = 32767, localMax = -32768;
  uint64_t sumSquares = 0;

  int32_t dmaBuffer[128];
  size_t bytesRead = 0;

  while (recording && sampleCount < MAX_SAMPLES) {
    if (i2s_read(I2S_PORT, dmaBuffer, sizeof(dmaBuffer), &bytesRead, pdMS_TO_TICKS(100)) == ESP_OK && bytesRead > 0) {
      size_t samplesRead = bytesRead / sizeof(int32_t);

      for (size_t i = 0; i < samplesRead && sampleCount < MAX_SAMPLES; i++) {
        int16_t rawPCM = (int16_t)(dmaBuffer[i] >> 16);

        float signal = static_cast<float>(rawPCM);
        signal = processBiquad(highPass, signal);
        signal = processBiquad(notch60, signal);
        signal = processBiquad(lowPass, signal);
        signal *= SIGNAL_GAIN;

        if (signal > TRANSIENT_SPIKE_LIMIT) signal = TRANSIENT_SPIKE_LIMIT;
        if (signal < -TRANSIENT_SPIKE_LIMIT) signal = -TRANSIENT_SPIKE_LIMIT;
        if (signal > 32767.0f) signal = 32767.0f;
        if (signal < -32768.0f) signal = -32768.0f;

        const int16_t pcm = static_cast<int16_t>(signal);
        if (pcm < localMin) localMin = pcm;
        if (pcm > localMax) localMax = pcm;
        sumSquares += static_cast<int32_t>(pcm) * static_cast<int32_t>(pcm);
        audioBuffer[sampleCount++] = pcm;
      }
    }
    vTaskDelay(1);
  }

  recordedMin = localMin;
  recordedMax = localMax;
  rmsLevel = sampleCount > 0 ? static_cast<uint32_t>(sqrt(static_cast<double>(sumSquares) / sampleCount)) : 0;
  
  if (!scanningAll) {
    fileCounter++;
    const String filename = nextSensorFilename(selectedMic);
    saveWavToSD(filename.c_str(), audioBuffer, sampleCount);
  }

  recording = false;
  recordingFinished = true;
  recordingTaskRunning = false;
  recordingTaskHandle = NULL;
  vTaskDelete(NULL);
}

void startRecording() {
  if (recording || recordingTaskRunning) return;
  sampleCount = 0;
  recordingFinished = false;
  recordedMin = 0; recordedMax = 0; rmsLevel = 0;
  resetFilters();
  recording = true;
  xTaskCreatePinnedToCore(recordingTask, "RecordingTask", 4096, NULL, 3, &recordingTaskHandle, 0);
}

void stopRecording() {
  if (!recording) return;
  recording = false;
  unsigned long timeout = millis() + 500;
  while (recordingTaskRunning && millis() < timeout) delay(1);
}

void recordAllMicsTask(void*) {
  scanningAll = true;
  fileCounter++;
  
  for (uint8_t ch = 1; ch <= 8; ch++) {
    selectMicrophone(ch);
    i2s_zero_dma_buffer(I2S_PORT);
    resetFilters();

    sampleCount = 0;
    int16_t localMin = 32767, localMax = -32768;
    uint64_t sumSquares = 0;
    int32_t dmaBuffer[128];
    size_t bytesRead = 0;
    unsigned long startMicTime = millis();

    while ((millis() - startMicTime < 20000) && sampleCount < MAX_SAMPLES) {
      if (i2s_read(I2S_PORT, dmaBuffer, sizeof(dmaBuffer), &bytesRead, pdMS_TO_TICKS(100)) == ESP_OK && bytesRead > 0) {
        size_t samplesRead = bytesRead / sizeof(int32_t);
        for (size_t i = 0; i < samplesRead && sampleCount < MAX_SAMPLES; i++) {
          int16_t rawPCM = (int16_t)(dmaBuffer[i] >> 16);
          float signal = static_cast<float>(rawPCM);
          signal = processBiquad(highPass, signal);
          signal = processBiquad(notch60, signal);
          signal = processBiquad(lowPass, signal);
          signal *= SIGNAL_GAIN;

          if (signal > TRANSIENT_SPIKE_LIMIT) signal = TRANSIENT_SPIKE_LIMIT;
          if (signal < -TRANSIENT_SPIKE_LIMIT) signal = -TRANSIENT_SPIKE_LIMIT;
          if (signal > 32767.0f) signal = 32767.0f;
          if (signal < -32768.0f) signal = -32768.0f;

          const int16_t pcm = static_cast<int16_t>(signal);
          if (pcm < localMin) localMin = pcm;
          if (pcm > localMax) localMax = pcm;
          sumSquares += static_cast<int32_t>(pcm) * static_cast<int32_t>(pcm);
          audioBuffer[sampleCount++] = pcm;
        }
      }
    }

    const String filename = nextSensorFilename(ch);
    saveWavToSD(filename.c_str(), audioBuffer, sampleCount);
    Serial.printf("[SCAN] Saved Mic %u (20 seconds).\n", ch);

    if (ch < 8) {
      Serial.printf("[SCAN] Pausing for 5 seconds before switching to Mic %u...\n", ch + 1);
      unsigned long pauseStart = millis();
      while (millis() - pauseStart < 5000) {
        server.handleClient(); 
        delay(10);
      }
    }
  }

  scanningAll = false;
  vTaskDelete(NULL);
}

void setup() {
  Serial.begin(115200); 
  delay(1000);
  
  Serial.println("\n========================================");
  Serial.println("AUSCULTA STETHOSCOPE - AUTO-SCAN 8 ZONES");
  Serial.println("========================================");

  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  selectMicrophone(1);

  audioBuffer = (int16_t*) ps_malloc(MAX_SAMPLES * sizeof(int16_t));
  if (!audioBuffer) {
    audioBuffer = (int16_t*) malloc(MAX_SAMPLES * sizeof(int16_t));
  }

  initI2S();
  initSDCard();

  setupHighPass(highPass, FILTER_LOW_HZ, 0.707f); 
  setupNotch(notch60, FILTER_NOTCH_HZ, 10.0f); 
  setupLowPass(lowPass, FILTER_HIGH_HZ, 0.707f); 
  resetFilters();

  delay(500);
  WiFi.mode(WIFI_AP); 
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);

  server.on("/", HTTP_GET, [](){ server.send_P(200, "text/html", MAIN_PAGE); });
  
  server.on("/mic", HTTP_GET, [](){
    if (server.hasArg("channel")) {
      int ch = server.arg("channel").toInt();
      selectMicrophone(ch);
      i2s_zero_dma_buffer(I2S_PORT);
      server.send(200, "text/plain", "Selected " + sensorName(selectedMic));
    } else {
      server.send(400, "text/plain", "Missing channel");
    }
  });

  server.on("/record/start", HTTP_GET, [](){ 
    if (recording || scanningAll) { server.send(409, "text/plain", "Busy"); return; } 
    if (server.hasArg("channel")) {
      selectMicrophone(server.arg("channel").toInt());
    }
    startRecording(); 
    server.send(200, "text/plain", "Started"); 
  });
  
  server.on("/record/all", HTTP_GET, [](){
    if (recording || scanningAll) { server.send(409, "text/plain", "Busy"); return; }
    xTaskCreatePinnedToCore(recordAllMicsTask, "ScanAllTask", 8192, NULL, 3, NULL, 0);
    server.send(200, "text/plain", "Automated 8-zone scan started!");
  });

  server.on("/record/stop", HTTP_GET, [](){ stopRecording(); server.send(200, "text/plain", "Stopped"); });

  // Fixed SD root traversal with proper closing handles to avoid file handle leaks
  server.on("/list", HTTP_GET, [](){
    String json = "[";
    File root = SD.open("/");
    bool first = true;
    if (root) {
      File file = root.openNextFile();
      while (file) {
        String fileName = String(file.name());
        // Clean filename formatting
        if (!fileName.startsWith("/")) {
          fileName = "/" + fileName;
        }
        if (fileName.endsWith(".wav")) {
          if (!first) json += ",";
          json += "\"" + fileName + "\"";
          first = false;
        }
        file.close(); // Close individual file handle
        file = root.openNextFile();
      }
      root.close(); // Explicitly close root directory handle
    }
    json += "]";
    server.send(200, "application/json", json);
  });

  server.on("/audio", HTTP_GET, [](){ 
    const bool download = server.hasArg("download") && server.arg("download") == "1";
    if (download && (recording || recordingTaskRunning || scanningAll)) {
      server.send(409, "text/plain", "Please wait until recording and saving finish, then download again.");
      return;
    }
    if (!server.hasArg("file")) {
      server.send(400, "text/plain", "Missing file parameter");
      return;
    }
    String filename = server.arg("file");
    if (!filename.startsWith("/")) filename = "/" + filename;

    if (!SD.exists(filename.c_str())) {
      server.send(404, "text/plain", "File not found");
      return;
    }

    File audioFile = SD.open(filename.c_str(), FILE_READ);
    if (!audioFile) {
      server.send(500, "text/plain", "Failed to open file");
      return;
    }

    if (download) {
      // Sanitize only the suggested download name; the SD file is unchanged.
      String downloadName = filename.substring(filename.lastIndexOf('/') + 1);
      for (unsigned int i = 0; i < downloadName.length(); i++) {
        const char c = downloadName[i];
        if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
              (c >= '0' && c <= '9') || c == '.' || c == '_' || c == '-' || c == ' ' || c == '(' || c == ')')) {
          downloadName.setCharAt(i, '_');
        }
      }
      if (!downloadName.endsWith(".wav")) downloadName += ".wav";
      server.sendHeader("Content-Disposition", "attachment; filename=\"" + downloadName + "\"");
      server.sendHeader("Cache-Control", "no-store");
    }
    server.streamFile(audioFile, "audio/wav");
    audioFile.close();
  });

  server.begin(); 
  Serial.println("Web server running at http://192.168.4.1");
}

void loop() { 
  server.handleClient(); 
  if (recordingFinished) { 
    recordingFinished = false; 
    Serial.printf("[AUDIO STATS] Min: %d | Max: %d | RMS: %u\n", recordedMin, recordedMax, rmsLevel); 
  } 
  delay(1); 
}
