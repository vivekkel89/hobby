/*
  ESP32 AC Controller v3.5 (Humidity Priority Mode + Dual Sliders + 15m Fan Timeout)
  SHT31 + IR LED + LCD 16x2 (I2C) + web UI + history graph
  + outdoor temperature from the internet (Open-Meteo) used by the algorithm

  Libraries (Library Manager): Adafruit SHT31, IRremoteESP8266,
                               LiquidCrystal_I2C
  Wiring: SHT31 SDA->21, SCL->22 | IR emitter signal on GPIO 4
          16x2 I2C LCD (0x27) shares the same SDA->21, SCL->22, 5V/3V3, GND
  Open http://ac-controller.local/ from any device on your home WiFi.
*/
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Wire.h>
#include <LittleFS.h>
#include <ESPmDNS.h>
#include <time.h>
#include <math.h>
#include <Adafruit_SHT31.h>
#include <LiquidCrystal_I2C.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>

// ---------- CONFIG ----------
const char* WIFI_SSID = "Starlink_IOT";
const char* WIFI_PASS = "Thrissur@IOT";
const uint8_t IR_PIN = 4;

const long     GMT_OFFSET_SEC  = 19800;  // India (UTC+5:30)

// Outdoor weather (Kuriachira, Thrissur)
const char*    OUT_LAT         = "10.503";
const char*    OUT_LON         = "76.224";
const uint32_t WEATHER_MS      = 15UL * 60UL * 1000UL;
const uint32_t OUT_STALE_MS    = 90UL * 60UL * 1000UL;

// Cooling band
const float    ON_OFFSET_MILD  = 0.7;
const float    ON_OFFSET_HOT   = 0.3;
const float    OFF_OFFSET_MILD = 0.5;
const float    OFF_OFFSET_HOT  = 1.5;
const float    HEAT_RANGE      = 8.0;

const uint32_t MIN_CHANGE_MS   = 1UL * 60UL * 1000UL;  // 1 minute delay
const uint32_t RESEND_MS       = 10UL * 60UL * 1000UL; // 10 minutes periodic state re-send
const uint32_t LOG_MS          = 5UL * 60UL * 1000UL;
const uint32_t SAVE_MS         = 30UL * 60UL * 1000UL;
const float    TEMP_OFFSET     = 0.0;
const float    RH_OFFSET       = 0.0;

// ---------- IR CODES ----------
uint16_t RAW_COOL[] = {4488, 4382, 572, 1578, 572, 506, 578, 1570, 574, 1578, 570, 508, 570, 508, 574, 1576, 572, 508, 570, 506, 572, 1580, 572, 506, 578, 500, 572, 1580, 570, 1576, 574, 504, 574, 1578, 570, 508, 570, 1578, 572, 506, 572, 1578, 570, 1578, 570, 1578, 570, 1580, 570, 1578, 570, 1578, 572, 506, 572, 1578, 570, 508, 572, 506, 572, 508, 572, 508, 570, 508, 570, 1580, 570, 508, 570, 508, 572, 1580, 572, 506, 574, 508, 570, 506, 572, 508, 572, 508, 574, 1580, 570, 1576, 572, 508, 572, 1578, 572, 1576, 572, 1578, 574, 1574, 572, 5170, 4456, 4406, 548, 1600, 550, 528, 550, 1600, 550, 1600, 550, 530, 548, 530, 548, 1602, 550, 532, 546, 532, 546, 1602, 550, 528, 550, 534, 544, 1602, 548, 1602, 548, 530, 548, 1602, 546, 534, 546, 1602, 550, 534, 544, 1602, 548, 1600, 548, 1602, 550, 1602, 544, 1604, 546, 1602, 546, 532, 552, 1600, 544, 534, 550, 530, 548, 530, 546, 536, 544, 534, 546, 1602, 546, 532, 546, 536, 546, 1604, 546, 532, 546, 532, 546, 534, 546, 534, 544, 534, 544, 1606, 544, 1602, 546, 532, 546, 1604, 546, 1604, 546, 1604, 542, 1606, 544};
uint16_t RAW_DRY[]  = {4462, 4408, 546, 1602, 550, 530, 548, 1600, 552, 1598, 550, 528, 550, 530, 550, 1600, 548, 530, 548, 534, 548, 1598, 550, 530, 548, 534, 546, 1600, 550, 1598, 550, 530, 550, 1600, 548, 530, 550, 530, 548, 534, 548, 1598, 550, 1602, 546, 1600, 550, 1602, 546, 1600, 546, 1602, 548, 1602, 550, 1600, 546, 534, 546, 532, 546, 532, 546, 532, 550, 534, 546, 1600, 548, 532, 548, 534, 544, 1606, 544, 534, 544, 1606, 546, 532, 546, 532, 546, 532, 550, 1604, 544, 1602, 548, 534, 544, 1606, 544, 536, 544, 1604, 546, 1602, 546, 5192, 4432, 4408, 548, 1600, 548, 534, 546, 1604, 546, 1600, 548, 534, 546, 532, 546, 1606, 544, 534, 546, 530, 550, 1600, 548, 534, 546, 532, 546, 1606, 546, 1600, 548, 532, 544, 1606, 544, 534, 544, 536, 546, 530, 548, 1604, 546, 1602, 546, 1604, 546, 1604, 544, 1606, 544, 1604, 544, 1604, 544, 1604, 546, 532, 546, 532, 546, 534, 546, 534, 548, 532, 544, 1606, 544, 534, 544, 536, 544, 1606, 544, 534, 546, 1604, 546, 534, 546, 532, 546, 534, 548, 1602, 544, 1606, 544, 534, 544, 1604, 546, 534, 544, 1604, 546, 1604, 546};
uint16_t RAW_FAN[]  = {4482, 4386, 570, 1578, 568, 512, 570, 1582, 566, 1580, 570, 512, 566, 514, 566, 1584, 566, 514, 566, 512, 568, 1578, 568, 512, 568, 510, 570, 1582, 568, 1580, 568, 514, 566, 1580, 570, 1584, 564, 514, 566, 1582, 566, 1584, 568, 1580, 568, 1584, 564, 1584, 566, 1586, 564, 512, 566, 1582, 566, 512, 568, 512, 566, 512, 572, 508, 566, 512, 570, 510, 568, 1606, 544, 1584, 566, 1606, 546, 512, 568, 508, 568, 1608, 540, 512, 566, 538, 544, 508, 570, 510, 566, 536, 544, 1608, 544, 1606, 540, 538, 540, 1610, 540, 1608, 540, 5196, 4428, 4412, 544, 1606, 544, 534, 544, 1604, 546, 1604, 544, 536, 542, 540, 542, 1604, 544, 534, 544, 536, 544, 1606, 544, 538, 540, 538, 540, 1606, 546, 1604, 542, 536, 544, 1606, 544, 1606, 542, 538, 542, 1608, 540, 1608, 540, 1606, 542, 1610, 540, 1606, 542, 1608, 544, 534, 544, 1606, 542, 536, 546, 534, 546, 534, 542, 538, 540, 538, 544, 534, 544, 1606, 544, 1604, 544, 1604, 542, 536, 546, 534, 546, 1606, 540, 538, 544, 534, 544, 536, 542, 540, 538, 540, 544, 1604, 544, 1604, 540, 538, 546, 1604, 540, 1608, 544};
uint16_t RAW_OFF[]  = {4460, 4410, 546, 1600, 548, 534, 544, 1602, 548, 1604, 548, 528, 552, 532, 546, 1602, 546, 532, 546, 532, 548, 1602, 550, 530, 548, 530, 548, 1604, 544, 1602, 548, 530, 552, 1600, 546, 532, 546, 1606, 544, 1604, 546, 1604, 544, 1602, 550, 528, 550, 1600, 550, 1598, 552, 1598, 548, 530, 548, 530, 548, 534, 548, 530, 550, 1600, 548, 530, 550, 532, 546, 1600, 552, 1600, 546, 1602, 550, 530, 548, 530, 550, 530, 548, 532, 546, 534, 548, 528, 550, 528, 550, 532, 546, 1602, 548, 1604, 546, 1602, 550, 1602, 546, 1600, 550, 5190, 4430, 4410, 544, 1602, 548, 532, 546, 1606, 544, 1602, 546, 532, 550, 528, 550, 1600, 546, 532, 548, 530, 550, 1600, 546, 534, 550, 530, 546, 1602, 546, 1604, 546, 534, 546, 1602, 546, 534, 550, 1600, 544, 1604, 548, 1600, 550, 1600, 546, 532, 548, 1602, 550, 1600, 550, 1602, 544, 534, 546, 530, 550, 530, 550, 530, 548, 1600, 546, 536, 546, 532, 546, 1602, 548, 1600, 548, 1600, 548, 532, 546, 536, 542, 534, 548, 538, 540, 536, 546, 534, 546, 530, 546, 532, 550, 1598, 548, 1600, 550, 1600, 550, 1604, 544, 1602, 546};
uint16_t RAW_DISP[] = {4484, 4406, 548, 1602, 550, 532, 546, 1604, 546, 1600, 550, 530, 548, 1602, 548, 530, 548, 1604, 544, 534, 544, 1604, 546, 532, 546, 534, 546, 1602, 550, 534, 544, 1602, 548, 532, 546, 1604, 544, 1604, 546, 1602, 546, 1604, 550, 528, 546, 1604, 546, 534, 548, 1602, 544, 534, 544, 534, 546, 532, 550, 528, 550, 1600, 546, 532, 546, 1604, 548, 530, 550, 1600, 546, 534, 548, 1600, 546, 532, 552, 528, 550, 1600, 546, 534, 546, 1604, 544, 534, 548, 1600, 546, 534, 546, 1602, 550, 1600, 546, 532, 546, 1604, 550, 528, 546, 5196, 4428, 4412, 544, 1606, 546, 532, 546, 1604, 546, 1604, 544, 534, 544, 1606, 544, 536, 544, 1604, 544, 532, 548, 1604, 546, 532, 546, 532, 546, 1604, 544, 534, 544, 1606, 544, 534, 546, 1604, 544, 1602, 546, 1604, 546, 1602, 546, 534, 548, 1602, 546, 532, 550, 1600, 544, 534, 546, 532, 550, 532, 546, 532, 546, 1604, 544, 534, 546, 1604, 548, 530, 550, 1600, 546, 534, 548, 1600, 544, 534, 546, 532, 546, 1604, 546, 534, 546, 1604, 544, 534, 544, 1606, 544, 534, 550, 1600, 546, 1604, 546, 532, 550, 1600, 546, 534, 548};

enum AcState { ST_OFF, ST_COOL, ST_FAN, ST_DRY };
const char* stateName[] = {"OFF", "COOLING", "FAN", "DRY"};

// ---------- HISTORY ----------
const uint16_t HIST_N = 2016;
const int16_t  OUT_NONE = -32768;
struct Rec { uint32_t ts; int16_t t; int16_t h; int16_t o; uint8_t st; uint8_t pad; };
struct Meta { uint16_t magic; uint16_t head; uint16_t count; };
const uint16_t HIST_MAGIC = 0xAC58;
Rec hist[HIST_N];
uint16_t histHead = 0, histCount = 0;
bool histDirty = false;

Adafruit_SHT31 sht31;
LiquidCrystal_I2C lcd(0x27, 16, 2);
bool lcdOk = false;
IRsend irsend(IR_PIN);
WebServer server(80);
Preferences prefs;

float curT = NAN, curH = NAN;
volatile float outT = NAN;
volatile uint32_t outMs = 0;
float targetT = 27.0;
bool  autoMode = true;
bool  useFeels = true;
bool  useOutdoor = true;
bool  mockSensor = true;
bool  humidPriority = false;
float targetHighH = 80.0;
float targetLowH = 70.0;

AcState acState = ST_OFF;
uint32_t lastChange = 0, lastSend = 0, lastRead = 0;
uint32_t lastLog = 0, lastSave = 0, lastWifi = 0, lastDisp = 0;
uint32_t fanStartTime = 0;
bool firstRun = true;

// Single-line HTML string including Humidity Priority controls and sliders
const char PAGE[] PROGMEM = "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'><meta name='theme-color' content='#111111'><link rel='manifest' href='/manifest.json'><title>AC Controller</title><style>body{font-family:system-ui;margin:0;padding:16px;background:#111;color:#eee;max-width:460px;margin:auto}.card{background:#1d1d1d;border-radius:14px;padding:16px;margin-bottom:14px}.big{font-size:34px;font-weight:600}.row{display:flex;justify-content:space-around;text-align:center}.lbl{color:#999;font-size:13px}input[type=range]{width:100%}button{padding:9px 13px;border:0;border-radius:10px;background:#333;color:#eee;font-size:15px;margin:3px;cursor:pointer}button.on{background:#2a7}.st{font-size:20px;text-align:center}canvas{width:100%;height:200px;display:block;margin-top:8px}.lg span{margin-right:10px;font-size:12px;color:#999}.lg i{display:inline-block;width:10px;height:10px;border-radius:2px;margin-right:4px}</style></head><body><div class='card'><div class='row'><div><div class='big' id='t'>28.5</div><div class='lbl'>Temp &deg;C</div></div><div><div class='big' id='f'>29.2</div><div class='lbl'>Feels like</div></div><div><div class='big' id='h'>65</div><div class='lbl'>Humidity %</div></div></div><div class='lbl' style='text-align:center;margin-top:12px'>Outside (Kuriachira): <b id='o' style='color:#eee;font-size:18px'>--</b> &deg;C <span id='hint' style='color:#EF9F27'></span></div><div class='st' style='margin-top:10px'>AC: <b id='s'>OFF</b></div></div><div class='card'><div class='lbl'><span id='tl'>Target</span>: <b id='tv'>27.0</b> &deg;C</div><input type='range' id='tt' min='20' max='40' step='0.5' value='27.0' oninput='window.editing=1;document.getElementById(\"tv\").textContent=this.value' onchange='window.saveSettings()'><div style='margin-top:12px'><label><input type='checkbox' id='au' checked onchange='window.saveSettings()'> Auto control</label></div><div style='margin-top:6px'><label><input type='checkbox' id='hp' onchange='window.saveSettings()'> Humidity Priority (Ignores Temp)</label></div><div style='margin-top:6px'><label><input type='checkbox' id='fe' checked onchange='window.saveSettings()'> Use feels-like temperature</label></div><div style='margin-top:6px'><label><input type='checkbox' id='ue' checked onchange='window.saveSettings()'> Adapt to outdoor temperature</label></div><div style='margin-top:6px'><label><input type='checkbox' id='mo' checked onchange='window.saveSettings()'> Mock sensor (Test mode)</label></div><div style='margin-top:12px' class='lbl'>High Humidity (Dry trigger): <b id='thhv'>80</b>%</div><input type='range' id='thh' min='50' max='95' step='1' value='80' oninput='document.getElementById(\"thhv\").textContent=this.value' onchange='window.saveSettings()'><div style='margin-top:8px' class='lbl'>Low Humidity (Fan target): <b id='tlhv'>70</b>%</div><input type='range' id='tlh' min='40' max='90' step='1' value='70' oninput='document.getElementById(\"tlhv\").textContent=this.value' onchange='window.saveSettings()'></div><div class='card'><div class='lbl'>Manual Control (turns Auto off)</div><button onclick=\"window.sendManual('cool')\">Cool</button><button onclick=\"window.sendManual('fan')\">Fan</button><button onclick=\"window.sendManual('dry')\">Dry</button><button onclick=\"window.sendManual('off')\">Off</button></div><div class='card'><div class='lbl'>Actions (Keeps Auto active)</div><button onclick=\"window.sendDisplay()\" style=\"background:#2563eb\">Display Toggle</button></div><div class='card'><div style='display:flex;justify-content:space-between;align-items:center'><b>History</b><div><button class='rg' data-h='6' onclick='window.loadHist(6)'>6h</button><button class='rg on' data-h='24' onclick='window.loadHist(24)'>24h</button><button class='rg' data-h='168' onclick='window.loadHist(168)'>7d</button></div></div><canvas id='c'></canvas><div class='lg' style='margin-top:6px'><span><i style='background:#EF9F27'></i>Temp</span><span><i style='background:#D85A30'></i>Feels like</span><span><i style='background:#378ADD'></i>Humidity</span><span><i style='background:#97A3AB'></i>Outside</span><br><span><i style='background:rgba(55,138,221,.4)'></i>Cooling</span><span><i style='background:rgba(50,205,50,.8)'></i>Fan mode</span><span><i style='background:rgba(29,158,117,.4)'></i>Dry mode</span></div><div style='margin-top:8px'><label class='lbl'><input type='checkbox' id='so' onchange='if(window.lastD)window.drawGraph(window.lastD)'> Show outside</label></div></div><div class='lbl' style='text-align:center;margin-bottom:10px'>Outdoor data: <a href='https://open-meteo.com/' style='color:#999'>Open-Meteo.com</a></div><script>window.editing=0;window.hrs=24;window.lastD=null;function computeFeels(t,h){return t+0.33*(h/100*6.105*Math.exp(17.27*t/(237.7+t)))-4;}window.pollStatus=async function(){try{const res=await fetch('/api/status');const r=await res.json();document.getElementById('t').textContent=r.t.toFixed(1);document.getElementById('f').textContent=r.f.toFixed(1);document.getElementById('h').textContent=r.h.toFixed(0);document.getElementById('s').textContent=r.state;document.getElementById('o').textContent=r.o==null?'--':r.o.toFixed(1);document.getElementById('hint').textContent=r.hint?'· cooler outside, open window':'';document.getElementById('tl').textContent=r.feels?'Target feels-like':'Target temperature';if(!window.editing){document.getElementById('tt').value=r.tt;document.getElementById('au').checked=r.auto;document.getElementById('hp').checked=r.hp;document.getElementById('fe').checked=r.feels;document.getElementById('ue').checked=r.uo;document.getElementById('mo').checked=r.mock;document.getElementById('thh').value=r.thh;document.getElementById('thhv').textContent=r.thh;document.getElementById('tlh').value=r.tlh;document.getElementById('tlhv').textContent=r.tlh;document.getElementById('tv').textContent=r.tt;}}catch(e){console.error('Poll error:',e);}};window.saveSettings=async function(){console.log('Saving settings to ESP32...');try{const tt=document.getElementById('tt').value;const au=document.getElementById('au').checked?1:0;const hp=document.getElementById('hp').checked?1:0;const fe=document.getElementById('fe').checked?1:0;const ue=document.getElementById('ue').checked?1:0;const mo=document.getElementById('mo').checked?1:0;const thh=document.getElementById('thh').value;const tlh=document.getElementById('tlh').value;await fetch(`/api/set?temp=${tt}&auto=${au}&hp=${hp}&feels=${fe}&out=${ue}&mock=${mo}&thh=${thh}&tlh=${tlh}`);window.editing=0;window.pollStatus();}catch(e){console.error('Save error:',e);}};window.sendManual=async function(m){console.log('Manual mode trigger:',m);try{await fetch('/api/manual?m='+m);window.pollStatus();}catch(e){console.error('Manual error:',e);}};window.sendDisplay=async function(){console.log('Display toggle trigger');try{await fetch('/api/display');window.pollStatus();}catch(e){console.error('Display error:',e);}};window.loadHist=async function(n){window.hrs=n;document.querySelectorAll('.rg').forEach(b=>b.classList.toggle('on',b.dataset.h==n));try{const res=await fetch('/api/history?hours='+n);const d=await res.json();window.lastD=d;window.drawGraph(d);}catch(e){console.error('History error:',e);}};window.drawGraph=function(d){const c=document.getElementById('c');if(!c)return;const dp=window.devicePixelRatio||1;const W=c.clientWidth||300;const H=200;c.width=W*dp;c.height=H*dp;const g=c.getContext('2d');g.scale(dp,dp);const P=d.p;g.font='11px system-ui';g.fillStyle='#999';if(!P||P.length<2){g.fillText('Collecting data... (one point every 5 min)',8,24);return;}const t0=P[0][0],t1=P[P.length-1][0],L=35,R=30,T=10,B=20,w=W-L-R,h=H-T-B;const so=document.getElementById('so').checked;const ov=P.map(p=>p[4]==null?null:p[4]/10);const tv=P.map(p=>p[1]/10);const hv=P.map(p=>p[2]/10);const fv=P.map((p,i)=>computeFeels(tv[i],hv[i]));const all=tv.concat(fv,so?ov.filter(v=>v!=null):[]);const lo=Math.floor(Math.min(...all)-1);const hi=Math.ceil(Math.max(...all)+1);const X=t=>L+(t-t0)/(t1-t0)*w;const Y=v=>T+h-(v-lo)/(hi-lo)*h;g.strokeStyle='rgba(255,255,255,0.08)';g.lineWidth=1;g.fillStyle='#777';const stepV=Math.max(1,Math.round((hi-lo)/4));for(let v=Math.ceil(lo);v<=hi;v+=stepV){const py=Y(v);g.beginPath();g.setLineDash([3,3]);g.moveTo(L,py);g.lineTo(W-R,py);g.stroke();g.setLineDash([]);g.fillText(v+'\\u00b0C',2,py+3);}for(let i=0;i<P.length-1;i++){const s=P[i][3];if(s&&P[i+1][0]-P[i][0]<=d.step*3){if(s==1)g.fillStyle='rgba(55,138,221,.22)';else if(s==2)g.fillStyle='rgba(50,205,50,.8)';else if(s==3)g.fillStyle='rgba(29,158,117,.22)';else g.fillStyle='rgba(255,255,255,.05)';g.fillRect(X(P[i][0]),T,Math.max(1,X(P[i+1][0])-X(P[i][0])),h);}}const ln=(v,Yf,col,dash)=>{g.beginPath();g.strokeStyle=col;g.lineWidth=2;g.setLineDash(dash||[]);let pen=0;for(let i=0;i<P.length;i++){if(v[i]==null){pen=0;continue;}if(i&&P[i][0]-P[i-1][0]>d.step*3)pen=0;const px=X(P[i][0]),py=Yf(v[i]);if(pen)g.lineTo(px,py);else{g.moveTo(px,py);pen=1;}}g.stroke();g.setLineDash([]);};ln(hv,Y,'#378ADD');if(so)ln(ov,Y,'#97A3AB',[5,3]);ln(fv,Y,'#D85A30');ln(tv,Y,'#EF9F27');const f=t=>{const D=new Date(t*1000);return(window.hrs>24?D.toLocaleDateString([],{day:'numeric',month:'short'})+' ':'')+D.toLocaleTimeString([],{hour:'2-digit',minute:'2-digit'});};g.textAlign='left';g.fillText(f(t0),L,H-4);g.textAlign='right';g.fillText(f(t1),W-R,H-4);};window.pollStatus();window.loadHist(24);setInterval(window.pollStatus,3000);setInterval(()=>window.loadHist(window.hrs),60000);</script></body></html>";

// ---------- HELPERS ----------
float feelsLike(float t, float rh) {
  float e = rh / 100.0f * 6.105f * expf(17.27f * t / (237.7f + t));
  return t + 0.33f * e - 4.0f;
}

float controlTemp() { return useFeels ? feelsLike(curT, curH) : curT; }

bool outdoorValid() {
  uint32_t m = outMs;
  return m != 0 && (millis() - m) < OUT_STALE_MS && !isnan((float)outT);
}

float heatLoad() {
  if (!useOutdoor || !outdoorValid()) return 0.5f;
  float k = ((float)outT - targetT) / HEAT_RANGE;
  return k < 0 ? 0 : (k > 1 ? 1 : k);
}

bool windowHint() {
  return outdoorValid() && !isnan(curT) && !isnan(curH) &&
         (float)outT <= curT - 2.0f && controlTemp() > targetT;
}

time_t nowTs() {
  time_t t = time(nullptr);
  return t > 1700000000 ? t : 0;
}

void sendRaw(uint16_t* buf, uint16_t len) {
  if (len <= 1) { Serial.println("[IR] Code not set, skipping transmission"); return; }
  irsend.sendRaw(buf, len, 38);
}

void applyState(AcState s, bool force = false) {
  if (!force && s == acState) return;
  acState = s;
  if (s == ST_COOL) sendRaw(RAW_COOL, sizeof(RAW_COOL) / 2);
  else if (s == ST_FAN) sendRaw(RAW_FAN, sizeof(RAW_FAN) / 2);
  else if (s == ST_DRY) sendRaw(RAW_DRY, sizeof(RAW_DRY) / 2);
  else sendRaw(RAW_OFF, sizeof(RAW_OFF) / 2);
  lastChange = lastSend = millis();
  Serial.printf("[CONTROL] AC State Changed -> %s\n", stateName[s]);
}

bool fetchOutdoor() {
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  String url = String("https://api.open-meteo.com/v1/forecast?latitude=") + OUT_LAT +
               "&longitude=" + OUT_LON + "&current=temperature_2m";
  if (!http.begin(client, url)) return false;
  http.setTimeout(8000);
  http.setConnectTimeout(8000);
  bool ok = false;
  int code = http.GET();
  if (code == 200) {
    String body = http.getString();
    int c = body.indexOf("\"current\":{");
    if (c >= 0) {
      const char* key = "\"temperature_2m\":";
      int p = body.indexOf(key, c);
      if (p >= 0) {
        float v = body.substring(p + strlen(key)).toFloat();
        if (v > -20.0f && v < 60.0f) { outT = v; outMs = millis(); ok = true; }
      }
    }
  }
  http.end();
  return ok;
}

void weatherTask(void*) {
  for (;;) {
    uint32_t waitMs = 10000;
    if (WiFi.status() == WL_CONNECTED)
      waitMs = fetchOutdoor() ? WEATHER_MS : 60000;
    vTaskDelay(pdMS_TO_TICKS(waitMs));
  }
}

void logPoint() {
  time_t ts = nowTs();
  if (!ts || isnan(curT) || isnan(curH)) return;
  Rec &r = hist[histHead];
  r.ts = (uint32_t)ts;
  r.t = (int16_t)lroundf(curT * 10.0f);
  r.h = (int16_t)lroundf(curH * 10.0f);
  r.o = outdoorValid() ? (int16_t)lroundf((float)outT * 10.0f) : OUT_NONE;
  r.st = (uint8_t)acState;
  histHead = (histHead + 1) % HIST_N;
  if (histCount < HIST_N) histCount++;
  histDirty = true;
}

void saveHistory() {
  if (!histDirty) return;
  File f = LittleFS.open("/hist.bin", "w");
  if (!f) return;
  Meta m = { HIST_MAGIC, histHead, histCount };
  f.write((const uint8_t*)&m, sizeof(m));
  f.write((const uint8_t*)hist, sizeof(hist));
  f.close();
  histDirty = false;
}

void loadHistory() {
  File f = LittleFS.open("/hist.bin", "r");
  if (!f) return;
  Meta m;
  if (f.read((uint8_t*)&m, sizeof(m)) == sizeof(m) &&
      m.magic == HIST_MAGIC && m.head < HIST_N && m.count <= HIST_N) {
    if (f.read((uint8_t*)hist, sizeof(hist)) == sizeof(hist)) {
      histHead = m.head;
      histCount = m.count;
    }
  }
  f.close();
}

void control() {
  if (isnan(curT) || isnan(curH)) return;
  uint32_t now = millis();

  if (!autoMode) {
    if (now - lastSend > RESEND_MS) {
      applyState(acState, true); // Periodic 10-minute refresh in manual mode
    }
    return;
  }

  AcState want = acState;

  if (humidPriority) {
    // --- HUMIDITY PRIORITY MODE ---
    // Ignore temperature entirely. Use High & Low humidity sliders with hysteresis.
    if (acState != ST_DRY && curH > targetHighH) {
      want = ST_DRY;
    } else if (acState == ST_DRY) {
      if (curH <= targetLowH) {
        want = ST_FAN; // Humidity controlled, switch to FAN
      }
    } else {
      // If not in DRY, check if humidity is already low or acceptable
      if (curH <= targetLowH && acState != ST_OFF) {
        want = ST_FAN;
      }
    }
  } else {
    // --- STANDARD TEMPERATURE / FEELS-LIKE AUTO CONTROL ---
    float ct = controlTemp();
    float k = heatLoad();
    float onOff  = ON_OFFSET_MILD  + (ON_OFFSET_HOT  - ON_OFFSET_MILD)  * k;
    float offOff = OFF_OFFSET_MILD + (OFF_OFFSET_HOT - OFF_OFFSET_MILD) * k;

    if (ct >= targetT + onOff) {
      want = ST_COOL;
    } else if (ct <= targetT - offOff) {
      want = ST_FAN;
    }
  }

  // Common 15-minute Fan Timeout Rule (Transitions FAN -> OFF)
  if (want == ST_FAN) {
    if (acState != ST_FAN) {
      fanStartTime = now;
    } else if (now - fanStartTime >= 15UL * 60UL * 1000UL) {
      want = ST_OFF;
    }
  } else {
    fanStartTime = now;
  }

  if (firstRun) { firstRun = false; lastChange = now - MIN_CHANGE_MS; }

  // Apply state changes with 1-minute minimum delay, or periodic 10-minute resend heartbeat
  if (want != acState && now - lastChange >= MIN_CHANGE_MS) {
    applyState(want);
  } else if (want == acState && now - lastSend > RESEND_MS) {
    applyState(acState, true); // Periodic 10-minute state refresh heartbeat
  }
}

void handleStatus() {
  char buf[450], ob[12];
  float t = isnan(curT) ? 0 : curT, h = isnan(curH) ? 0 : curH;
  if (outdoorValid()) snprintf(ob, sizeof(ob), "%.1f", (float)outT);
  else strcpy(ob, "null");
  snprintf(buf, sizeof(buf),
    "{\"t\":%.2f,\"h\":%.1f,\"f\":%.2f,\"o\":%s,\"tt\":%.1f,\"auto\":%s,\"hp\":%s,\"feels\":%s,\"uo\":%s,\"mock\":%s,\"thh\":%.0f,\"tlh\":%.0f,\"hint\":%s,\"state\":\"%s\"}",
    t, h, feelsLike(t, h), ob, targetT,
    autoMode ? "true" : "false", humidPriority ? "true" : "false",
    useFeels ? "true" : "false", useOutdoor ? "true" : "false", mockSensor ? "true" : "false",
    targetHighH, targetLowH,
    windowHint() ? "true" : "false", stateName[acState]);
  server.send(200, "application/json", buf);
}

void handleSet() {
  if (server.hasArg("temp")) targetT = constrain(server.arg("temp").toFloat(), 20, 40);
  if (server.hasArg("auto")) autoMode = server.arg("auto") == "1";
  if (server.hasArg("hp"))   humidPriority = server.arg("hp") == "1";
  if (server.hasArg("feels")) useFeels = server.arg("feels") == "1";
  if (server.hasArg("out"))  useOutdoor = server.arg("out") == "1";
  if (server.hasArg("mock")) mockSensor = server.arg("mock") == "1";
  if (server.hasArg("thh"))  targetHighH = constrain(server.arg("thh").toFloat(), 50, 95);
  if (server.hasArg("tlh"))  targetLowH = constrain(server.arg("tlh").toFloat(), 40, 90);
  
  prefs.putFloat("tt", targetT); 
  prefs.putBool("auto", autoMode); 
  prefs.putBool("hp", humidPriority);
  prefs.putBool("feels", useFeels);
  prefs.putBool("out", useOutdoor);
  prefs.putBool("mock", mockSensor);
  prefs.putFloat("thh", targetHighH);
  prefs.putFloat("tlh", targetLowH);
  
  server.send(200, "text/plain", "ok");
}

void handleManual() {
  String m = server.arg("m");
  autoMode = false; 
  prefs.putBool("auto", false);
  if (m == "cool") applyState(ST_COOL, true);
  else if (m == "fan") applyState(ST_FAN, true);
  else if (m == "dry") applyState(ST_DRY, true);
  else applyState(ST_OFF, true);
  server.send(200, "text/plain", "ok");
}

void handleDisplayToggle() {
  sendRaw(RAW_DISP, sizeof(RAW_DISP) / 2);
  Serial.println("[CONTROL] AC Display Toggle Signal Sent (Auto mode preserved)");
  server.send(200, "text/plain", "ok");
}

void handleHistory() {
  int hours = server.hasArg("hours") ? server.arg("hours").toInt() : 24;
  hours = constrain(hours, 1, 168);
  time_t now = nowTs();
  uint32_t from = now ? (uint32_t)now - (uint32_t)hours * 3600UL : 0;

  uint16_t n = 0;
  for (uint16_t i = 0; i < histCount; i++) {
    const Rec &r = hist[(histHead + HIST_N - histCount + i) % HIST_N];
    if (r.ts >= from) n++;
  }
  uint16_t stride = (n + 399) / 400;
  if (stride < 1) stride = 1;

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");
  String chunk;
  chunk.reserve(1200);
  chunk = "{\"step\":" + String(300UL * stride) + ",\"p\":[";
  bool first = true;
  uint16_t k = 0;
  for (uint16_t i = 0; i < histCount; i++) {
    const Rec &r = hist[(histHead + HIST_N - histCount + i) % HIST_N];
    if (r.ts < from) continue;
    if ((k++ % stride) != 0) continue;
    char os[8], b[72];
    if (r.o == OUT_NONE) strcpy(os, "null"); else snprintf(os, sizeof(os), "%d", r.o);
    snprintf(b, sizeof(b), "%s[%lu,%d,%d,%d,%s]", first ? "" : ",",
             (unsigned long)r.ts, r.t, r.h, r.st, os);
    first = false;
    chunk += b;
    if (chunk.length() > 1000) { server.sendContent(chunk); chunk = ""; }
  }
  chunk += "]}";
  server.sendContent(chunk);
  server.sendContent("");
}

void handleManifest() {
  server.send(200, "application/manifest+json", R"JSON({
    "name": "AC Controller",
    "short_name": "AC Control",
    "start_url": "/",
    "display": "standalone",
    "background_color": "#111111",
    "theme_color": "#111111",
    "icons": [
      {
        "src": "data:image/svg+xml,<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 100 100'><rect width='100' height='100' rx='20' fill='%232a7'/><text x='50' y='68' font-size='55' text-anchor='middle' fill='white'>❄️</text></svg>",
        "sizes": "192x192",
        "type": "image/svg+xml"
      }
    ]
  })JSON");
}

void updateDisplay() {
  if (!lcdOk) return;
  char row1[17], row2[17];
  float fT = isnan(curT) ? 0.0f : curT;
  int iH   = isnan(curH) ? 0 : (int)lroundf(curH);

  // Get current time from NTP sync
  time_t now = nowTs();
  struct tm timeinfo;
  char timeStr[6] = "--:--";
  char dateStr[7] = "--/--";
  
  if (now > 0) {
    localtime_r(&now, &timeinfo);
    strftime(timeStr, sizeof(timeStr), "%H:%M", &timeinfo);
    strftime(dateStr, sizeof(dateStr), "%d-%b", &timeinfo);
  }

  // Shorten state names to fit cleanly (COOLING -> COOL)
  const char* shortState = "OFF";
  if (acState == ST_COOL) shortState = "COOL";
  else if (acState == ST_FAN) shortState = "FAN";
  else if (acState == ST_DRY) shortState = "DRY";

  // Row 1: Temp, Humidity, Time (Max 16 chars: "T28.5 H74% 14:35")
  snprintf(row1, sizeof(row1), "T%.1f H%%d %s", fT, iH, timeStr); // format cleanly
  snprintf(row1, sizeof(row1), "T%.1f H%d%% %s", fT, iH, timeStr);

  // Row 2: AC State & Short Date (Max 16 chars: "AC: COOL | 28-May")
  snprintf(row2, sizeof(row2), "AC: %-4s | %s", shortState, dateStr);

  // Safe I2C transmission check
  Wire.beginTransmission(0x27);
  if (Wire.endTransmission() == 0) {
    lcd.setCursor(0, 0);
    lcd.print(row1);
    lcd.setCursor(0, 1);
    lcd.print(row2);
  } else {
    lcdOk = false; // Mark inactive if unplugged at runtime
  }
}

void enhancedSerialLog() {
  time_t ts = nowTs();
  if (!ts) {
    Serial.println("[LOG] Timestamp not yet synchronized via NTP");
    return;
  }

  // Convert Unix timestamp to local broken-down time structure
  struct tm timeinfo;
  localtime_r(&ts, &timeinfo);
  
  // Format into a readable string (e.g., "2026-05-18 14:35:48")
  char timeStr[25];
  strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);

  char outTempStr[8];
  if (outdoorValid()) {
    snprintf(outTempStr, sizeof(outTempStr), "%.1f", (float)outT);
  } else {
    strcpy(outTempStr, "N/A");
  }

  Serial.printf("[LOG] %s | LocalT:%.1f, FeelsT:%.1f, RH:%.0f%%, OutdoorT:%s, ACState:%s, Mode:%s\n",
                timeStr,
                isnan(curT) ? 0.0f : curT,
                controlTemp(),
                isnan(curH) ? 0.0f : curH,
                outTempStr,
                stateName[acState],
                autoMode ? "Auto" : "Manual");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  irsend.begin();
  prefs.begin("ac", false);
  targetT       = prefs.getFloat("tt", 27.0);
  autoMode      = prefs.getBool("auto", true);
  humidPriority = prefs.getBool("hp", false);
  useFeels      = prefs.getBool("feels", true);
  useOutdoor    = prefs.getBool("out", true);
  mockSensor    = prefs.getBool("mock", true);
  targetHighH   = prefs.getFloat("thh", 80.0);
  targetLowH    = prefs.getFloat("tlh", 70.0);

  if (!LittleFS.begin(true)) {
    Serial.println("[SETUP] LittleFS mount failed");
  } else {
    loadHistory();
  }

  Wire.begin(21, 22);
  Wire.setClock(100000);
  delay(100);
  sht31.begin(0x44);

  Wire.beginTransmission(0x27);
  if (Wire.endTransmission() == 0) {
    lcd.init();
    lcd.backlight();
    lcdOk = true;
    lcd.setCursor(0, 0);
    lcd.print("AC Controller v3");
    lcd.setCursor(0, 1);
    lcd.print("Booting up...");
  } else {
    lcdOk = false;
    Serial.println("[SETUP] LCD not detected on I2C address 0x27 (Running headless)");
  }

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) {
    delay(500); 
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[SETUP] WiFi Connected! IP Address: "); 
    Serial.println(WiFi.localIP());
  }

  if (MDNS.begin("ac-controller")) {
    Serial.println("[SETUP] mDNS responder started: http://ac-controller.local");
  }

  configTime(GMT_OFFSET_SEC, 0, "pool.ntp.org", "time.google.com");

  server.on("/", []() { server.send_P(200, "text/html", PAGE); });
  server.on("/manifest.json", handleManifest);
  server.on("/api/status", handleStatus);
  server.on("/api/set", handleSet);
  server.on("/api/manual", handleManual);
  server.on("/api/display", handleDisplayToggle);
  server.on("/api/history", handleHistory);
  server.begin();

  xTaskCreatePinnedToCore(weatherTask, "weather", 12288, NULL, 1, NULL, 0);
}

void loop() {
  server.handleClient();
  uint32_t now = millis();

  if (WiFi.status() != WL_CONNECTED && now - lastWifi > 30000) {
    lastWifi = now;
    WiFi.reconnect();
  }
  
  if (now - lastRead >= 5000) {
    lastRead = now;
    float t = sht31.readTemperature(), h = sht31.readHumidity();
    if (!isnan(t) && !isnan(h)) { 
      curT = t + TEMP_OFFSET; 
      curH = h + RH_OFFSET; 
    } else if (mockSensor) {
      curT = 28.5 + TEMP_OFFSET;
      curH = 65.0 + RH_OFFSET;
    } else {
      curT = NAN;
      curH = NAN;
    }
    control();
  }
  
  if (now - lastLog >= LOG_MS)   { 
    lastLog = now;  
    logPoint(); 
    enhancedSerialLog(); 
  }
  if (now - lastSave >= SAVE_MS) { lastSave = now; saveHistory(); }
  if (now - lastDisp >= 1000)    { lastDisp = now; updateDisplay(); }
}
