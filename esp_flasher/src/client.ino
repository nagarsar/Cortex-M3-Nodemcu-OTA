#include <WiFiClient.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include "ESP8266FtpServer.h"

#define MAISON 0

#if MAISON
  const char* ssid     = "SFR_1B50";      // SSID
  const char* password = "dpg85ghew3qxq74bpjb3";      // Password
  IPAddress ip(192, 168, 1, 110);
  IPAddress gateway(192, 168, 1, 1);
  IPAddress subnet(255, 255, 255, 0);
#else
  const char* ssid     = "Automotive-Lighting";      // SSID
  const char* password = "Lightning-Control";      // Password
  IPAddress ip(192, 168, 2, 110);
  IPAddress gateway(192, 168, 2, 1);
  IPAddress subnet(255, 255, 255, 0);
#endif


/***********************************************\
|          PAREMETRER POUR CHAQUE MODULE        |
|            - L'ADRESSE IP                     |
|            - LE SSID                          |
|            - LE MOT DE PASSE                  |
\***********************************************/
// const char* ssid     = "MIX";       // SSID
// const char* password = "1234567890";// Password

// NETWORK: Static IP details...

const int   watchdog = 5000;        // Fréquence du watchdog - Watchdog frequency
unsigned long previousMillis = millis();
uint8_t nof=0;
uint8_t pnof=0;

FtpServer ftpSrv;
ESP8266WebServer server(80); //non - officiel

const int led = 16;

void handleNotFound(){
  digitalWrite(led, 1);
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET)?"GET":"POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i=0; i<server.args(); i++){
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
  digitalWrite(led, 0);
}


void spiffsNewFileFromFtp(){
  String str="";
  Dir dir = SPIFFS.openDir("/");
  while (dir.next() ) {
    str += dir.fileName();
    str += " / ";
    str += dir.fileSize();
    str += "\r\n";
    nof++;
  }
  //Serial.print(str);
  if ( nof != pnof ) {
    Serial.println("Liste des fichiers:");
    Serial.println(str);
  }
  pnof = nof;
  nof=0;
}


void televersement(String str_namefile, String state){
  if (state == "true"){

      str_namefile = "/"+ str_namefile;
      int lines_number = countlines(str_namefile);
      //Serial.println(str_namefile);
      //Serial.println(lines_number);

      // open file for reading
      File f = SPIFFS.open(str_namefile, "r");
      if (!f) {
          Serial.println("fichier non present");
      }

      //on lance le trigger
      Serial.println("TRIG");
      // On ajoute le nom de fichier avant meme d'envoyer le pgm
      Serial.println(str_namefile);

      //on attend 0.5 second
      delay(500);

      // flashage par port serie
      // Serial.println("============ Lecture du fichier ===========");
      for (int i=1; i<=lines_number; i++){
        String s=f.readStringUntil('\n');
        // Serial.print(i);
        // Serial.print(":");
        Serial.println(s);
        delay(5);
        if (i % 10 == 1) digitalWrite(led, !digitalRead(led));
      }
      Serial.print(":flash "); Serial.println(lines_number); // <- Validation si nombre de lignes correct
      //Serial.println("===========================================");
  }
}

int countlines(String filename)
{
  // Compte le nombre de lignes qu'il y a dans le fichier
  File f = SPIFFS.open(filename, "r");
  int ch=0;
  int lines=0;

  while ( f.available() ){
    if (char(f.read()) == '\n')
      lines++;
  }
  f.close();
  return lines;
}


void setup() {

  pinMode(led, OUTPUT);
  digitalWrite(led, 0);
  Serial.begin(115200);
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Static IP Setup Info Here...
  WiFi.config(ip, gateway, subnet);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");


  if (MDNS.begin("esp8266")) {
      Serial.println("MDNS responder started");
  }
  // connexion OK, on demarre le server web
  server.on("/control", [](){
    digitalWrite(led, 1);
    String message = "\nArguments: ";
    message += server.args();
    message += "\n";
    for (uint8_t i=0; i<server.args(); i++){
      message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
    }
    server.send(200, "text/plain", message);
    digitalWrite(led, 0);

    televersement(server.arg(0),server.arg(1));

    digitalWrite(led, 0);
  });

  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("Server started");

  Serial.println("Mon local IP address: ");
  Serial.println(WiFi.localIP());

  if (SPIFFS.begin()) {
     Serial.println("SPIFFS opened!");
     ftpSrv.begin("esp8266", "esp8266"); // username, password for ftp. Set ports in ESP8266FtpServer.h (default 21, 50009 for PASV)
  }
}



void loop() {
  //Serial.println(WiFi.localIP());
  spiffsNewFileFromFtp();
  ftpSrv.handleFTP();
  server.handleClient();
}
