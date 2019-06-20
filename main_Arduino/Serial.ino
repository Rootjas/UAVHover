//--------Receive dat with start- and endmarkers combined with parsing------//

const byte numChars = 32;
//
//
// variables to hold the parsed data
//char messageFromPC[numChars] = {0};
//int integerFromPC = 0;
//float floatFromPC[3];



void recvWithStartEndMarkers(bool &newData, char receivedChars[32], char tempChars[32]) {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;
  newData = false;
  
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();
    
    if(recvInProgress == true) {
      if (rc != endMarker){
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars)ndx= numChars - 1;
      }
      else{
        receivedChars[ndx] = '\0';  // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }
    else if (rc == startMarker)recvInProgress = true;
  }
}

void parseData(float &sp, float &Hoek_sp, int &event, char tempChars[32]) {                      //split the data into its parts    
  char * strtokIndx;                    //this is used by strtok() as an index

  strtokIndx = strtok(tempChars, ",");
  sp = atof(strtokIndx);    //dit is rotatie
  
  strtokIndx = strtok(tempChars, ",");      
  Hoek_sp = atof(strtokIndx);    //dit is afstand

  strtokIndx = strtok(NULL, ",");
  event = Round(atof(strtokIndx));    //is de switch event
}
