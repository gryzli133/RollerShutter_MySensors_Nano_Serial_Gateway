// Enable debug prints to serial monitor
#define MY_DEBUG

// uncomment if using as standalone Serial gateway
//#define MY_GATEWAY_SERIAL

// Enable and select radio type attached
#define MY_RADIO_NRF24

#define MY_TRANSPORT_WAIT_READY_MS 1

//#define MY_RF24_PA_LEVEL RF24_PA_LOW

//#define MY_REPEATER_FEATURE

// uncomment if we want to manually assign an ID
#define MY_NODE_ID 20

// only for Arduino Mega !!!!!!!!!!!!!!
#define MY_RF24_CE_PIN 49
#define MY_RF24_CS_PIN 53
// ------------------------------------

#include <Bounce2.h>
#include <MySensors.h>
#include <SPI.h>

//new begin-----------------------------

enum CoverState 
{
  STOP,
  UP,                             // Window covering. Up.
  DOWN,                           // Window covering. Down.
};
#define DIRECTION_UP 1
#define DIRECTION_DOWN 0
#define STATE_UP 100              // 100 is opened - up
#define STATE_DOWN 0              // 0 is closed - down
#define PRESENT_MESSAGE "Roller Shutter for Domoticz"
bool IS_ACK = false; //is to acknowlage

class RollerShutter
{ 
  uint8_t CHILD_ID_COVER;
  uint8_t CHILD_ID_SET;
  int buttonPinUp;
  int buttonPinDown;
  int relayPinUp;
  int relayPinDown;
  bool relayON;
  bool relayOFF;

  static bool initial_state_sent;
  int value = 0;
  int oldValueUp = 0;
  int oldValueDown = 0;

  Bounce debouncerUp = Bounce();  
  Bounce debouncerDown = Bounce();
  Bounce debouncerStop = Bounce();

  
  int rollTime = loadState(CHILD_ID_SET);

  
  float timeOneLevel = rollTime / 100.0;
  float requestedShutterLevel = 0;
  float currentShutterLevel = 0;
  unsigned long lastLevelTime = 0;
  bool isMoving = false;
  int directionUpDown;
  bool calibrateDown;
  bool calibrateUp;
  unsigned long calibrationStartTime;
  float calibrationTime = 0.0;
  bool calibratedDown;
  bool calibratedUp;
  int coverState = STOP;
  
  public:
  RollerShutter(int childId, int setId, int buttonUp, int buttonDown, int relayUp, int relayDown, int debaunceTime, bool invertedRelay) : 
                                           msgUp(childId, V_UP),msgDown(childId, V_DOWN),
                                           msgStop(childId, V_STOP), msgPercentage(childId, V_PERCENTAGE), 
                                           msgSetpoint(setId, V_HVAC_SETPOINT_HEAT), msgActual(setId, V_TEMP)
  {
                                          // constructor - like "setup" part of standard program
    CHILD_ID_COVER = childId;
    CHILD_ID_SET = setId;
    buttonPinUp = buttonUp;
    buttonPinDown = buttonDown;
    relayPinUp = relayUp;
    relayPinDown = relayDown;    
    relayON = !invertedRelay;
    relayOFF = invertedRelay;
    pinMode(buttonPinUp, INPUT_PULLUP);     // Setup the button
    pinMode(buttonPinDown, INPUT_PULLUP);   // Activate internal pull-up
    debouncerUp.attach(buttonPinUp);        // After setting up the button, setup debouncer
    debouncerDown.attach(buttonPinDown);    // After setting up the button, setup debouncer
    debouncerUp.interval(debaunceTime);     // After setting up the button, setup debouncer
    debouncerDown.interval(debaunceTime);   // After setting up the button, setup debouncer 
    digitalWrite(relayPinUp, relayOFF);     // Make sure relays are off when starting up
    digitalWrite(relayPinDown, relayOFF);   // Make sure relays are off when starting up
    pinMode(relayPinUp, OUTPUT);            // Then set relay pins in output mode
    pinMode(relayPinDown, OUTPUT);          // Then set relay pins in output mode


    int state = loadState(CHILD_ID_COVER);
    currentShutterLevel = state;
    requestedShutterLevel = state;
  }  

  MyMessage msgUp;
  MyMessage msgDown;
  MyMessage msgStop;
  MyMessage msgPercentage;
  MyMessage msgSetpoint;
  MyMessage msgActual;  

  void sendState()                          // Send current state and status to gateway.
  {  
    if(currentShutterLevel==100)
    {
      send(msgUp.set(1));
    }
    else 
    {
      if(currentShutterLevel==0)
      {
        send(msgDown.set(1));
      }
      else
      {
        send(msgStop.set(1));
      }
    }
    send(msgPercentage.set((int)currentShutterLevel));
    send(msgSetpoint.set((int)rollTime));
    send(msgActual.set((int)rollTime));
  } 
  
  void shuttersUp(void) 
  {
    #ifdef MY_DEBUG
      Serial.println("Shutters going up");
    #endif
    if (digitalRead(relayPinDown) == relayON) {
      digitalWrite(relayPinDown, relayOFF);
      wait(50);                             // NEED TO BE CHANGED !!!!!!!!!!!!
    }
    digitalWrite(relayPinUp, relayON);
  
    directionUpDown = DIRECTION_UP;
    isMoving = true;
    coverState = UP;
    sendState();
  }

  void shuttersDown(void) 
  {
    #ifdef MY_DEBUG
      Serial.println("Shutters going down");
    #endif
    if (digitalRead(relayPinUp) == relayON) {
      digitalWrite(relayPinUp, relayOFF);
      wait(50);                             // NEED TO BE CHANGED !!!!!!!!!!!!
    }
    digitalWrite(relayPinDown, relayON);
  
    directionUpDown = DIRECTION_DOWN;
    isMoving = true;
    coverState = DOWN;
    sendState();
  }    

  void shuttersHalt(void) 
  {
    #ifdef MY_DEBUG
      Serial.println("Shutters halted");
    #endif
      digitalWrite(relayPinUp, relayOFF);
      digitalWrite(relayPinDown, relayOFF);
    
      isMoving = false;
      requestedShutterLevel = currentShutterLevel;
    #ifdef MY_DEBUG
      Serial.println("saving state to: ");
      Serial.println(String(currentShutterLevel));
    #endif
      saveState(CHILD_ID_COVER, (int)currentShutterLevel);
      coverState = STOP;
      sendState();
  }
    
  void changeShuttersLevel(int level) 
  {
      int dir = (level > currentShutterLevel) ? DIRECTION_UP : DIRECTION_DOWN;
      if (isMoving && dir != directionUpDown) {
        shuttersHalt();
      }
      requestedShutterLevel = level;
  }

  void Update()
  {
      debouncerUp.update();
      value = debouncerUp.read();
      if (value == 0 && value != oldValueUp) {
        if(isMoving){
          shuttersHalt();
        }  
        else{
        calibrateUp = false;
        calibratedUp = false;
        changeShuttersLevel(STATE_UP);
        }
        //state = UP;
        //sendState();
      }
      oldValueUp = value;
    
      debouncerDown.update();
      value = debouncerDown.read();
      if (value == 0 && value != oldValueDown) {
        if(isMoving){
          shuttersHalt();
        }  
        else{
        calibrateDown = false;
        calibratedDown = false;
        changeShuttersLevel(STATE_DOWN);
        }    
        //state = DOWN;
        //sendState();
      }
      oldValueDown = value;
    
      if(currentShutterLevel != 100)
      {
        calibrateUp = false;
        calibratedUp = false;
      }
      if(currentShutterLevel != 0)
      {
        calibrateDown = false;
        calibratedDown = false;
      }
      
      if (isMoving) 
      {
        unsigned long _now = millis();
        if (_now - lastLevelTime >= timeOneLevel * 1000) {
          if (directionUpDown == DIRECTION_UP) {
            currentShutterLevel += 1;
          } else {
            currentShutterLevel -= 1;
          }
          currentShutterLevel = constrain(currentShutterLevel, 0, 100);
          #ifdef MY_DEBUG
          Serial.println(String(requestedShutterLevel));
          Serial.println(String(currentShutterLevel));
          #endif
          lastLevelTime = millis();
          send(msgPercentage.set((int)currentShutterLevel));
        }
        if (currentShutterLevel == requestedShutterLevel) 
        {
          if(currentShutterLevel == 0 && !calibratedDown)
          {
            if(calibrateDown == false)
            {
              calibrateDown = true;
              calibratedDown = false;
              calibrationStartTime = _now;
            }
            else 
            {
              if(calibratedDown == false)
              {
                if (_now - calibrationStartTime >= calibrationTime * 1000)
                {
                 calibratedDown = true;
                }
              }
            }
          }
          else if (currentShutterLevel == 100 && !calibratedUp)
          {
            if(calibrateUp == false)
            {
              calibrateUp = true;
              calibratedUp = false;
              calibrationStartTime = _now;
            }
            else 
            {
              if(calibratedUp == false)
              {
                if (_now - calibrationStartTime >= calibrationTime * 1000)
                {
                 calibratedUp = true;
                }
              }
            }
          }
          else
          {
            shuttersHalt();
          }
        }
      } 
      else 
      {
        if (requestedShutterLevel != currentShutterLevel) 
        {
          if (requestedShutterLevel > currentShutterLevel) {
            shuttersUp();
          }
          else {
            shuttersDown();
          }
          lastLevelTime = millis();
        }
      }
  }         
  

  void SyncController()
  {
    sendState();
  }
  
  void Present()
  {
    // Register all sensors to gw (they will be created as child devices)
     present(CHILD_ID_COVER, S_COVER, PRESENT_MESSAGE, IS_ACK);
     present(CHILD_ID_SET, S_HVAC);
  }

  void Receive(const MyMessage &message)
  {
    #ifdef MY_DEBUG
      Serial.println("recieved incomming message");
      Serial.println("Recieved message for sensor: ");
      Serial.println(String(message.sensor));
      Serial.println("Recieved message with type: ");
      Serial.println(String(message.type));
    #endif
    
     if (message.sensor == CHILD_ID_COVER) {
        int per;
        switch (message.type) {
          case V_UP:
            changeShuttersLevel(STATE_UP);
            break;
    
          case V_DOWN:
            changeShuttersLevel(STATE_DOWN);
            break;
    
          case V_STOP:
            shuttersHalt();
            break;

          case V_STATUS:
            message.getBool()?per=100:per=0;
            changeShuttersLevel(per);
            break;
    
          case V_PERCENTAGE:
            per = message.getInt();
            if (per > 100) {
              per = 100;
            }
            else if (per<0) {
              per = 0;
            }
            changeShuttersLevel(per);
            break;
        }
      } 
    else if (message.sensor == CHILD_ID_SET) {
    
        if (message.type == V_HVAC_SETPOINT_COOL || message.type == V_HVAC_SETPOINT_HEAT) 
        {
          #ifdef MY_DEBUG
          Serial.println(", New status: V_HVAC_SETPOINT_COOL, with payload: ");
          #endif      
          String strRollTime = message.getString();
          rollTime = strRollTime.toFloat();
          #ifdef MY_DEBUG
          Serial.println("rolltime value: ");
          Serial.println(String(rollTime));
          #endif
          saveState(CHILD_ID_SET, rollTime);
          sendState();
          timeOneLevel = rollTime / 100.0;
        }
      }
    #ifdef MY_DEBUG
      Serial.println("exiting incoming message");
    #endif
      return;
    }     
};

// define your RollerShutter objects here
// RollerShutter(int childId, int setId, int buttonUp, int buttonDown, int relayUp, int relayDown, int debaunceTime, bool invertedRelay)
RollerShutter blind1(0, 20, 14, 15, 22, 23, 50, 0);  
RollerShutter blind2(1, 21, 16, 17, 24, 25, 50, 0);
RollerShutter blind3(2, 22, 18, 19, 26, 27, 50, 0);  
RollerShutter blind4(3, 23, 20, 21, 28, 29, 50, 0);

void setup() 
{ 
  // Setup locally attached sensors
  delay(5000);
  blind1.SyncController(); // send actual value to controller
  blind2.SyncController();
  blind3.SyncController();
  blind4.SyncController();
}
void presentation()  
{   
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("RollerShutter as class object", "1.1");
  blind1.Present();
  blind2.Present();
  blind3.Present();
  blind4.Present();
}

void loop() 
{ 
  blind1.Update();
  blind2.Update();
  blind3.Update();
  blind4.Update();
}

void receive(const MyMessage &message) {
  blind1.Receive(message);
  blind2.Receive(message);
  blind3.Receive(message);
  blind4.Receive(message);
}
