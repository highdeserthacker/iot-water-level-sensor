/**************************************************************************************/
#include <ArduinoJson.h>
#include "MyApp.h"
#include "QTimer.h"
#include "QMath.h"
#include "QFile.h"

/**************************************************************************************/
// Debugging
/**************************************************************************************/
// refer to MyApp.h
float       _AppSettingsVersion= 0;

/**************************************************************************************/
/* Identifier for this device.
   Used for mqtt, ota host name, debugging.
   For mqtt, this name must be unique across all active microcontrollers on the lan. */
const int   _MaxDeviceNameLen= 15;
char        _pDeviceIdentifier[_MaxDeviceNameLen+1]= "pool";

/**************************************************************************************/
// Pin Definitions
/**************************************************************************************/
// Inputs
const int Pin_FloatSensorLow=    (D5);                // Lower float
const int Pin_FloatSensorHigh=   (D6);                // Upper float

// (D1) & (D2) are used for I2C
const int PinOneWire=            (D2);

/**************************************************************************************/
// MQTT
/**************************************************************************************/
#include "QMQTT.h"               // MQTT Server Interface

/**************************************************************************************/
// Core
/**************************************************************************************/
QFile *                    _pEnvSettingsFile= NULL;

#include "QCore.h"                                   
QCore *                    _pCore;

/**************************************************************************************/
// Temperature Sensor
/**************************************************************************************/
#include "QMQTT_Entity.h"
QMQTT_Entity_Temperature_Sensor * _pTemperatureEntity= NULL;
const char *   _pTemperatureSensorSubtopic= "temperature";  // device/[deviceIdentifier]/entities/temperature

/**************************************************************************************/
// Float Sensor
/**************************************************************************************/
int                        _nFloats= 2;               // Set this to type of float installed, 1|2.            
QMQTT_Entity_Binary_Sensor      * _pWaterLevelSensorEntityLow= NULL;
QMQTT_Entity_Binary_Sensor      * _pWaterLevelSensorEntityHigh= NULL;

const char *   _pWaterLevelSensorLowSubtopic= "waterlevellow";    // device/[deviceIdentifier]/entities/waterlevellow
const char *   _pWaterLevelSensorHighSubtopic= "waterlevelhigh";    // device/[deviceIdentifier]/entities/waterlevelhigh

/**************************************************************************************/
void setup()
{
   #ifdef _DEBUG
   QTraceSwitch TraceSwitch= QTrace::_TraceSwitchesLevel_Verbose;
   #else
   QTraceSwitch TraceSwitch= QTrace::_TraceSwitchesLevel_Warning;
   #endif

   pinMode(Pin_FloatSensorLow, INPUT_PULLUP);
   pinMode(Pin_FloatSensorHigh, INPUT_PULLUP);
   pinMode(PinOneWire, OUTPUT);

   /* If app settings file not yet created on this device, create it. */
   QFile * pSettingsFile= new QFile(APP_SETTINGS_FILENAME);
   if (!pSettingsFile->Exists())
      WriteAppSettings();

   /* Read app settings file. 
      device topic name
      Trace settings, reporting freq, the various debug flags for QCore.
      build date
      Temp sensor calibration
   */
   ReadAppSettings();

   /* Set up core common services for wifi, mqtt, trace, etc. */
   QCore::ServiceSettingT  Services= (QCore::ServiceSettingT) (QCore::ServiceSettingT::SST_Default);
   _pCore= new QCore(_pDeviceIdentifier, Services); 

   Serial.printf("\nsetup(): exit\n");   

} // setup
/**************************************************************************************/
void loop()
{
   /* DoService() checks for wifi, mqtt, ntp, etc. */
   _pCore->DoService();

   if (QWifi::Master()->IsConnected())
   {  // Wifi connection is ok

      /* QMQTT_Entity */
      /* Create Sensor entities. Creation of 1st entity initializes QMQTT_Entity class.
         This is done after wifi and mqtt are set up.  */
      if ((QMQTT::Master()->IsConnected()) && (_pWaterLevelSensorEntityLow == NULL))
      {
         /* Lower Water level sensor - the switch is closed (on) when float is down, open (off) when float is up.
            Float in up position: switch is open (off). Input is pulled up and reads high. Want this to be reported as Up= on/true/1.
            Float in down position: switch is closed (on). Input is grounded and reads low. Want this reported as Down= off/false/0.
         */
         _pWaterLevelSensorEntityLow= new QMQTT_Entity_Binary_Sensor(/*SubTopic*/_pWaterLevelSensorLowSubtopic, QMQTT_Entity::EIOT::IOT_GPIO, /*Pin*/Pin_FloatSensorLow, /*ActiveLow*/false);
         _pWaterLevelSensorEntityLow->SetReadSensorCallback(BinarySensorReadCallback);

         /* Upper Water level sensor */
         if (_nFloats > 1)
         {
            _pWaterLevelSensorEntityHigh= new QMQTT_Entity_Binary_Sensor(/*SubTopic*/_pWaterLevelSensorHighSubtopic, QMQTT_Entity::EIOT::IOT_GPIO, /*Pin*/Pin_FloatSensorHigh, /*ActiveLow*/false);
            _pWaterLevelSensorEntityHigh->SetReadSensorCallback(BinarySensorReadCallback);
            _pWaterLevelSensorEntityHigh->SetReportSensorCallback(ReportSensorCallback);
         }

         // I2C Temperature Sensor
         int Pin; 
         QTemperature::TemperatureSensorT SensorType;
         Pin= PinOneWire;
         SensorType= QTemperature::TST_I2C;
         _pTemperatureEntity= new QMQTT_Entity_Temperature_Sensor(/*SubTopic*/_pTemperatureSensorSubtopic, /*SensorType*/SensorType, /*Pin*/Pin);
      }

      /* Service the entities - updates sensor values, etc. This handles state reporting to mqtt. */
      QMQTT_Entity::DoService();
   }

} // loop
/**************************************************************************************/
/* Called for read of each float sensor. Readings are accumulated to determine
   stability and combining of floats for overall value.
   This gets called every 1 second. 
   This relies on calls made in order of sensor Id.  
   Inputs:  id - id of device. Corresponds to it's index position in the list of entities 0|1
            SensorState - true/Up, false/Down
   Returns: measured stable reading. This will be the previous stable value until a new
            stable value is accepted.                 

   Initialize to an invalid state, upper float up, lower float down.
   Encoded state - upper up|lower down 

   State Machine - 
   0: New State == Current State:  no change seen
   1: New State != CurrentState: change in state. Save NewState as CandidateState, start timer. Move to next state.
   2: Stability Check: stay here as long as NewState == CandidateState. Else back to state 0. If timer is done, have
      completed stable measurement, move to next state.
   3: Update state: timer has completed, and no change in CandidateState since then. Set CurrentState to CandidateState. 
*/
int            _StateMachineIndex=  0;
// Combined Sensor Variables
#define ILLEGAL_STATE         0x02
#define FULL_STATE            0x03
#define FILL_STATE            0x01
#define LOW_STATE             0x00           // For 2 float configuration, this is urgent level. For 1 float, it is fill state.

uint16_t       _CurrentState=   FULL_STATE;      
uint16_t       _CandidateState;
uint16_t       _NewState= 0;
const char *   _StateNames[]= {"Low", "Fill", "Invalid", "Full"};
QTimestamp::TimestampType  _LastStateChangeTimestamp= 0;
QTimestamp::TimestampType  _RawStateChangeTimestamp= 0;
QTimer *       _pStabilityTimer= new QTimer(/*min*/1 */*sec*/60 */*msec*/1000,/*Repeat*/false,/*Start*/false,/*Done*/true);

int BinarySensorReadCallback(int Id, bool SensorState)
{
   _NewState= QMath::SetBitState(_NewState, Id, (SensorState)?(1):(0));

   if (Id == (_nFloats - 1))
   {  // Callback for last float (whether 1 or 2 are configured).

      /* State Machine - check that state readings are stable. Confirm that once state changes for a given
      entity, that it stays in that state for minimum stability period (60 sec). */
      switch (_StateMachineIndex)
      {
         case 0:
         default:
         {
            if (_NewState != _CurrentState)
            {  /* Change in state. */
               _CandidateState= _NewState;
               _pStabilityTimer->Start();
               _RawStateChangeTimestamp= QTimestamp::GetNowTimeMsec();
               _StateMachineIndex++;
               #ifdef _DEBUG
               _Trace.printf(TS_MAIN, TLT_Verbose, "BinarySensorReadCallback(): sensor state change detected (%s to %s), begin stability check.", _StateNames[_CurrentState], _StateNames[_CandidateState]);
               #endif
            }
            break;
         }
         case 1:
         {
            if (_NewState != _CandidateState)
            {  /* Instability, start over. */
               _pStabilityTimer->Start();             // Causes wait before looking for a change in CurrentState.
               _RawStateChangeTimestamp= QTimestamp::GetNowTimeMsec();
               _StateMachineIndex++;
               #ifdef _DEBUG
               _Trace.printf(TS_MAIN, TLT_Verbose, "%s", "BinarySensorReadCallback(): sensor state unstable, toss reading and sleep.");
               #endif
            }
            else if (_pStabilityTimer->IsDone())
            {  /* Stable change in state. */  
               #ifdef _DEBUG
               _Trace.printf(TS_MAIN, TLT_Verbose, "BinarySensorReadCallback(): sensor state change stable, update state from %s to %s.", _StateNames[_CurrentState], _StateNames[_CandidateState]);
               #endif

               if (_NewState == ILLEGAL_STATE)
                  _Trace.printf(TS_MAIN, TLT_Error, "BinarySensorReadCallback(): Illegal State (%s)! Check sensor.", _StateNames[_CurrentState]);

               _CurrentState= _CandidateState;
               _LastStateChangeTimestamp= QTimestamp::GetNowTimeMsec();
               _StateMachineIndex= 0;
            }
            break;
         }
         case 2:
         {  /* Instability in measurement found, sleep it off. */
            if (_pStabilityTimer->IsDone())
            {  
               _StateMachineIndex= 0;
               #ifdef _DEBUG
               _Trace.printf(TS_MAIN, TLT_Verbose, "%s", "BinarySensorReadCallback(): sleep period complete, return to checking state.");
               #endif
            }
            break;
         }
      }
   }

   return _CurrentState;

} // BinarySensorReadCallback
/**************************************************************************************/
/* Callback from high/upper float entity for reporting. Prepares and returns custom json payload. 
   Needed to augment report with additional information:
      - Float state in integer form
      - Float state in text form
      - utc timestamp of last state change
      - StateChangeAgeSec: number of seconds since last state change.
      - StateChangeRawAgeSec: number of seconds since any change in state detected. Raw, not smoothed.
           Can be used to inform is sensor is working

   Of the form:
   {
      "state":"on",                 // this is the state of the upper binary sensor. Required, but not relevant for dual float sensor.
      "FloatStateInt": 3,
      "FloatStateStr": "Full",
      "StateChangeAgeSec": 36.4,
      "StateChangeRawAgeSec": 7.6
   }
*/
void ReportSensorCallback(int Id, bool SensorState)
{
   /* Prep json buffer. */
   StaticJsonBuffer<JSON_OBJECT_SIZE(16)> jsonBuffer;
   JsonObject& JsonRoot= jsonBuffer.createObject();

   JsonRoot[QMQTT_Entity::_pStateCommand]= (SensorState)?(QMQTT_Entity::_pStateOn):(QMQTT_Entity::_pStateOff);
   JsonRoot["FloatStateInt"]=          _CurrentState;
   JsonRoot["FloatStateStr"]=          _StateNames[_CurrentState];
   JsonRoot["StateChangeAgeSec"]=      QTimestamp::GetAgeSec(_LastStateChangeTimestamp);
   JsonRoot["StateChangeRawAgeSec"]=   QTimestamp::GetAgeSec(_RawStateChangeTimestamp);

   int Cnt= JsonRoot.measureLength() + 1;
   char JsonPayloadStr[Cnt];
   int nBytes= JsonRoot.printTo(JsonPayloadStr, sizeof(JsonPayloadStr));

   _pWaterLevelSensorEntityHigh->ReportJsonStr(JsonPayloadStr);

} // ReportSensorCallback
/**************************************************************************************/
/* Writes the application settings in json format to settings file.
   Of the form:
   {
      "version":"1.0",
      "device_name":"pool",
   }
*/
void WriteAppSettings()
{
   const int JsonBuffserSize= JSON_OBJECT_SIZE(16);
   StaticJsonBuffer<JsonBuffserSize> jsonBuffer;
   JsonObject& JsonRoot= jsonBuffer.createObject();

   JsonRoot[JSON_PROP_VERSION]=           (float) 1.0;      // Use a float form of version for ease of comparison between versions
   JsonRoot[JSON_PROP_DEVICE_NAME]=        _pDeviceIdentifier;

   int Cnt= JsonRoot.measureLength() + 1;
   char JsonText[Cnt];
   Cnt= JsonRoot.printTo(JsonText, sizeof(JsonText));

   QFile * pSettingsFile= new QFile(APP_SETTINGS_FILENAME);
   pSettingsFile->WriteStr(JsonText);
   //_Trace.printf(TS_MAIN, TLT_Verbose, "WriteAppSettings(): json:%s", JsonText);
   Serial.printf("\nWriteEnvSettings(): created file.\n");   
   
} // WriteAppSettings
/**************************************************************************************/
/* Reads the application settings file.   */
void ReadAppSettings()
{
   /* Load configuration data from the configuration file. */
   /* Set defaults in the event that no cfg file yet. */
   QFile * pSettingsFile= new QFile(APP_SETTINGS_FILENAME);

   if (pSettingsFile->Exists())
   {
      int ConfigBfrSize= 256;
      char ConfigBfr[ConfigBfrSize];
      int Cnt= pSettingsFile->ReadStr(ConfigBfr, ConfigBfrSize);        // Read all the settings
   
      /* Parse the json configuration data */
      StaticJsonBuffer<JSON_OBJECT_SIZE(32)> jsonBuffer;                // allocates buffer on stack
      JsonObject& JsonRoot= jsonBuffer.parseObject(ConfigBfr);

      if (JsonRoot.success())
      {  // Successful parse
         _AppSettingsVersion= JsonRoot[JSON_PROP_VERSION];

         if (JsonRoot.containsKey(JSON_PROP_DEVICE_NAME))
            strlcpy(_pDeviceIdentifier, JsonRoot[JSON_PROP_DEVICE_NAME], _MaxDeviceNameLen+1);

      }
   }
   else
   {  /* Settings file not found. */
   }
   
} // ReadAppSettings

