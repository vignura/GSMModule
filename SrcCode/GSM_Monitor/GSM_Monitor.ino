#include <Relay.h>
#include <SoftwareSerial.h>

/* control macros */
#define PRINT_DEBUG
// #define ENABLE_WARNING

/* pin mappings */
#define RELAY_OUT_PIN       13  
#define RELAY_FEDBK_PIN     A0
#define BATT_MON_PIN        A2
#define PULSE_SENSE_PIN     2 // or 3
#define GSM_TX_PIN          9
#define GSM_RX_PIN          10

/* command strings and IDs */
#define GSM_RING_STR                            "\r\nRING"

#define CMD_INVALID_CMD_ID                      (-1)
#define CMD_GSM_CALL_RECV_ID                    0x01
#define CMD_GSM_INVALID_CALL_RECV_ID            0x02

/* warnings */
#define OFF_STATE_WARNING                       0x01
#define LOW_BATT_WARNING                        0x02
#define SENSE_WARNING                           0x03

/* system macros */
#define MAX_DEBUG_MSG_SIZE                      128
#define MAX_CMD_STRING_SIZE                     128

#define GSM_BAUDRATE                            9600
#define DEBUG_BAUDRATE                          9600
#define GSM_SERIAL_READ_DELAY_MS                0x02

// ATDxxxxxxxxxx; -- watch out here for semicolon at the end!!
#define GSM_CONTACT_NUMBER                      "9880303867" 
#define GSM_RECEIVE_NUMBER                      "6384215939" 
#define MAX_OFFSTATE_TIME_SECONDS               (1800UL)
#define LOW_BATT_THRESHOLD                      500
#define SENSE_MONITOR_PERIOD_SEC                10
#define SENSE_PULSE_PER_PERIOD                  1
#define CALL_TIMEOUT_SEC                        10
#define WARNING_PERIOD_MIN                      (30)

Relay Rly(RELAY_OUT_PIN, RELAY_ON, true /* active low is true */);
SoftwareSerial SS_GSM(GSM_TX_PIN, GSM_RX_PIN);

uint8_t g_CurState = Rly.getState();
uint8_t g_PreState = Rly.getState();

unsigned long g_ulOFFTime_ms = 0;
unsigned long g_ulStartime_ms = 0;
unsigned long g_ulWarStartTime_ms = 0;

/* set this to true by default */
bool g_bSendWarning = true;

#ifdef PRINT_DEBUG
  char g_arrcMsg[MAX_DEBUG_MSG_SIZE] = {0};
#endif

char g_arrcGSMMsg[MAX_CMD_STRING_SIZE] = {0};

/* pulse count */
volatile unsigned long g_vulPulseCount = 0;

/***********************************************************************************************/
/*! 
* \fn         :: setup()
* \author     :: Vignesh S
* \date       :: 02-Jun-2020
* \brief      :: This function preforms the one time initializations
* \param[in]  :: None
* \return     :: None
*/
/***********************************************************************************************/
void setup() {
 
  /* intialize pulse sense pin for interrupt */
  attachInterrupt(digitalPinToInterrupt(PULSE_SENSE_PIN), PulseSense_ISR, RISING);

  SS_GSM.begin(GSM_BAUDRATE);

  /* intialize GSM serial port */
  #ifdef PRINT_DEBUG
    /* initalize debug port */
    Serial.begin(DEBUG_BAUDRATE);
  #endif

  #ifdef PRINT_DEBUG
      snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "INIT Success");
      Serial.println(g_arrcMsg);
  #endif

}

/***********************************************************************************************/
/*! 
* \fn         :: EmgStopInterrupt()
* \author     :: Vignesh S
* \date       :: 02-Jun-2020
* \brief      :: This main loop function that performs the following tasks
*                 1. GSM Command handling
*                 2. OFF State detection
*                 3. Battery Low volatge detection
*                 4. Sense pin monitoring
* \param[in]  :: None
* \return     :: None
*/
/***********************************************************************************************/
void loop() {
  
  char arrcCmd[MAX_CMD_STRING_SIZE] = {0};
  int iReadBytes = 0;
  int iCmdID = 0;

  /* receive and process GSM commands */
  iReadBytes = RecvCmd(arrcCmd, MAX_CMD_STRING_SIZE); 
  if(iReadBytes > 0)
  {
    #ifdef PRINT_DEBUG
      snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "Received: [%d] %s", iReadBytes, arrcCmd);
      Serial.println(g_arrcMsg);
    #endif

    /* print bytes */
    // printBytes(arrcCmd, iReadBytes);

    // validate the command
    if(isValidCmd(arrcCmd, iReadBytes, &iCmdID) == true)
    {
      // if valid command is received, process it
      // CmdProcess(iCmdID, g_arrcGSMMsg);
      
      // SS_GSM.println(g_arrcGSMMsg);
    }
    else
    {
      // do nothing
    }
  }

  /* off state detection */
  if(detectOFFState(MAX_OFFSTATE_TIME_SECONDS))
  {
    ProcessWarning(OFF_STATE_WARNING);
  }

  /* Low battery detection */
  if(detectLowBatt())
  {
    ProcessWarning(LOW_BATT_WARNING);
  }

  /* Sense Pin detection */
  if(detectSensePin())
  {
    ProcessWarning(SENSE_WARNING);
  }

}


/***********************************************************************************************/
/*! 
* \fn         :: PulseSense_ISR()
* \author     :: Vignesh S
* \date       :: 02-Jun-2020
* \brief      :: This function increment the global pulse count for every pulse received
* \param[in]  :: None
* \return     :: None
*/
/***********************************************************************************************/
void PulseSense_ISR()
{
  /* increment the count for every pulse received */
  g_vulPulseCount++;
}

/***********************************************************************************************/
/*! 
* \fn         :: RecvCmd()
* \author     :: Vignesh S
* \date       :: 02-Jun-2020
* \brief      :: This function cheks GSM serial port for received data and if data is available
*                then reads it, the number of bytes read is returned.
* \param[in]  :: pBuff, iBuflen
* \return     :: iIndex
*/
/***********************************************************************************************/
int RecvCmd(char *pBuff, int iBuflen)
{
  int iIndex = 0;

  if(pBuff == NULL)
  {
    return -1;
  }
  
  while(iIndex < iBuflen)
  {
    delay(GSM_SERIAL_READ_DELAY_MS);
    
    if(SS_GSM.available())
    {
      pBuff[iIndex] = SS_GSM.read();
      iIndex++;
    }
    else
    {
      break;
    }
  }

  return iIndex;
}

/***********************************************************************************************/
/*! 
* \fn         :: isValidCmd()
* \author     :: Vignesh S
* \date       :: 02-Jun-2020
* \brief      :: This function validates the received command and return true if it a valid 
*                command else returns false, if a valid command is received, then the 
*                corresponding command ID is returned through out_iCmdID parameter, similarly
*                Invalid command ID error code is returned.
* \param[in]  :: parrcCmd, iCmdLen 
* \param[out] :: out_iCmdID
* \return     :: true or false
*/
/***********************************************************************************************/
bool isValidCmd(char *parrcCmd, int iCmdLen, int *out_iCmdID)
{
  int iRetVal = 0;

  if((parrcCmd == NULL) || (out_iCmdID == NULL) || (iCmdLen <= 0))
  {
    return false;
  }

  if (StrnCmp(parrcCmd, GSM_RING_STR, strlen(GSM_RING_STR)) == true)
  {
    #ifdef PRINT_DEBUG
      snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "Call Received");
      Serial.println(g_arrcMsg);
    #endif

    /* validate caller */
    if(isVaildCaller(parrcCmd, iCmdLen))
    {
      *out_iCmdID = CMD_GSM_CALL_RECV_ID;
    }
    else
    {
      *out_iCmdID = CMD_GSM_INVALID_CALL_RECV_ID;
    }
    
    return true;
  }
  else
  {
    // invalid command
    #ifdef PRINT_DEBUG
      snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE,"Invalid Cmd: %s", parrcCmd);
      Serial.println(g_arrcMsg);
    #endif
    *out_iCmdID = CMD_INVALID_CMD_ID;
  }

  return false;
}
/***********************************************************************************************/
/*! 
* \fn         :: isValidCmd()
* \author     :: Vignesh S
* \date       :: 02-Jun-2020
* \brief      :: This function validates the caller's mobile number and returns true if the 
*                number is valid else returns false
* \param[in]  :: parrcCmd, iCmdLen 
* \param[out] :: None
* \return     :: true or false
*/
/***********************************************************************************************/
bool isVaildCaller(char *parrcCmd, int iCmdLen)
{
  if (StrnCmp(parrcCmd, GSM_CONTACT_NUMBER, strlen(GSM_CONTACT_NUMBER)) == true)
  {
    return true;
  }

  return false;
}

/***********************************************************************************************/
/*! 
* \fn         :: StrnCmp()
* \author     :: Vignesh S
* \date       :: 05-DEC-2018
* \brief      :: This function compares two strings, returns true if they are identical else
*                false.  
* \param[in]  :: pString1, pString2, iLen 
* \return     :: true or false
*/
/***********************************************************************************************/
bool StrnCmp(char *pString1, char *pString2, int iLen)
{
  if((pString1 == NULL) || (pString2 == NULL) || (iLen <= 0))
  {
    return false;
  }

  for(int iIndex = 0; (iIndex < iLen); iIndex++)
  {
    if(pString1[iIndex] != pString2[iIndex])
    {
      return false;
    }
  }

  return true;
}

/***********************************************************************************************/
/*! 
* \fn         :: CmdProcess()
* \author     :: Vignesh S
* \date       :: 02-Jun-2020
* \brief      :: This function processes the recceived command and preforms corresponding task
* \param[in]  :: iCmdID
* \param[out] :: ipResponse
* \return     :: None
*/
/***********************************************************************************************/
void CmdProcess(int iCmdID, char *pResponse)
{ 
  int iRetVal = 0;

  if(pResponse == NULL)
  {
    return;
  }

  switch(iCmdID)
  {
    case CMD_GSM_CALL_RECV_ID:
      // sprintf(pResponse, "%s", "Response");
      /* don't answer the call just hangup */
      HangupCall();
      #ifdef PRINT_DEBUG
        snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "Valid Call Toggling RELAY");
        Serial.println(g_arrcMsg);
      #endif
      
      Rly.ToggleState();
    break;

    case CMD_GSM_INVALID_CALL_RECV_ID:
      /* don't answer the call just hangup */
      HangupCall();

      #ifdef PRINT_DEBUG
        snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "Invalid Call Hanging up");
        Serial.println(g_arrcMsg);
      #endif
    break;

    default:
      ;// do nothing 
  }
}

/***********************************************************************************************/
/*! 
* \fn         :: detectOFFState()
* \author     :: Vignesh S
* \date       :: 02-Jun-2020
* \brief      :: This function processes the recceived command and preforms corresponding task
* \param[in]  :: ulOFFTime_Sec OFF time threshold in seconds
* \param[out] :: None
* \return     :: true | false
*/
/***********************************************************************************************/
bool detectOFFState(unsigned long ulOFFTime_Sec)
{
  bool OFFState = false;

  /* update current state */
  g_CurState = Rly.getState();

  /* start the timer on ON to OFF Transition */
  if((g_PreState == RELAY_ON) && (g_CurState == RELAY_OFF))
  {
    g_ulStartime_ms = millis();
  }
  /* stop the timer on OFF to ON State Transition */
  /* clear the off time count */
  else if((g_PreState == RELAY_OFF) && (g_CurState == RELAY_ON))
  {
    g_ulOFFTime_ms = 0;
  }
  /* if the relay is OFF contineously then update off time */
  else if((g_PreState == RELAY_OFF) && (g_CurState == RELAY_OFF))
  {
    g_ulOFFTime_ms = millis() - g_ulStartime_ms;
  }
  else
  {
    // do nothing
  }

  /* If off time is greater than g_ulOFFTime_ms then */
  if((g_ulOFFTime_ms / 1000) > ulOFFTime_Sec)
  {
    OFFState = true;
  }

  /* assign current state to previous state */
  g_PreState = g_CurState;

  return OFFState;
}

/***********************************************************************************************/
/*! 
* \fn         :: ProcessWarning()
* \author     :: Vignesh S
* \date       :: 02-Jun-2020
* \brief      :: This function processes warnings and initiates a call to GSM_CONTACT_NUMBER
*                and also sends warning messages
* \param[in]  :: iWarnID
* \param[out] :: None
* \return     :: None
*/
/***********************************************************************************************/
void ProcessWarning(int iWarnID)
{
  switch(iWarnID)
  {
    case OFF_STATE_WARNING:
      snprintf(g_arrcGSMMsg, MAX_CMD_STRING_SIZE, "System is OFF for more than %d minutes", 
              (MAX_OFFSTATE_TIME_SECONDS / 60));
    break;

    case LOW_BATT_WARNING:
      snprintf(g_arrcGSMMsg, MAX_CMD_STRING_SIZE, "Low Battery WARNING...!");
    break;

    case SENSE_WARNING:
      snprintf(g_arrcGSMMsg, MAX_CMD_STRING_SIZE, "Sense Input WARNING...!");
    break;

    deafult:
      return;
  }

#ifdef ENABLE_WARNING
  /* send consecutive warnings with atleast WARNING_PERIOD_MIN  interval inbetween */
  if((g_bSendWarning == false) && ((millis() - g_ulWarStartTime_ms) / (1000UL * 60UL) > WARNING_PERIOD_MIN) )
  {
    g_bSendWarning = true;
  }
  
  if(g_bSendWarning == true)
  {
    /* call and hangup */
    MakeCall(GSM_CONTACT_NUMBER);
    delay(CALL_TIMEOUT_SEC * 1000UL);

    /* send message */
    SendMessage(GSM_CONTACT_NUMBER, g_arrcGSMMsg);

    g_bSendWarning = false;
    g_ulWarStartTime_ms = millis();
  }
#endif
}

/***********************************************************************************************/
/*! 
* \fn         :: detectLowBatt()
* \author     :: Vignesh S
* \date       :: 02-Jun-2020
* \brief      :: This function detects battery volagte level from BATT_MON_PIN
* \param[in]  :: None
* \param[out] :: None
* \return     :: true | false
*/
/***********************************************************************************************/
bool detectLowBatt()
{
  int iBattVolt = 0;
  bool LowBattState = false;

  iBattVolt = analogRead(BATT_MON_PIN);

  if(iBattVolt < LOW_BATT_THRESHOLD)
  {
    LowBattState = true;
  }

  return LowBattState;
}

/***********************************************************************************************/
/*! 
* \fn         :: detectSensePin()
* \author     :: Vignesh S
* \date       :: 02-Jun-2020
* \brief      :: This function processes pulse generated by sense pin 
* \param[in]  :: None
* \param[out] :: None
* \return     :: true | false
*/
/***********************************************************************************************/
bool detectSensePin()
{
  bool SenseState = false;

  /* monitor pulse count for every cycle */
  if((millis() / 1000) % SENSE_MONITOR_PERIOD_SEC)
  {
    if(g_vulPulseCount < SENSE_PULSE_PER_PERIOD)
    {
      SenseState = true;
    }

    /* reset the count */
    g_vulPulseCount = 0;
  }

  return SenseState;
}


/***********************************************************************************************/
/*! 
* \fn         :: MakeCall()
* \author     :: Vignesh S
* \date       :: 06-Jun-2020
* \brief      :: This function makes a call to PhNumber
* \param[in]  :: PhNumber - Phone number (10 digit string)
* \param[out] :: None
* \return     :: None
*/
/***********************************************************************************************/
void MakeCall(const char *PhNumber)
{
  snprintf(g_arrcGSMMsg, MAX_CMD_STRING_SIZE, "ATD+%s;", PhNumber);
  SS_GSM.println(g_arrcGSMMsg);
  #ifdef PRINT_DEBUG
    snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "Calling %s\n", PhNumber);
    Serial.println(g_arrcMsg);
  #endif
}

/***********************************************************************************************/
/*! 
* \fn         :: HangupCall()
* \author     :: Vignesh S
* \date       :: 06-Jun-2020
* \brief      :: This function hangups a call
* \param[in]  :: None
* \param[out] :: None
* \return     :: None
*/
/***********************************************************************************************/
void HangupCall()
{
  SS_GSM.println("ATH");
  #ifdef PRINT_DEBUG
    snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "Hangup Call\n");
    Serial.println(g_arrcMsg);
  #endif
  delay(1000);
}

/***********************************************************************************************/
/*! 
* \fn         :: ReceiveCall()
* \author     :: Vignesh S
* \date       :: 06-Jun-2020
* \brief      :: This function sets the GSM module to receive a call 
* \param[in]  :: None
* \param[out] :: None
* \return     :: None
*/
/***********************************************************************************************/
void ReceiveCall()
{
  SS_GSM.println("ATA");
  #ifdef PRINT_DEBUG
    snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "Receiving Call\n");
    Serial.println(g_arrcMsg);
  #endif
}

/***********************************************************************************************/
/*! 
* \fn         :: SendMessage()
* \author     :: Vignesh S
* \date       :: 06-Jun-2020
* \brief      :: This function sends a message
* \param[in]  :: PhNumber - Phone number (10 digit string)
* \param[in]  :: Message - Message to send
* \param[out] :: None
* \return     :: None
*/
/***********************************************************************************************/
void SendMessage(const char *PhNumber, const char *Message)
{
  SS_GSM.println("AT+CMGF=1");    //Sets the GSM Module in Text Mode
  delay(1000);  // Delay of 1000 milli seconds or 1 second
  
  snprintf(g_arrcGSMMsg, MAX_CMD_STRING_SIZE, "AT+CMGS=\"+91%s\"\r", PhNumber);
  SS_GSM.println(g_arrcGSMMsg); 
  delay(1000);

  SS_GSM.println(Message);// The SMS text you want to send
  delay(100);
  SS_GSM.println((char)26);// ASCII code of CTRL+Z
  delay(1000);
}

/***********************************************************************************************/
/*! 
* \fn         :: printBytes()
* \author     :: Vignesh S
* \date       :: 06-Jun-2020
* \brief      :: This function each character in hexadecimal form
* \param[in]  :: array
* \param[in]  :: Size
* \param[out] :: None
* \return     :: None
*/
/***********************************************************************************************/
void printBytes(char *array, int iSize)
{
  for(int i = 0; i < iSize; i++)
  {
    #ifdef PRINT_DEBUG
      memset(g_arrcMsg, 0, sizeof(g_arrcMsg));
      snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "[%d]: 0x%x[%c]", i, array[i], array[i]);
      Serial.println(g_arrcMsg);
    #endif
  }
}
