#include <Relay.h>
#include <SoftwareSerial.h>

/* control macros */
#define PRINT_DEBUG
#define ENABLE_WARNING
#define ENABLE_DEBUG_SMS
#define ENABLE_STATUS_SMS

/* pin mappings */
/* Relay Pins */
#define RELAY_OUT_PIN       12  
#define RELAY_FEDBK_PIN     A0
/* Battery Monitering pin */
#define BATT_MON_PIN        A2
/* sense pins */
#define SENSE_PIN_8KV       2
#define SENSE_PIN_6KV       3
#define MAX_SENSE_PIN_COUNT 2
/* GSM Pins */
#define GSM_TX_PIN          9
#define GSM_RX_PIN          10
#define GSM_POWER_KEY       11

/* command strings and IDs */
#define GSM_RING_STR                            "\r\nRING"
#define GSM_CALL_ID_STR                         "\r\n+CLIP"
#define GSM_POWER_DOWN_STR                      "NORMAL POWER DOWN"

#define CMD_INVALID_CMD_ID                      (-1)
#define CMD_GSM_CALL_RECV_ID                    0x01
#define CMD_GSM_INVALID_CALL_RECV_ID            0x02
#define CMD_GSM_POWER_DOWN_ID                   0x03

/* warnings */
#define OFF_STATE_WARNING                       0
#define LOW_BATT_WARNING                        1
#define SENSE_WARNING_8KV                       2
#define SENSE_WARNING_6KV                       3
#define MAX_WARNING_COUNT                       4

/* system macros */
#define MAX_DEBUG_MSG_SIZE                      128
#define MAX_CMD_STRING_SIZE                     128

#define GSM_BAUDRATE                            9600
#define DEBUG_BAUDRATE                          9600
#define GSM_SERIAL_READ_DELAY_MS                0x02

/* sense pin event types */
#define SENSE_EVENT_PULSE_COUNT                 0x01
#define SENSE_EVENT_LOW_TO_HIGH                 0x02
#define SENSE_EVENT_HIGH_TO_LOW                 0x03
#define SENSE_EVENT_PIN_CHNAGE                  0x04
#define SENSE_EVENT_LOW                         0x05
#define SENSE_EVENT_HIGH                        0x06
#define SELECTED_SENSE_EVENT_8KV                SENSE_EVENT_HIGH
#define SELECTED_SENSE_EVENT_6KV                SENSE_EVENT_HIGH

// ATDxxxxxxxxxx; -- watch out here for semicolon at the end!!
// CLIP: "+916384215939",145,"",,"",0"
#define GSM_CONTACT_NUMBER_1                    "9880303867"
#define GSM_CONTACT_NUMBER_2                    "9543807282" 
#define MAX_CONTACT_NUMBERS_STORED              2

#define MAX_OFFSTATE_TIME_SECONDS               (1800UL)

#define LOW_BATT_THRESHOLD                      500
#define LOW_BAT_MAX_ADC_SAMPLES                 5
#define BATT_VOLT_SCALING_FACTOR                (4.31f)

#define SENSE_MONITOR_PERIOD_SEC                10
#define SENSE_PULSE_PER_PERIOD                  5
#define SENSE_HIGH_LOW_COUNT                    30

#define CALL_TIMEOUT_SEC                        10
#define GSM_POWER_KEY_PULSE_TIME_MS             (2000)
#define MESSAGE_SEND_DELAY_MS                   (5000)

/* warning timeouts in minutes */
#define OFFSTATE_WARNING_PERIOD_MIN             (30)
#define LOWBAT_WARNING_PERIOD_MIN               (30)
#define SENSE_WARNING_8KV_PERIOD_MIN            (300) /* LOW Priority */
#define SENSE_WARNING_6KV_PERIOD_MIN            (30) /* HIGH Priority */
#define STATUS_SMS_PERIOD_HOUR                  (12)

Relay Rly(RELAY_OUT_PIN, RELAY_ON, true /* active low is true */);
SoftwareSerial SS_GSM(GSM_TX_PIN, GSM_RX_PIN);

uint8_t g_CurState = Rly.getState();
uint8_t g_PreState = Rly.getState();

unsigned int g_uiBattVolt = 0;
unsigned long g_ulOFFTime_ms = 0;
unsigned long g_ulStartime_ms = 0;
unsigned long g_ulWarStartTime_ms[MAX_WARNING_COUNT] = {0};
unsigned int g_uiLastStatusTime_hr = 0;

/* set this to true by default */
int g_iSendWarning[MAX_WARNING_COUNT] = {0};
unsigned int g_uiWarningCnt[MAX_WARNING_COUNT] = {0};

#ifdef PRINT_DEBUG
  char g_arrcMsg[MAX_DEBUG_MSG_SIZE] = {0};
#endif

char g_arrcCmd[MAX_CMD_STRING_SIZE] = {0};
char g_arrcGSMMsg[MAX_CMD_STRING_SIZE] = {0};
char g_arrcMsgTxt[MAX_CMD_STRING_SIZE] = {0};

/* Global Varibles used by detectSensePin() */
byte g_PrePinState[MAX_SENSE_PIN_COUNT] = {HIGH};
unsigned long g_ulPreTime[MAX_SENSE_PIN_COUNT] = {0};
unsigned long g_ulPulseCount[MAX_SENSE_PIN_COUNT] = {0};
unsigned long g_ulPinStateCnt[MAX_SENSE_PIN_COUNT] = {0};

/* contact numbers */
char ContactNumbers[MAX_CONTACT_NUMBERS_STORED][11] = {GSM_CONTACT_NUMBER_1, GSM_CONTACT_NUMBER_2};
uint8_t g_MatchIndex = 0; 

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
  
  int iRet = 0;
  
  /* set the sense pin to input pullup */
  pinMode(SENSE_PIN_8KV, INPUT_PULLUP);
  pinMode(SENSE_PIN_6KV, INPUT_PULLUP);
  
  /* init GSM module */
  SS_GSM.begin(GSM_BAUDRATE);

  /* Enable caller ID */
  EnableCallerId(true);

  /* intialize GSM serial port */
  #ifdef PRINT_DEBUG
    /* initalize debug port */
    Serial.begin(DEBUG_BAUDRATE);
  #endif

  /* set the GSM_POWER_KEY to output */
  /* the GSM module needs a LOW 
    pulse on GSM_POWER_KEY for 2 seconds 
    every startup */
  pinMode(GSM_POWER_KEY, OUTPUT);
  GSM_PowerUpDown();

  #ifdef PRINT_DEBUG
    /* get status string */
    iRet = getStatusString(g_arrcMsgTxt, MAX_DEBUG_MSG_SIZE);
    snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "%s [%d]", g_arrcMsgTxt, iRet);
    Serial.println(g_arrcMsg);
    snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "INIT Success");
    Serial.println(g_arrcMsg);
  #endif

  /* eanble warnings by default */
  for(int i = 0; i < MAX_WARNING_COUNT; i++)
  {
    g_iSendWarning[i] = true;
  }
  
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

  int iReadBytes = 0;
  int iCmdID = 0;
  int iRet = 0;

  /* memset the cmmand array */
  memset(g_arrcCmd, 0, sizeof(g_arrcCmd));

  /* receive and process GSM commands */
  iReadBytes = RecvCmd(g_arrcCmd, MAX_CMD_STRING_SIZE); 
  if(iReadBytes > 0)
  {
    #ifdef PRINT_DEBUG
      snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "Received: [%d] %s", iReadBytes, g_arrcCmd);
      Serial.println(g_arrcMsg);
    #endif

    /* print bytes */
    // printBytes(g_arrcCmd, iReadBytes);

    // validate the command
    if(isValidCmd(g_arrcCmd, iReadBytes, &iCmdID) == true)
    {
      // if valid command is received, process it
      CmdProcess(iCmdID, g_arrcGSMMsg);
      
      // SS_GSM.println(g_arrcGSMMsg);
    }
    else
    {
      // do nothing
    }
  }

#if 1
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
#endif

  /* Sense Pin detection on 8 KV - Low Priority */
  if(detectSensePin(SELECTED_SENSE_EVENT_8KV, SENSE_PIN_8KV))
  {
    ProcessWarning(SENSE_WARNING_8KV);
  }

  /* Sense Pin detection on 6 KV - High Priority */
  if(detectSensePin(SELECTED_SENSE_EVENT_6KV, SENSE_PIN_6KV))
  {
    ProcessWarning(SENSE_WARNING_6KV);
  }

  /* send status SMS once every STATUS_SMS_PERIOD_HOUR */ 
  sendStatusSMS(STATUS_SMS_PERIOD_HOUR);

  // #ifdef PRINT_DEBUG
  //   /* get status string */
  //   iRet = getStatusString(g_arrcMsgTxt, MAX_DEBUG_MSG_SIZE);
  //   snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "%s [%d]", g_arrcMsgTxt);
  //   Serial.println(g_arrcMsg);
  // #endif

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

  /* for every call GSM_RING_STR will received followed by GSM_CALL_ID_STR 
  in some cases, the both strings would be received as single message and in other
  cases both will be received as seperate strings, we only need GSM_CALL_ID_STR string 
  to validate the caller ID, hence if both strings are received as seperate messages, 
  we could process the GSM_CALL_ID_STR and discard the GSM_RING_STR, but when they are
  received as a single mesage, then we need to process the GSM_RING_STR also hence a
  case for GSM_RING_STR is added after GSM_CALL_ID_STR */

  if (StrnCmp(parrcCmd, GSM_CALL_ID_STR, strlen(GSM_CALL_ID_STR)) == true)
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
  else if (StrnCmp(parrcCmd, GSM_RING_STR, strlen(GSM_RING_STR)) == true)
  {
    /* GSM_RING_STR followed by \r\n - 8 chars 
       GSM_CALL_ID_STR followed by contact number - 13 + 10 chars
       total =  31 chars [followed by additional info] */
      if(iCmdLen < 31)
      {
        #ifdef PRINT_DEBUG
          snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "Inavlid String: %s", parrcCmd);
          Serial.println(g_arrcMsg);
        #endif

        *out_iCmdID = CMD_GSM_INVALID_CALL_RECV_ID;
      }
      else
      {
        #ifdef PRINT_DEBUG
          snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "Call Received");
          Serial.println(g_arrcMsg);
        #endif

        /* validate caller */
        /* remove the eight bytes ie GSM_RING_STR followed by \r\n 
           and pass the string */
        if(isVaildCaller(&parrcCmd[8], iCmdLen))
        {
          *out_iCmdID = CMD_GSM_CALL_RECV_ID;
        }
        else
        {
          *out_iCmdID = CMD_GSM_INVALID_CALL_RECV_ID;
        }
      }
      
      return true;
  }
  else if(detectGSMPowerDown(parrcCmd, iCmdLen))
  {
    #ifdef PRINT_DEBUG
      snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "Power Down Cmd: %s", parrcCmd);
      Serial.println(g_arrcMsg);
    #endif
    *out_iCmdID = CMD_GSM_POWER_DOWN_ID;
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
  int iIndex = 0;
  int iRet = 0;

  for(iIndex = 0; iIndex < MAX_CONTACT_NUMBERS_STORED; iIndex++)
  {
    // iRet = snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "\r\n+CLIP: "+91%snprintf",145,"",,"",0", ContactNumbers);
    #ifdef PRINT_DEBUG
      snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE,"Cmp: %s : %s", parrcCmd, ContactNumbers[iIndex]);
      Serial.println(g_arrcMsg);
    #endif
    if (StrnCmp(&parrcCmd[13], ContactNumbers[iIndex], 10) == true)
    {
      g_MatchIndex = iIndex;
      #ifdef PRINT_DEBUG
        snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE,"Match found");
        Serial.println(g_arrcMsg);
      #endif
      
      return true;
    }
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
      
      /* send message */
      /* get system time */
      iRetVal = getTimeString(g_arrcMsgTxt, MAX_CMD_STRING_SIZE, millis());
      if(Rly.getState() == RELAY_ON)
      {
        snprintf((g_arrcMsgTxt + iRetVal), MAX_CMD_STRING_SIZE, "%s", "System is ON");
      }
      else
      {
        snprintf((g_arrcMsgTxt + iRetVal), MAX_CMD_STRING_SIZE, "%s", "System is OFF");
      }
      
      /* wait before sending a message */
      delay(MESSAGE_SEND_DELAY_MS);
      SendMessage(ContactNumbers[g_MatchIndex], g_arrcMsgTxt);
      delay(MESSAGE_SEND_DELAY_MS);

      #ifdef ENABLE_DEBUG_SMS
        /* send debug message */
        snprintf(g_arrcMsgTxt, MAX_CMD_STRING_SIZE, "Cmd Str: %s", g_arrcCmd);        
        delay(MESSAGE_SEND_DELAY_MS);
        SendMessage(ContactNumbers[g_MatchIndex], g_arrcMsgTxt);
        delay(MESSAGE_SEND_DELAY_MS);
      #endif

    break;

    case CMD_GSM_POWER_DOWN_ID:

      /* power up the GSM module */
      #ifdef PRINT_DEBUG
        snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "GSM Power Down detected");
        Serial.println(g_arrcMsg);
      #endif

      /* power up the GSM module */
      #ifdef PRINT_DEBUG
        snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "Powering up GSM module");
        Serial.println(g_arrcMsg);
      #endif

      GSM_PowerUpDown();

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
  int iRet = 0;

  /* get time string */
  iRet = getTimeString(g_arrcMsgTxt, MAX_CMD_STRING_SIZE, millis());

  switch(iWarnID)
  {
    case OFF_STATE_WARNING:
      iRet += snprintf((g_arrcMsgTxt + iRet), MAX_CMD_STRING_SIZE, "System is OFF for more than %d minutes\nLast Warn ", 
              (MAX_OFFSTATE_TIME_SECONDS / 60));
      iRet += getTimeString((g_arrcMsgTxt + iRet), MAX_CMD_STRING_SIZE, g_ulWarStartTime_ms[OFF_STATE_WARNING]);

      /* send consecutive warnings with atleast WARNING_PERIOD_MIN  interval inbetween */
      
      /* process OFF state warning */
      if((g_iSendWarning[OFF_STATE_WARNING] == false) && ((millis() - g_ulWarStartTime_ms[OFF_STATE_WARNING]) / (1000UL * 60UL) > OFFSTATE_WARNING_PERIOD_MIN))
      {
        g_iSendWarning[OFF_STATE_WARNING] = true;
      }
      
      if(g_iSendWarning[OFF_STATE_WARNING] == true)
      {
        // #ifdef PRINT_DEBUG
        //   snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "OFF State: Sending warning");
        //   Serial.println(g_arrcMsg);
        // #endif

        SendWarning();
        /* increment warning counter */
        g_uiWarningCnt[OFF_STATE_WARNING]++;

        g_iSendWarning[OFF_STATE_WARNING] = false;
        g_ulWarStartTime_ms[OFF_STATE_WARNING] = millis();
      }
    break;

    case LOW_BATT_WARNING:
      iRet += snprintf((g_arrcMsgTxt + iRet), MAX_CMD_STRING_SIZE, "Low Battery WARNING...!\nLast Warn ");
      iRet += getTimeString((g_arrcMsgTxt + iRet), MAX_CMD_STRING_SIZE, g_ulWarStartTime_ms[LOW_BATT_WARNING]);
      
      /* process Low battery warning */
      if((g_iSendWarning[LOW_BATT_WARNING] == false) && ((millis() - g_ulWarStartTime_ms[LOW_BATT_WARNING]) / (1000UL * 60UL) > LOWBAT_WARNING_PERIOD_MIN))
      {
        g_iSendWarning[LOW_BATT_WARNING] = true;
      }
      
      if(g_iSendWarning[LOW_BATT_WARNING] == true)
      {
        // #ifdef PRINT_DEBUG
        //   snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "Low batt: Sending warning");
        //   Serial.println(g_arrcMsg);
        // #endif

        SendWarning();
        /* increment warning counter */
        g_uiWarningCnt[LOW_BATT_WARNING]++;

        g_iSendWarning[LOW_BATT_WARNING] = false;
        g_ulWarStartTime_ms[LOW_BATT_WARNING] = millis();
      }
    break;

    case SENSE_WARNING_8KV:
      iRet += snprintf((g_arrcMsgTxt + iRet), MAX_CMD_STRING_SIZE, "Mild Fence Warning 8KV (Low Priority)...!\nLast Warn ");
      iRet += getTimeString((g_arrcMsgTxt + iRet), MAX_CMD_STRING_SIZE, g_ulWarStartTime_ms[SENSE_WARNING_8KV]);
      /* process Sense warning */
      if((g_iSendWarning[SENSE_WARNING_8KV] == false) && ((millis() - g_ulWarStartTime_ms[SENSE_WARNING_8KV]) / (1000UL * 60UL) > SENSE_WARNING_8KV_PERIOD_MIN))
      {
        g_iSendWarning[SENSE_WARNING_8KV] = true;
      }
      
      if(g_iSendWarning[SENSE_WARNING_8KV] == true)
      {
        // #ifdef PRINT_DEBUG
        //   snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "Sense 8K: Sending warning");
        //   Serial.println(g_arrcMsg);
        // #endif

        SendWarning();
        /* increment warning counter */
        g_uiWarningCnt[SENSE_WARNING_8KV]++;

        g_iSendWarning[SENSE_WARNING_8KV] = false;
        g_ulWarStartTime_ms[SENSE_WARNING_8KV] = millis();
      }
    break;

    case SENSE_WARNING_6KV:
      iRet += snprintf((g_arrcMsgTxt + iRet), MAX_CMD_STRING_SIZE, "Severe Fence Warning 6KV (High Priority)...!\nLast Warn ");
      iRet += getTimeString((g_arrcMsgTxt + iRet), MAX_CMD_STRING_SIZE, g_ulWarStartTime_ms[SENSE_WARNING_6KV]);
      /* process Sense warning */
      if((g_iSendWarning[SENSE_WARNING_6KV] == false) && ((millis() - g_ulWarStartTime_ms[SENSE_WARNING_6KV]) / (1000UL * 60UL) > SENSE_WARNING_6KV_PERIOD_MIN))
      {
        g_iSendWarning[SENSE_WARNING_6KV] = true;
      }
      
      if(g_iSendWarning[SENSE_WARNING_6KV] == true)
      {
        // #ifdef PRINT_DEBUG
        //   snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "Sense 6K: Sending warning");
        //   Serial.println(g_arrcMsg);
        // #endif
        SendWarning();
        /* increment warning counter */
        g_uiWarningCnt[SENSE_WARNING_6KV]++;

        g_iSendWarning[SENSE_WARNING_6KV] = false;
        g_ulWarStartTime_ms[SENSE_WARNING_6KV] = millis();
      }
    break;

    deafult:
      return;
  }

  // #ifdef PRINT_DEBUG
  //   snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "%s", g_arrcMsgTxt);
  //   Serial.println(g_arrcMsg);
  // #endif

  // for(int i = 0; i < MAX_WARNING_COUNT; i++)
  // {
  //   #ifdef PRINT_DEBUG
  //     snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "[%d] State: %d Start Time: %d sec", i, g_iSendWarning[i], (g_ulWarStartTime_ms[i] / 1000UL));
  //     Serial.println(g_arrcMsg);
  //   #endif
  // }
}

/***********************************************************************************************/
/*! 
* \fn         :: SendWarning()
* \author     :: Vignesh S
* \date       :: 23-Jun-2020
* \brief      :: This sends warnings call and warning messages
* \param[in]  :: None
* \param[out] :: None
* \return     :: None
*/
/***********************************************************************************************/
void SendWarning()
{
#ifdef ENABLE_WARNING
  /* disable caller ID */
  EnableCallerId(false);

  /* send message */
  for(int i = 0; i < MAX_CONTACT_NUMBERS_STORED; i++)
  {
    /* call and hangup */
    MakeCall(ContactNumbers[i]);
    delay(CALL_TIMEOUT_SEC * 1000UL);

    HangupCall();
    
    /* send message */
    SendMessage(ContactNumbers[i], g_arrcMsgTxt);

    /* delay */
    delay(5000);
  }

  /* disable caller ID */
  EnableCallerId(true);
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
  bool LowBattState = false;

  /* clear the global variable */
  g_uiBattVolt = 0;

  /* average over LOW_BAT_MAX_ADC_SAMPLES */
  for(int  i = 0; i < LOW_BAT_MAX_ADC_SAMPLES; i++)
  {
    g_uiBattVolt += analogRead(BATT_MON_PIN);
    delay(5);
  }

  g_uiBattVolt /= LOW_BAT_MAX_ADC_SAMPLES;

  if(g_uiBattVolt < LOW_BATT_THRESHOLD)
  {
    LowBattState = true;
  }

  // #ifdef PRINT_DEBUG
  //   snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "Bat Voltage: %d", g_uiBattVolt);
  //   Serial.println(g_arrcMsg);
  // #endif

  return LowBattState;
}

/***********************************************************************************************/
/*! 
* \fn         :: detectSensePin()
* \author     :: Vignesh S
* \date       :: 02-Jun-2020
* \brief      :: This function processes pulse generated by sense pin 
* \param[in]  :: iEventType (pulse count or level trigger)
* \param[out] :: None
* \return     :: true | false
*/
/***********************************************************************************************/
bool detectSensePin(int iEventType, const int iSensePin)
{
  byte pinState = 0;
  int iIndex = 0;
  bool SenseState = false;
  unsigned long CurTime = (millis() / 1000);

  /* select iIndex based on pin */
  switch(iSensePin)
  {
    case SENSE_PIN_8KV:
      iIndex = 0;
    break;

    case SENSE_PIN_6KV:
      iIndex = 1;
    break;

    default:
    ;
  }

  /* if system state is OFF then just return false */
  if(Rly.getState() == RELAY_OFF)
  {
    return false;
  }

  switch(iEventType)
  {
    case SENSE_EVENT_PULSE_COUNT:

      /* detect low */
      pinState = digitalRead(iSensePin);

      /* increment the pulse count if previous state is not equal to current state */
      if(g_PrePinState[iIndex] != pinState)
      {
        g_ulPulseCount[iIndex]++;
      }

      // #ifdef PRINT_DEBUG
      //   snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "Prev: %d Pin: %d Pulse: %d", g_PrePinState[iIndex], pinState, g_ulPulseCount[iIndex]);
      //   Serial.println(g_arrcMsg);
      // #endif

      /* monitor pulse count for every cycle */
      if((CurTime > 0) && ((CurTime % SENSE_MONITOR_PERIOD_SEC) == 0) && (CurTime != g_ulPreTime[iIndex]))
      {
        if(g_ulPulseCount[iIndex] < SENSE_PULSE_PER_PERIOD)
        {
          SenseState = true;
        }

        // #ifdef PRINT_DEBUG
        //   // snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "[%d sec] Pulse Count: %d", CurTime, g_ulPulseCount[iIndex]);
        //   snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "Pulse Count: %d", g_ulPulseCount[iIndex]);
        //   Serial.println(g_arrcMsg);
        // #endif

        /* reset the count */
        g_ulPulseCount[iIndex] = 0;

      }

      /* assign current pin state previous state */
      g_PrePinState[iIndex] = pinState;
      g_ulPreTime[iIndex] = CurTime;

    break;

    case SENSE_EVENT_LOW_TO_HIGH:

      /* detect low */
      pinState = digitalRead(iSensePin);

      // #ifdef PRINT_DEBUG
      //   snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "Prev: %d Pin: %d", g_PrePinState[iIndex], pinState);
      //   Serial.println(g_arrcMsg);
      // #endif

      if((g_PrePinState[iIndex] == LOW) && (pinState == HIGH))
      {
        SenseState = true;
      }
      
      /* assign current pin state previous state */
      g_PrePinState[iIndex] = pinState;

    break;

    case SENSE_EVENT_HIGH_TO_LOW:

      /* read pin state */
      pinState = digitalRead(iSensePin);

      // #ifdef PRINT_DEBUG
      //   snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "Prev: %d Pin: %d", g_PrePinState[iIndex], pinState);
      //   Serial.println(g_arrcMsg);
      // #endif

      if((g_PrePinState[iIndex] == HIGH) && (pinState == LOW))
      {
        SenseState = true;
      }
      
      /* assign current pin state previous state */
      g_PrePinState[iIndex] = pinState;

    break;

    case SENSE_EVENT_PIN_CHNAGE:

      /* read pin state */
      pinState = digitalRead(iSensePin);

      // #ifdef PRINT_DEBUG
      //   snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "Prev: %d Pin: %d", g_PrePinState[iIndex], pinState);
      //   Serial.println(g_arrcMsg);
      // #endif

      if(g_PrePinState[iIndex] != pinState)
      {
        SenseState = true;
      }
      
      /* assign current pin state previous state */
      g_PrePinState[iIndex] = pinState;

    break;

    case SENSE_EVENT_LOW:

      /* read pin state */
      pinState = digitalRead(iSensePin);

      // #ifdef PRINT_DEBUG
      //   snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "Prev: %d Pin: %d Cnt: %d", g_PrePinState[iIndex], pinState, g_ulPinStateCnt[iIndex]);
      //   Serial.println(g_arrcMsg);
      // #endif

      if((pinState == LOW) && (g_PrePinState[iIndex] == LOW))
      {
        g_ulPinStateCnt[iIndex]++;
      }
      else
      {
        g_ulPinStateCnt[iIndex] = 0;
      }

      if(g_ulPinStateCnt[iIndex] >= SENSE_HIGH_LOW_COUNT)
      {
        g_ulPinStateCnt[iIndex] = 0;
        SenseState = true;
      }

      /* assign current pin state previous state */
      g_PrePinState[iIndex] = pinState;

    break;

    case SENSE_EVENT_HIGH:

      /* read pin state */
      pinState = digitalRead(iSensePin);

      // #ifdef PRINT_DEBUG
      //   snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "Prev: %d Pin: %d Cnt: %d", g_PrePinState[iIndex], pinState, g_ulPinStateCnt[iIndex]);
      //   Serial.println(g_arrcMsg);
      // #endif

      if((pinState == HIGH) && (g_PrePinState[iIndex] == HIGH))
      {
        g_ulPinStateCnt[iIndex]++;
      }
      else
      {
        g_ulPinStateCnt[iIndex] = 0;
      }

      if(g_ulPinStateCnt[iIndex] >= SENSE_HIGH_LOW_COUNT)
      {
        g_ulPinStateCnt[iIndex] = 0;
        SenseState = true;
      }

      /* assign current pin state previous state */
      g_PrePinState[iIndex] = pinState;
      
    break;

    default:
    ;
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
  snprintf(g_arrcGSMMsg, MAX_CMD_STRING_SIZE, "ATD+91%s;", PhNumber);
  SS_GSM.println(g_arrcGSMMsg);
  #ifdef PRINT_DEBUG
    snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "Calling cmd: %s\n", g_arrcGSMMsg);
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
* \fn         :: EnableCallerId()
* \author     :: Vignesh S
* \date       :: 18-Jun-2020
* \brief      :: This function enables or disables caller iD
* \param[in]  :: None
* \param[out] :: None
* \return     :: None
*/
/***********************************************************************************************/
void EnableCallerId(bool state)
{
  if(state == true)
  {
    SS_GSM.println("AT+CLIP=1\r");
    #ifdef PRINT_DEBUG
      snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "Caller ID enabled");
      Serial.println(g_arrcMsg);
    #endif
  }
  else
  {
    SS_GSM.println("AT+CLIP=1\r");
    #ifdef PRINT_DEBUG
      snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "Caller ID disabled");
      Serial.println(g_arrcMsg);
    #endif
  }

  /* delay for mode switch */
  delay(2000);
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

  #ifdef PRINT_DEBUG
    snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "Sending to %s", PhNumber);
    Serial.println(g_arrcMsg);
    snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "%s", Message);
    Serial.println(g_arrcMsg);
  #endif

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

/***********************************************************************************************/
/*! 
* \fn         :: GSM_PowerUpDown()
* \author     :: Vignesh S
* \date       :: 22-Jun-2020
* \brief      :: This function toggles the GSM_POWER_KEY
* \param[in]  :: none
* \param[in]  :: None
* \param[out] :: None
* \return     :: None
*/
/***********************************************************************************************/
void GSM_PowerUpDown()
{
  delay(GSM_POWER_KEY_PULSE_TIME_MS / 2);
  digitalWrite(GSM_POWER_KEY, HIGH);
  delay(GSM_POWER_KEY_PULSE_TIME_MS);
  digitalWrite(GSM_POWER_KEY, LOW);  
  delay(GSM_POWER_KEY_PULSE_TIME_MS / 2);
}

/***********************************************************************************************/
/*! 
* \fn         :: detectGSMPowerDown()
* \author     :: Vignesh S
* \date       :: 22-Jun-2020
* \brief      :: This function detectes power down 
* \param[in]  :: none
* \param[out] :: None
* \return     :: None
*/
/***********************************************************************************************/
bool detectGSMPowerDown(char *string, int iSize)
{
  char testStr[] = GSM_POWER_DOWN_STR;
  
  // #ifdef PRINT_DEBUG
  //   snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "CMP: %s CMP Size:%d", string, iSize);
  //   Serial.println(g_arrcMsg);
  // #endif
  
  if(iSize < strlen(testStr))
  {
    return false;    
  }

  for(int i = 0; i < (iSize - strlen(testStr)); i++)
  {
    // #ifdef PRINT_DEBUG
    //   snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "[%d] CMP: %c and %c", i, string[i], testStr[0]);
    //   Serial.println(g_arrcMsg);
    // #endif

    if(string[i] == testStr[0])
    {
      if(StrnCmp(&string[i], testStr, strlen(testStr)))
      {
        // #ifdef PRINT_DEBUG
        //   snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "[%d] CMP: %s matches %s", i, &string[i], testStr);
        //   Serial.println(g_arrcMsg);
        // #endif

        return true;
      }
    }
  }

  return false;
}

/***********************************************************************************************/
/*! 
* \fn         :: getTimeString()
* \author     :: Vignesh S
* \date       :: 28-Jun-2020
* \brief      :: This function frames time from millis() in HH:MM:SS format
* \param[in]  :: buffer
* \param[in]  :: ulTime_Sec
* \param[in]  :: iSize
* \return     :: number of characters processed
*/
/***********************************************************************************************/
int getTimeString(char *buffer, int iSize, unsigned long ulTime_ms)
{
  int iRet = 0;
  int iHour = 0;
  int iMin = 0;
  int iSec = 0;
  unsigned long ulTime_Sec = (ulTime_ms / 1000UL);

  if(buffer == NULL)
  {
    return -1;
  }

  iHour = (int)((ulTime_Sec) / 3600UL);
  iMin = (int)((ulTime_Sec - ((unsigned long)iHour * 3600UL)) / 60);
  iSec = (int)(ulTime_Sec % 60);

  return snprintf(buffer, iSize, "[%02d:%02d:%02d] ", iHour, iMin, iSec);
}

/***********************************************************************************************/
/*! 
* \fn         :: getStatusString()
* \author     :: Vignesh S
* \date       :: 28-Jun-2020
* \brief      :: This function frames system status string
* \param[in]  :: buffer
* \param[in]  :: iSize
* \return     :: number of characters processed
*/
/***********************************************************************************************/
int getStatusString(char *buffer, int iSize)
{
  int iRet = 0;
  float fBatVoltage = 0;

  if(buffer == NULL)
  {
    return -1;
  }

  iRet += getTimeString(buffer, iSize, millis());

  iRet += snprintf((buffer + iRet), (iSize -iRet), "STATUS\n");
    
  /* Get System ON OFF status */
  if(Rly.getState() == RELAY_ON)
  {
    iRet += snprintf((buffer + iRet), (iSize -iRet), "SYS ON\n");
  }
  else
  {
    iRet += snprintf((buffer + iRet), (iSize -iRet), "SYS OFF\n");
  }

  /* print battery voltage */
  fBatVoltage = ((float)g_uiBattVolt * (5.0f / 1024.0f) * BATT_VOLT_SCALING_FACTOR);
  iRet += snprintf((buffer + iRet), (iSize -iRet), "BAT %02d.%d%dV\n", 
                    (int)fBatVoltage, /* integer part */
                    ((int)(fBatVoltage * 10) % 10),/* first digit */
                    ((int)(fBatVoltage * 100) % 10)); /* second digit */

  /* print warning counts */
  iRet += snprintf((buffer + iRet), (iSize -iRet), "WRN [OFF LBAT SEN8K SEN6K]\n");
  iRet += snprintf((buffer + iRet), (iSize -iRet), "CNT [%4d %4d %4d %4d]\n", 
                  g_uiWarningCnt[OFF_STATE_WARNING],
                  g_uiWarningCnt[LOW_BATT_WARNING],
                  g_uiWarningCnt[SENSE_WARNING_8KV],
                  g_uiWarningCnt[SENSE_WARNING_6KV]);

  /* print warning timeouts */
  iRet += snprintf((buffer + iRet), (iSize -iRet), "TOT [%4d %4d %4d %4d]\n", 
                  OFFSTATE_WARNING_PERIOD_MIN,
                  LOWBAT_WARNING_PERIOD_MIN,
                  SENSE_WARNING_8KV_PERIOD_MIN,
                  SENSE_WARNING_6KV_PERIOD_MIN);
  return iRet; 
}

/***********************************************************************************************/
/*! 
* \fn         :: sendStatusSMS()
* \author     :: Vignesh S
* \date       :: 28-Jun-2020
* \brief      :: This function sends system status string to stored contacts
* \param[in]  :: none
* \param[out] :: none
* \return     :: none
*/
/***********************************************************************************************/
void sendStatusSMS(unsigned int uiPeroidHour)
{
#ifdef ENABLE_STATUS_SMS

  int iRet = 0;
  int iIndex = 0;
  unsigned int uiTime_hr = (unsigned int)(millis() / (1000UL * 3600UL));

  if((uiTime_hr != g_uiLastStatusTime_hr) && ((uiTime_hr % uiPeroidHour) == 0))
  {
    /* get status string */
    iRet = getStatusString(g_arrcMsgTxt, MAX_CMD_STRING_SIZE);

    /* send the to all contacts */
    for(iIndex = 0; iIndex < MAX_CONTACT_NUMBERS_STORED; iIndex++)
    {
      SendMessage(ContactNumbers[iIndex], g_arrcMsgTxt);
      delay(MESSAGE_SEND_DELAY_MS);
    }

    /* set last status sent time to current time */
    g_uiLastStatusTime_hr = uiTime_hr;
  }

  // #ifdef PRINT_DEBUG
  //   snprintf(g_arrcMsg, MAX_DEBUG_MSG_SIZE, "Ststus Size: %d Time: %d hr Last Time: %d hr",
  //            iRet, uiTime_hr, g_uiLastStatusTime_hr);
  //   Serial.println(g_arrcMsg);
  // #endif

#endif
}