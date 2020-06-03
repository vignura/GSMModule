#include <Relay.h>
#include <SoftwareSerial.h>

/* control macros */
#define PRINT_DEBUG

/* pin mappings */
#define RELAY_OUT_PIN       13  
#define RELAY_FEDBK_PIN     A1
#define BATT_MON_PIN        A0
#define PULSE_SENSE_PIN     2 // or 3
#define GSM_TX_PIN          9
#define GSM_RX_PIN          10

/* command strings and IDs */
#define CMD_RELAY_TOGGLE_STR                    "Call Received"

#define CMD_INVALID_CMD_ID                      (-1)
#define CMD_RELAY_TOGGLE_ID                     0x01

/* warnings */
#define OFF_STATE_WARNING                       0x01
#define LOW_BATT_WARNING                        0x02
#define SENSE_WARNING                           0x03

/* system macros */
#define MAX_DEBUG_MSG_SIZE                      128
#define MAX_CMD_STRING_SIZE                     32

#define GSM_BAUDRATE                            9600
#define DEBUG_BAUDRATE                          9600
#define GSM_SERIAL_READ_DELAY_MS                0x02

#define MAX_OFFSTATE_TIME_SECONDS               (1800UL)

#define LOW_BATT_THRESHOLD                      500

Relay Rly(RELAY_OUT_PIN, RELAY_ON, true /* active low is true */);
SoftwareSerial SS_GSM(GSM_TX_PIN, GSM_RX_PIN);

uint8_t g_CurState = Rly.getState();
uint8_t g_PreState = Rly.getState();

unsigned long g_ulOFFTime_ms = 0;
unsigned long g_ulStartime_ms = 0;

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
      sprintf(g_arrcMsg, "INIT Success");
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
      sprintf(g_arrcMsg, "Received: [%d] %s", iReadBytes, arrcCmd);
      Serial.println(g_arrcMsg);
    #endif

    // validate the command
    if(isValidCmd(arrcCmd, iReadBytes, &iCmdID) == true)
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
    ProcessWarning(OFF_STATE_WARNING);
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

  if (StrnCmp(parrcCmd, CMD_RELAY_TOGGLE_STR, strlen(CMD_RELAY_TOGGLE_STR)) == true)
  {
    /* further string processing */

    #ifdef PRINT_DEBUG
      sprintf(g_arrcMsg,"Toggle command received");
      Serial.println(g_arrcMsg);
    #endif

    *out_iCmdID = CMD_RELAY_TOGGLE_ID;
    return true;
  }
  else
  {
    // invalid command
    #ifdef PRINT_DEBUG
      sprintf(g_arrcMsg,"Invalid Cmd: %s", parrcCmd);
      Serial.println(g_arrcMsg);
    #endif
    *out_iCmdID = CMD_INVALID_CMD_ID;
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
    case CMD_RELAY_TOGGLE_ID:

      sprintf(pResponse, "%s", "Response");
      
      #ifdef PRINT_DEBUG
        sprintf(g_arrcMsg, "Toggling RELAY state");
        Serial.println(g_arrcMsg);
      #endif
      
      Rly.ToggleState();
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
* \brief      :: This function processes the recceived command and preforms corresponding task
* \param[in]  :: ulOFFTime_Sec OFF time threshold in seconds
* \param[out] :: None
* \return     :: true | false
*/
/***********************************************************************************************/
void ProcessWarning(int iWarnID)
{
  switch(iWarnID)
  {
    case OFF_STATE_WARNING:
    break;

    case LOW_BATT_WARNING:
    break;

    case SENSE_WARNING:
    break;

    deafult:
      ; /* do nothing */
  }
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

  return SenseState;
}
