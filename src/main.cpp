#include <Arduino.h>
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include "Adafruit_MPR121.h"
#include "Adafruit_MCP23017.h"

//THERE USED TO BE MULTIPLE AUDIO INCLUDES REPRO THAT!!!!!!!!!!!!!!!!


// GUItool: begin automatically generated code
AudioSynthWaveform       lfo;            //xy=55,734
AudioSynthNoisePink      noise2;         //xy=638,654
AudioSynthWaveform       waveVoiceA2;    //xy=641,615
AudioSynthNoisePink      noise3;         //xy=641,855
AudioSynthNoisePink      noise1;         //xy=643,401
AudioSynthWaveform       waveVoiceA3;    //xy=644,816
AudioSynthWaveform       waveVoiceA1;    //xy=646,362
AudioSynthNoisePink      noise;          //xy=647,169
AudioSynthWaveform       waveVoiceB2;    //xy=645,699
AudioSynthWaveform       waveVoiceA;     //xy=649,131
AudioSynthWaveform       waveVoiceB3;    //xy=648,900
AudioSynthWaveform       waveVoiceB1;    //xy=650,446
AudioSynthWaveform       waveVoiceB;     //xy=654,210
AudioMixer4              waveVoiceMixer2; //xy=814,652
AudioMixer4              waveVoiceMixer; //xy=819,165
AudioMixer4              waveVoiceMixer3; //xy=819,842
AudioMixer4              waveVoiceMixer1; //xy=828,398
AudioEffectChorus        chorus2;        //xy=1010,588
AudioEffectChorus        chorus3;        //xy=1012,788
AudioEffectChorus        chorus1;        //xy=1015,335
AudioEffectFreeverb      freeverb2;      //xy=1014,740
AudioEffectChorus        chorus;         //xy=1018,104
AudioEffectEnvelope      envelope2;      //xy=1016,692
AudioEffectFreeverb      freeverb3;      //xy=1016,940
AudioEffectFreeverb      freeverb1;      //xy=1019,487
AudioEffectEnvelope      envelope3;      //xy=1018,892
AudioEffectFreeverb      freeverb;       //xy=1022,256
AudioEffectEnvelope      envelope;       //xy=1024,208
AudioEffectBitcrusher    bitcrusher2;    //xy=1025,642
AudioEffectBitcrusher    bitcrusher3;    //xy=1027,842
AudioEffectBitcrusher    bitcrusher1;    //xy=1030,389
AudioEffectBitcrusher    bitcrusher;     //xy=1033,158
AudioEffectEnvelope      envelope1;      //xy=1034,433
AudioMixer4              effectMixer3;   //xy=1233,856
AudioMixer4              effectMixer2;   //xy=1235,656
AudioMixer4              effectMixer1;   //xy=1240,405
AudioMixer4              effectMixer;    //xy=1244,175
AudioMixer4              filterMixer2;   //xy=1623,745
AudioMixer4              filterMixer3;   //xy=1625,946
AudioMixer4              filterMixer1;   //xy=1628,492
AudioMixer4              filterMixer;    //xy=1631,262
AudioMixer4              first4Premix;   //xy=1970,549
AudioOutputI2S           i2s1;           //xy=2161,549
AudioConnection          patchCord1(noise2, 0, waveVoiceMixer2, 1);
AudioConnection          patchCord2(waveVoiceA2, 0, waveVoiceMixer2, 0);
AudioConnection          patchCord3(noise3, 0, waveVoiceMixer3, 1);
AudioConnection          patchCord4(noise1, 0, waveVoiceMixer1, 1);
AudioConnection          patchCord5(waveVoiceA3, 0, waveVoiceMixer3, 0);
AudioConnection          patchCord6(waveVoiceA1, 0, waveVoiceMixer1, 0);
AudioConnection          patchCord7(noise, 0, waveVoiceMixer, 1);
AudioConnection          patchCord8(waveVoiceB2, 0, waveVoiceMixer2, 2);
AudioConnection          patchCord9(waveVoiceA, 0, waveVoiceMixer, 0);
AudioConnection          patchCord10(waveVoiceB3, 0, waveVoiceMixer3, 2);
AudioConnection          patchCord11(waveVoiceB1, 0, waveVoiceMixer1, 2);
AudioConnection          patchCord12(waveVoiceB, 0, waveVoiceMixer, 2);
AudioConnection          patchCord13(waveVoiceMixer2, envelope2);
AudioConnection          patchCord14(waveVoiceMixer2, freeverb2);
AudioConnection          patchCord15(waveVoiceMixer2, chorus2);
AudioConnection          patchCord16(waveVoiceMixer2, bitcrusher2);
AudioConnection          patchCord17(waveVoiceMixer2, 0, filterMixer2, 0);
AudioConnection          patchCord18(waveVoiceMixer, envelope);
AudioConnection          patchCord19(waveVoiceMixer, freeverb);
AudioConnection          patchCord20(waveVoiceMixer, chorus);
AudioConnection          patchCord21(waveVoiceMixer, bitcrusher);
AudioConnection          patchCord22(waveVoiceMixer, 0, filterMixer, 0);
AudioConnection          patchCord23(waveVoiceMixer3, envelope3);
AudioConnection          patchCord24(waveVoiceMixer3, freeverb3);
AudioConnection          patchCord25(waveVoiceMixer3, chorus3);
AudioConnection          patchCord26(waveVoiceMixer3, bitcrusher3);
AudioConnection          patchCord27(waveVoiceMixer3, 0, filterMixer3, 0);
AudioConnection          patchCord28(waveVoiceMixer1, envelope1);
AudioConnection          patchCord29(waveVoiceMixer1, freeverb1);
AudioConnection          patchCord30(waveVoiceMixer1, chorus1);
AudioConnection          patchCord31(waveVoiceMixer1, bitcrusher1);
AudioConnection          patchCord32(waveVoiceMixer1, 0, filterMixer1, 0);
AudioConnection          patchCord33(chorus2, 0, effectMixer2, 0);
AudioConnection          patchCord34(chorus3, 0, effectMixer3, 0);
AudioConnection          patchCord35(chorus1, 0, effectMixer1, 0);
AudioConnection          patchCord36(freeverb2, 0, effectMixer2, 3);
AudioConnection          patchCord37(chorus, 0, effectMixer, 0);
AudioConnection          patchCord38(envelope2, 0, effectMixer2, 2);
AudioConnection          patchCord39(freeverb3, 0, effectMixer3, 3);
AudioConnection          patchCord40(freeverb1, 0, effectMixer1, 3);
AudioConnection          patchCord41(envelope3, 0, effectMixer3, 2);
AudioConnection          patchCord42(freeverb, 0, effectMixer, 3);
AudioConnection          patchCord43(envelope, 0, effectMixer, 2);
AudioConnection          patchCord44(bitcrusher2, 0, effectMixer2, 1);
AudioConnection          patchCord45(bitcrusher3, 0, effectMixer3, 1);
AudioConnection          patchCord46(bitcrusher1, 0, effectMixer1, 1);
AudioConnection          patchCord47(bitcrusher, 0, effectMixer, 1);
AudioConnection          patchCord48(envelope1, 0, effectMixer1, 2);
AudioConnection          patchCord49(effectMixer3, 0, filterMixer3, 3);
AudioConnection          patchCord50(effectMixer2, 0, filterMixer2, 3);
AudioConnection          patchCord51(effectMixer1, 0, filterMixer1, 3);
AudioConnection          patchCord52(effectMixer, 0, filterMixer, 3);
AudioConnection          patchCord53(filterMixer2, 0, first4Premix, 2);
AudioConnection          patchCord54(filterMixer3, 0, first4Premix, 3);
AudioConnection          patchCord55(filterMixer1, 0, first4Premix, 1);
AudioConnection          patchCord56(filterMixer, 0, first4Premix, 0);
AudioConnection          patchCord57(first4Premix, 0, i2s1, 0);
AudioConnection          patchCord58(first4Premix, 0, i2s1, 1);
AudioControlSGTL5000     sgtl5000_1;     //xy=96,20
// GUItool: end automatically generated code






#define ACTIVE_VOICE_SIZE 4
#define MUX_PINS 3
#define MUX_BUFFER_SIZE 8
#define KEYBOARD_SIZE 12
#define TOTAL_MIXERS 13
#define CHORUS_DELAY_LENGTH (16*AUDIO_BLOCK_SAMPLES)

Adafruit_MPR121 touchKeyboard = Adafruit_MPR121();
Adafruit_MCP23017 mcp;
//Bounce nextMixer = Bounce(35, 8);
//Bounce nextVoice = Bounce(36, 8);



//FUNCTION PROTOTYPES

/*
@params - pin number pressed
This function is called when the capacitive touch sensing a new pin has been touched
The pin number is passed and is used to check if there is space in the voice buffer for this
note. If there is space in the buffer an index is returned and is used to modify the voice
at that avaiable index in the voice A and B arrays
*/
void PlayNote(int id);
/*
@params - pin number pressed
This function is called when the capacitive touch senses that a pin that was previously touched is
no floater touched. The note associated with that pin is removed from the activeVoiceCount array
and the associated waveforms are turned off
*/
void StopNote(int id);
/*
@params VALUE - the number we are getting the index of
@params arrayToCheck - pointer to the array that is being checked
@params size - the size of the array we are checking
@returns the index of the element added to the end of the array 
This function is used to add a specific value to the end of an array and return the index
*/
int AddToEndofArrayAndGetIndex(int value, int8_t* array, int size);
/*
@params index - the index that we want to replace with -1 to "clear" it
@params arrayToCheck - pointer to the array that is being checked
@params size - the size of the array we are checking
This function is called when we want to "make space" in an array. An example would be for
the activeVoiceCount array. 
*/
void ClearPosition(int index, int8_t* arrayToCheck, int size);
/*
@params numToFind - the number we are getting the index of
@params arrayToCheck - pointer to the array that is being checked
@params size - the size of the array we are checking
@returns the index of the numToFind if it exists in the array if not returns -1
This function is used to get the index of a specific array element from a specific array of specific size
*/
int GetIndex(int numToFind, const int8_t* arryToCheck, int size);
/*
@params numToFind - the number we are getting the index of
@params arrayToCheck - pointer to the array that is being checked
@params size - the size of the array we are checking
@returns the index of the numToFind if it exists in the array if not returns -1
BYTE ARRAY OVERLOAD This function is used to get the index of a specific array element from a specific array of specific size
*/
int GetIndex(int numToFind, const byte* arryToCheck, int size);
int GetRandomIndex(int* arrayToCheck, int size);

/*
@param muxSelectPins - the array of digital pins that are attached to the select pins of the mux
@param analogPin - the analog pin the Z pin of the mux is attached to
@param muxBuffer - an array that stores the values of the inputs on the mux
@returns bool - this means that an input on the mux has changed beyond the noise threshold
@summary - This function toggles the proper select pins by taking in a byte value that will only be as high as 7 or 00000111
With this constraint. Bit shifting 1 to equal 1,2, and 4 a bit wise AND can be used to toggle the mux select pins
The function will then read the Mux output and call CalculateThreshold this will determine if the
input has changed enough to update the audio controls
*/
bool UpdateMuxBuffer(const u_int8_t* muxSeletPins, u_int8_t analogPin, int* muxBuffer);

/*
@param muxSelectPins - the array of digital pins that are attached to the select pins of the mux
@param analogPin - the analog pin the Z pin of the mux is attached to
@param muxBuffer - an array that stores the values of the inputs on the mux
@returns bool - this means that an input on the mux has changed beyond the noise threshold
@summary - This function toggles the proper select pins by taking in a byte value that will only be as high as 7 or 00000111
With this constraint. Bit shifting 1 to equal 1,2, and 4 a bit wise AND can be used to toggle the mux select pins
The function will then read the Mux output and call CalculateThreshold this will determine if the
input has changed enough to update the audio controls
*/
bool CalculateThresholdBoundry(int analogVal, int prevAnalogVal);

void SetupPinModes(const u_int8_t* pinArray, int size); //Just loops through the pins defined in the mux select pin arrays and sets the pin mode
void DebugMessageFromAdafruit();
void InitializeAudioObjects();
float Map(float x, float out_max);
void ApplyEffectMuxChanges(int* muxBuffer);
void ApplyWaveShapeMuxChanges(int* muxBuffer);
void ApplydahdsrMuxChanges(int* muxBuffer);
void ModifyMixerGain(int currentVoiceBranch, int currentMixer, int muxBufferData, int channel);
void UpdateBtnStateAndVoiceGlobal();
void UpdateBtnStateAndMixerGlobal();
void ChangeLEDStatus(const int currVoice, const int currMixer);
void DebugArray(const char* desc, int* array, int size);
void DebugArray(const char* desc, bool* array, int size);
void Debug(const char* desc, int value, int milli=0);
void Debug(const char* desc);

//PIN DECLARATIONS
u_int8_t effectPropertiesMuxSelectPins[] = {27,28,29};
u_int8_t effectMuxReadPin = A21;
u_int8_t waveShapePropertiesMuxSelectPins[] = {24,25,26}; //includes 2 mixer pots
u_int8_t waveShapeMuxReadPin = A22;
u_int8_t dahdsrPropertiesMuxSelectPins[] = {30,31,34}; //includes 2 mixer pots and DAHDSR pots
u_int8_t dahdsrMuxReadPin = A20;


//GLOBAL (Non-Array) VARIABLES 
int octave = 0;
float waveA0currentPhase = 0;
uint16_t touchVal = 0;
uint16_t lastTVal = 0;
bool canDebug = true;
volatile int globalCurrentMixerControl = 0;
volatile int globalCurrentVoiceControl = 0;
volatile boolean newBtnState = false;
volatile unsigned long lastInterruptTime = 0;

//GLOBAL ARRAY OBJECTS
int noteFreq[12] = {233,247,262,277,293,311,329,349,370,391,415,440};
int8_t activeVoiceCount[ACTIVE_VOICE_SIZE] = {-1, -1, -1, -1};

int effectMuxBuffer[MUX_BUFFER_SIZE] = {0,0,0,0,0,0,0,0};
int waveShapeMuxBuffer[MUX_BUFFER_SIZE] = {0,0,0,0,0,0,0,0};
int dahsdrMuxBuffer[MUX_BUFFER_SIZE] = {0,0,0,0,0,0,0,0};
bool muxUpdateControl[MUX_BUFFER_SIZE] = {};
// Allocate the delay lines for left and right channels
short delayline[CHORUS_DELAY_LENGTH];

short waveshapes[] = {WAVEFORM_SINE, WAVEFORM_SQUARE, WAVEFORM_TRIANGLE, WAVEFORM_SAWTOOTH, WAVEFORM_SAWTOOTH_REVERSE};

AudioSynthWaveform *voiceB[] = {&waveVoiceB, &waveVoiceB1, &waveVoiceB2, &waveVoiceB3};
AudioSynthWaveform *voiceA[] = {&waveVoiceA, &waveVoiceA1, &waveVoiceA2, &waveVoiceA3};
AudioMixer4 *mixers[] = {&waveVoiceMixer, &waveVoiceMixer1, &waveVoiceMixer2, &waveVoiceMixer3, &effectMixer, &effectMixer1, &effectMixer2, &effectMixer3, &filterMixer, &filterMixer1, &filterMixer2, &filterMixer3, &first4Premix};
AudioEffectFreeverb *freeverbs[] = {&freeverb, &freeverb1, &freeverb2, &freeverb3};    
AudioEffectChorus *chorusi[] = {&chorus, &chorus1, &chorus2, &chorus3};         
AudioEffectEnvelope *envelopes[] = {&envelope, &envelope1, &envelope2, &envelope3};   
AudioEffectBitcrusher *bitCrusherz[] = {&bitcrusher, &bitcrusher1, &bitcrusher2, &bitcrusher3};
AudioSynthNoisePink *noises[] = {&noise, &noise1, &noise2, &noise3};
void setup() 
{
  //Setup code for SCL1 & SDA1 interface pins
  Wire1.setSDA(38);
  Wire1.setSCL(37);
 
  Serial.begin(9600);
  AudioMemory(160);
  sgtl5000_1.enable();
  sgtl5000_1.volume(0.9);

  SetupPinModes(effectPropertiesMuxSelectPins, MUX_PINS);
  SetupPinModes(waveShapePropertiesMuxSelectPins, MUX_PINS);
  SetupPinModes(dahdsrPropertiesMuxSelectPins, MUX_PINS);
  
  pinMode(36, INPUT_PULLUP);
  pinMode(35, INPUT_PULLUP);
  
  mcp.begin(&Wire1);        // use default address 0

  for(int i = 0; i < 8; i++)
  {
    mcp.pinMode(i, OUTPUT);
  }
  
  attachInterrupt(35, UpdateBtnStateAndMixerGlobal, FALLING);
  attachInterrupt(36, UpdateBtnStateAndVoiceGlobal, FALLING);

  for(int i = 0; i < 8; i++)
  {
    mcp.digitalWrite(i, LOW); //turn all LEDs off
  }

  DebugMessageFromAdafruit();
  InitializeAudioObjects();
  
  waveVoiceMixer.gain(0, 0.4);
  waveVoiceMixer.gain(1, 0.4);
  waveVoiceMixer.gain(2, 0.1);
  waveVoiceMixer.gain(3, 0);

  bitcrusher.bits(16);
  bitcrusher.sampleRate(44100);

  effectMixer.gain(0, 0);
  effectMixer.gain(1, 1.0);
  effectMixer.gain(2, 0);
  effectMixer.gain(3, 0);
  
  filterMixer.gain(0, 0.0);
  filterMixer.gain(1, 0.0);
  filterMixer.gain(2, 0.0);
  filterMixer.gain(3, 1.0);

  first4Premix.gain(0, 0.2);
  first4Premix.gain(1, 0.2);
  first4Premix.gain(2, 0.2);
  first4Premix.gain(3, 0.2);

  waveVoiceA.begin(0.0, 349, WAVEFORM_TRIANGLE);
  waveVoiceB.begin(0.0, 349, WAVEFORM_TRIANGLE);
}


void loop() 
{
  // put your main code here, to run repeatedly:
  touchVal = touchKeyboard.touched();
  

  for(int i = 0; i < 12; i++)
  {
    if((bitRead(touchVal, i)) && !(bitRead(lastTVal, i)))
    {
      PlayNote(i);
    }

    if(!(bitRead(touchVal, i)) && (bitRead(lastTVal, i)))
    {
      StopNote(i);
    }
  }
  
  lastTVal = touchVal;
  
  if(UpdateMuxBuffer(effectPropertiesMuxSelectPins, effectMuxReadPin, effectMuxBuffer))
  {
    ApplyEffectMuxChanges(effectMuxBuffer);
  }
  
  UpdateMuxBuffer(dahdsrPropertiesMuxSelectPins, dahdsrMuxReadPin, dahsdrMuxBuffer);
  //UpdateMuxBuffer(waveShapePropertiesMuxSelectPins, waveShapeMuxReadPin, waveShapeMuxBuffer);

  if(newBtnState)
  {
    Debug("New button state detected!");
    ChangeLEDStatus(globalCurrentVoiceControl, globalCurrentMixerControl);
  }
}

void PlayNote(int noteIndex)
{ 
  int index = AddToEndofArrayAndGetIndex(noteIndex, activeVoiceCount, 4);
  if(index > -1)
  {
    AudioNoInterrupts();
    //Debug("Successfully added a voice");
    //Debug("Actual Index Used", index);
    voiceA[index]->amplitude(1.0);
    voiceB[index]->amplitude(1.0);
    voiceA[index]->frequency(noteFreq[noteIndex]);
    voiceB[index]->frequency(noteFreq[noteIndex]);
    AudioInterrupts();
  }
  else
  {
    Serial.println("Note Budder is full");
  }
}

void StopNote(int id)
{
 //Serial.println("Note turned off");

 int index = GetIndex(id, activeVoiceCount, ACTIVE_VOICE_SIZE);
 if(index > -1)
 {
   voiceA[index]->amplitude(0.0);
   voiceB[index]->amplitude(0.0);
   ClearPosition(index, activeVoiceCount, ACTIVE_VOICE_SIZE);
 }
}

int AddToEndofArrayAndGetIndex(int value, int8_t* array, int size)
{
  Serial.print("Looking for: ");
  Serial.println(value);
  Serial.print("Contents of Array: ");
  for(int i = 0; i < size; i++)
  {
    Serial.print(array[i]);
    Serial.print(" ");
    if(array[i] == -1 || array[i] == value) //held keys dont take up additional space
    {
      array[i] = value;
      Serial.print(array[i]);
      Serial.println(" ");
      return i;
    }

  }
  Serial.println("Error");
  return -1;
}

void ClearPosition(int index, int8_t* arrayToCheck, int size)
{
  Serial.print("Contents of Array: ");
  for(int i = 0; i < size; i++)
  {
    if(i == index)
    {
      arrayToCheck[i] = -1;
    }
      
    Serial.print(arrayToCheck[i]);
    Serial.print(" ");
  }

  Serial.println(" ");
}

int GetRandomIndex(int* arrayToCheck, int size)
{
  int randIndex = random(KEYBOARD_SIZE);
  return randIndex;
}

int GetIndex(int numToFind, const int8_t* arryToCheck, int size)
{
  for(int i = 0; i < size; i++)
  {
    //Serial.print("Contents of Array: ");
    //Serial.println(arryToCheck[i]);

    if(numToFind == arryToCheck[i])
    {
      return i;
    }
  }
  return -1;
}

int GetIndex(int numToFind, const byte* arryToCheck, int size)
{
  for(int i = 0; i < size; i++)
  {
    //Serial.print("Contents of Array: ");
    //Serial.println(arryToCheck[i]);

    if(numToFind == arryToCheck[i])
    {
      return i;
    }
  }
  return -1;
}

bool UpdateMuxBuffer(const u_int8_t* muxSeletPins, u_int8_t analogPin, int* muxBuffer)
{
  int tempMuxBuffer[MUX_BUFFER_SIZE];
  bool anychanges = false;
  
  for(byte outputPin = 0; outputPin< 8; outputPin++)
  {
    for(int i =0; i<3; i++)
    {
      if(outputPin & (1 << i))//toggle the mux select pins
      {
        digitalWrite(muxSeletPins[i], HIGH);
      }
      else
      {
        digitalWrite(muxSeletPins[i], LOW);
      }
    }
    //Debug("The Alleged MUX Y PiN", outputPin);
    tempMuxBuffer[outputPin] = analogRead(analogPin);
    
    
    
    bool hasMetThreshold = CalculateThresholdBoundry(tempMuxBuffer[outputPin], muxBuffer[outputPin]);
    muxUpdateControl[outputPin] = hasMetThreshold;

 
    //Debug("MUX Port Y", outputPin);
    //Debug("New Y Port Reading", tempMuxBuffer[outputPin]);
    //Debug("Prev Y Port Reading", muxBuffer[outputPin]);
    //Debug("Update MuxBuffer?", hasMetThreshold);
  
    if(hasMetThreshold)
    {
      muxBuffer[outputPin] = tempMuxBuffer[outputPin];
      if(!anychanges)
      {
        anychanges = true;
      }
    }
  }
  
  //Debug("Return Value", anychanges);
  if(anychanges)
  {
    return true;
  }
  else
  {
    return false;
  }
  
  
}

bool CalculateThresholdBoundry(int analogVal, int prevAnalogVal)
{
  int threshold;
  if(prevAnalogVal < 10)
  {
    threshold = prevAnalogVal * 4;
  }
  else if(prevAnalogVal < 50)
  {
    threshold = prevAnalogVal * 2;
  }
  else if(prevAnalogVal < 100)
  {
    threshold = prevAnalogVal * 0.5;
  }
  else
  {
    threshold = prevAnalogVal * 0.05;
  }

  //if the new analogVal is greater than prev + threshold OR if analogVal is less than prev - threshold
  if((analogVal > prevAnalogVal + threshold) || (analogVal < prevAnalogVal - threshold))
  {
   return true; 
  }
  else
  {
    return false;
  }
}

void SetupPinModes(const u_int8_t* pinArray, int size)
{
  for(int i = 0; i < size; i++)
  {
    pinMode(pinArray[i], OUTPUT);
  }
}

void InitializeAudioObjects()
{
  lfo.begin(0.5, 440, WAVEFORM_TRIANGLE);
  for(int i = 0; i > ACTIVE_VOICE_SIZE; i++)
  {
    voiceB[i]->begin(0.0, 349, WAVEFORM_TRIANGLE);
    voiceA[i]->begin(0.0, 349, WAVEFORM_TRIANGLE);
    if(!chorusi[i]->begin(delayline, CHORUS_DELAY_LENGTH, 0))
    {
      Debug("Chorusi that failed to Initialize", i);
    }
    freeverbs[i]->roomsize(0.5);
    freeverbs[i]->damping(0.5);

    bitCrusherz[i]->bits(5);
    bitCrusherz[i]->sampleRate(2000.0);

    //TODO ADD FILTERS

  }

  Debug("Waveforms initialized");
  Debug("bITCRUSHERZ initialized");
  Debug("Freeverbs initialized");
  
  for(int i = 0; i < TOTAL_MIXERS; i++)
  {
    if(i < 4) //setting the wave voice mixers
    {
      mixers[i]->gain(0, 0.4);
      mixers[i]->gain(1, 0.4);
      mixers[i]->gain(2, 0.1);
      mixers[i]->gain(3, 0);
    }
    else if((i > 7) && (i < 12)) //setting the filter Mixers
    {
      mixers[i]->gain(0, 0.9);
      mixers[i]->gain(1, 0.0);
      mixers[i]->gain(2, 0.0);
      mixers[i]->gain(3, 0.0);
    }
    
  }

  Debug("Mixers initialized");
}

float Map(float x, float out_max) 
{
  return (x - 0.0) * (out_max - 0.0) / (1023.0 - 0.0) + 0.0;
}

int Map(int x, int out_max) 
{
  return (x - 0.0) * (out_max - 0.0) / (1023.0 - 0.0) + 0.0;
}

void ApplyEffectMuxChanges(int* muxBuffer)
{
  Debug("Applying Effect Changes");
  DebugArray("Mux Control Array", muxUpdateControl, MUX_BUFFER_SIZE);

  for(int i = 0; i < MUX_BUFFER_SIZE; i++)
  {
    if(muxUpdateControl[i])
    {
      //AudioNoInterrupts();
      switch(i)
      {
        case 0:
        {      
          chorusi[globalCurrentVoiceControl]->voices(map(muxBuffer[i], 0, 1023, 0, 9)); 
          break;
        }
        case 1: 
        { 
          int newDepth = map(muxBuffer[i], 0, 1023, 1, 16);
          Debug("Raw BitDepth Pot Val", muxBuffer[i]);
          Debug("Changing BitDepth to ", newDepth);
          bitCrusherz[globalCurrentVoiceControl]->bits(newDepth);
          break;
        }
        case 2: 
        {
          int newSampleRate = map(muxBuffer[i], 0, 1023, 1, 44100);
          bitCrusherz[globalCurrentVoiceControl]->sampleRate(newSampleRate);
          Debug("Changing sample rate to: ", newSampleRate);
          break;
        }
        case 3: 
        {
          //TODO ADD FILTERS FREQ
          break;
        }
        case 4:
        {
          //TODO ADD FILTERS FREQ
          break;
        }
        case 5: 
        {
          freeverbs[globalCurrentVoiceControl]->roomsize(Map(float(muxBuffer[i]), 1.0));
          break;
        }
        case 6: 
        {
          freeverbs[globalCurrentVoiceControl]->damping(Map(float(muxBuffer[i]), 1.0));
          break;
        } 
        case 7: 
        {
          lfo.frequency(muxBuffer[i]);
          break;
        }
        default:
        {
          Serial.println("ERROR ERROR BLeEHEHEHEHEHEHEHEH");
        }
      }
      //AudioInterrupts();
    }
  }
}

void ApplyWaveShapeMuxChanges(int* muxBuffer)
{
  Debug("Applying WaveShape Changes");
  DebugArray("Mux Control Array", muxUpdateControl, MUX_BUFFER_SIZE);

  for(int i = 0; i < MUX_BUFFER_SIZE; i++)
  {
    if(muxUpdateControl[i])
    {
      //AudioNoInterrupts();
      switch(i)
      {
        case 0:
        { 
          int currentVoiceNoteIndex = activeVoiceCount[globalCurrentVoiceControl];
          if(currentVoiceNoteIndex > -1)
          {
           voiceA[globalCurrentVoiceControl]->frequency(noteFreq[currentVoiceNoteIndex]+muxBuffer[i]*2);
          }
          
          break;
        }
        /* //this recalculates the currentVoiceNoteIndex because the control for 
        globalCurrentVoiceControl is attached to an interrupt and can change mid statement*/
        case 1:
        { 
          int currentVoiceNoteIndex = activeVoiceCount[globalCurrentVoiceControl];
          if(currentVoiceNoteIndex > -1)
          {
           voiceB[globalCurrentVoiceControl]->frequency(noteFreq[currentVoiceNoteIndex]+muxBuffer[i]*2);
          }
          break;
        }
        case 2: 
        {
          voiceA[globalCurrentVoiceControl]->begin(waveshapes[Map(muxBuffer[i], 4)]);
          break;
        }
        case 3: 
        {
          voiceB[globalCurrentVoiceControl]->begin(waveshapes[Map(muxBuffer[i], 4)]);
          break;
        }
        case 4:
        {
          noises[globalCurrentVoiceControl]->amplitude(Map(float(muxBuffer[i]), 1.0));
          break;
        }
        case 5: //Mixer Channel 0
        {
          ModifyMixerGain(globalCurrentVoiceControl, globalCurrentMixerControl, muxBuffer[i], 0);
          break;
        }
        case 6: //Mixer Channel 1
        {
          ModifyMixerGain(globalCurrentVoiceControl, globalCurrentMixerControl, muxBuffer[i], 1);
          break;
        } 
        case 7: //Mixer Channel 2
        {
          ModifyMixerGain(globalCurrentVoiceControl, globalCurrentMixerControl, muxBuffer[i], 2);
          break;
        }
        default:
        {
          Serial.println("ERROR ERROR BLeEHEHEHEHEHEHEHEH");
        }
      }
      //AudioInterrupts();
    }
  }
}

void ApplydahdsrMuxChanges(int* muxBuffer)
{
  Debug("Applying Effect Changes");
  DebugArray("Mux Control Array", muxUpdateControl, MUX_BUFFER_SIZE);

  for(int i = 0; i < MUX_BUFFER_SIZE; i++)
  {
    if(muxUpdateControl[i])
    {
      //AudioNoInterrupts();
      switch(i)
      {
        case 0:
        {      
          envelopes[globalCurrentVoiceControl]->delay(muxBuffer[i]); 
          break;
        }
        case 1: 
        { 
          envelopes[globalCurrentVoiceControl]->attack(Map(muxBuffer[i], 11880)); 
          break;
        }
        case 2: 
        {
          envelopes[globalCurrentVoiceControl]->hold(Map(muxBuffer[i], 11880));
          break;
        }
        case 3: 
        {
          envelopes[globalCurrentVoiceControl]->decay(Map(muxBuffer[i], 11880));
          break;
        }
        case 4:
        {
          envelopes[globalCurrentVoiceControl]->sustain(Map(float(muxBuffer[i]), 1.0));
          break;
        }
        case 5: 
        {
          envelopes[globalCurrentVoiceControl]->release(Map(muxBuffer[i], 11880));
          break;
        }
        case 6: 
        {
          freeverbs[globalCurrentVoiceControl]->damping(Map(float(muxBuffer[i]), 1.0));
          break;
        } 
        case 7: //Mixer Channel 3
        {
          ModifyMixerGain(globalCurrentVoiceControl, globalCurrentMixerControl, muxBuffer[i], 3);
          break;
        }
        default:
        {
          Serial.println("ERROR ERROR BLeEHEHEHEHEHEHEHEH");
        }
      }
      //AudioInterrupts();
    }
  }
}

void ModifyMixerGain(const int currentVoiceBranch, const int currentMixer, int muxBufferData, int channel)
{
  switch(currentMixer)
  {
    case 0:
    {
      mixers[currentVoiceBranch]->gain(channel, muxBufferData);
      break;
    }
    case 1:
    {
      mixers[currentVoiceBranch + 4]->gain(channel, muxBufferData);
      break;
    }
    case 2:
    {
      mixers[currentVoiceBranch + 8]->gain(channel, muxBufferData);
      break;
    }
    case 3:
    {
      mixers[TOTAL_MIXERS - 1]->gain(channel, muxBufferData);
      break;
    }
    default:
    {
      Debug("Something went wrong. Reset");
      globalCurrentMixerControl = 0;
    }
  }
}

void UpdateBtnStateAndVoiceGlobal()
{
  if(millis() - lastInterruptTime > 300)
  {
    newBtnState = true;
    if(globalCurrentVoiceControl == 3)
    {
      globalCurrentVoiceControl = 0;
    }
    else
    {
      globalCurrentVoiceControl++;
    }

    lastInterruptTime = millis();
  }  
}

void UpdateBtnStateAndMixerGlobal()
{
  if(millis() - lastInterruptTime > 300)
  {
    newBtnState = true;
    if(globalCurrentMixerControl == 3)
    {
      globalCurrentMixerControl = 0;
    }
    else
    {
      globalCurrentMixerControl++;
    }
    lastInterruptTime = millis();
  } 
}

void ChangeLEDStatus(const int currVoice, const int currMixer)
{
  Debug("Changing LED states");
  Debug("Current Voice: ", currVoice);
  Debug("Current Mixer: ", currMixer);

  if(((currMixer + 4) < 8) && currVoice < 4)
  {
    for(int i = 0; i < 8; i++)
    {
      mcp.digitalWrite(i, LOW); //turn all LEDs off
    }
    
    mcp.digitalWrite(currMixer + 4, HIGH); 
    mcp.digitalWrite(currVoice, HIGH);
  }
  
  newBtnState = false;
}

void DebugMessageFromAdafruit()
{
  Serial.println("Adafruit MPR121 Capacitive Touch sensor test"); 
  
  // Default address is 0x5A, if tied to 3.3V its 0x5B
  // If tied to SDA its 0x5C and if SCL then 0x5D
  if (!touchKeyboard.begin(0x5A, &Wire1)) 
  {
    Serial.println("MPR121 not found, check wiring?");
    while (1);
  }
  Serial.println("MPR121 found!");  
  
}

void DebugArray(const char* desc, int* array, int size)
{
  if(canDebug)
  {
    Serial.print(desc);
    Serial.print(": ");
    for(int i = 0; i < size; i++)
    {
      Serial.print(array[i]);
      Serial.print(" ");
    }
    Serial.println(" ");
  }
}

void DebugArray(const char* desc, bool* array, int size)
{
  if(canDebug)
  {
    Serial.print(desc);
    Serial.print(": ");
    for(int i = 0; i < size; i++)
    {
      Serial.print(array[i]);
      Serial.print(" ");
    }
    Serial.println(" ");
  }
}

void Debug(const char* desc, int value, int milli=0)
{
  if(canDebug)
  {
    Serial.print(desc);
    Serial.print(": ");
    Serial.println(value);
    Serial.println(" ");

    delay(milli);
  }
}

void Debug(const char* desc)
{
  if(canDebug)
  {
    Serial.println(desc);
  }
  
}
