 
#include <Wire.h>
// Include Adafruit PCA9685 Servo Library
#include <Adafruit_PWMServoDriver.h>

#include <ADS1299ESP32.h>
#include <Biquad_multiChan.h>

#define N_EEG_CHANNELS (1)
#define SAMPLE_RATE_HZ (250.0)  //default setting for OpenBCI
#define FILTER_Q (0.5)        //critically damped is 0.707 (Butterworth)
#define FILTER_PEAK_GAIN_DB (0.0) //doesn't matter for Lowpass, highpass, notch, or bandpass
#define HP_CUTOFF_HZ (0.5)  //set the desired cutoff for the highpass filter
Biquad_multiChan stopDC_filter(N_EEG_CHANNELS,bq_type_highpass,HP_CUTOFF_HZ / SAMPLE_RATE_HZ, FILTER_Q, FILTER_PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
#define NOTCH_FREQ_HZ (50.0)
#define NOTCH_Q (4.0)              //pretty shap notch
Biquad_multiChan notch_filter1(N_EEG_CHANNELS,bq_type_notch,NOTCH_FREQ_HZ / SAMPLE_RATE_HZ, NOTCH_Q, FILTER_PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad_multiChan notch_filter2(N_EEG_CHANNELS,bq_type_notch,NOTCH_FREQ_HZ / SAMPLE_RATE_HZ, NOTCH_Q, FILTER_PEAK_GAIN_DB); //one for each channel because the object maintains the filter states

//Design signal detection filters
#define BP_FREQ_HZ (10.1f)  //focus on Alpha waves
#define BP_Q (2.0f)         //gives somewhat steeply sloped sides
Biquad_multiChan bandpass_filter1(N_EEG_CHANNELS,bq_type_bandpass,BP_FREQ_HZ / SAMPLE_RATE_HZ, BP_Q, FILTER_PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad_multiChan bandpass_filter2(N_EEG_CHANNELS,bq_type_bandpass,BP_FREQ_HZ / SAMPLE_RATE_HZ, BP_Q, FILTER_PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
#define RMS_LP_PERIOD_SEC  (1.0f)  //how long do we want to do the average (seconds)
#define RMS_LP_FREQ_HZ  (1.0f/RMS_LP_PERIOD_SEC)
#define RMS_LP_Q (1.0f)  //keep below 1.0 to avoid a peak at the cuttof frequency
Biquad_multiChan rms_lowpass_filter1(N_EEG_CHANNELS,bq_type_lowpass,RMS_LP_FREQ_HZ / SAMPLE_RATE_HZ, BP_Q, FILTER_PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad_multiChan rms_lowpass_filter2(N_EEG_CHANNELS,bq_type_lowpass,RMS_LP_FREQ_HZ / SAMPLE_RATE_HZ, BP_Q, FILTER_PEAK_GAIN_DB); //one for each channel because the object maintains the filter states

//for guard filtering
#define BP_GUARD_HZ (25.0)
Biquad_multiChan bandpass_filter3(N_EEG_CHANNELS,bq_type_bandpass,BP_GUARD_HZ / SAMPLE_RATE_HZ, BP_Q, FILTER_PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad_multiChan bandpass_filter4(N_EEG_CHANNELS,bq_type_bandpass,BP_GUARD_HZ / SAMPLE_RATE_HZ, BP_Q, FILTER_PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad_multiChan rms_lowpass_filter3(N_EEG_CHANNELS,bq_type_lowpass,RMS_LP_FREQ_HZ / SAMPLE_RATE_HZ, BP_Q, FILTER_PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad_multiChan rms_lowpass_filter4(N_EEG_CHANNELS,bq_type_lowpass,RMS_LP_FREQ_HZ / SAMPLE_RATE_HZ, BP_Q, FILTER_PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad_multiChan *bp1, *bp2, *lp1, *lp2;


//define some data variables
float EEG_inband_rms[N_EEG_CHANNELS];
float EEG_guard_rms[N_EEG_CHANNELS];
#define MICROVOLTS_PER_COUNT (0.02235174f) 
//#define MICROVOLTS_PER_COUNT (1.0f)  //don't scale, just operate as counts

//define some output pins
#define LED_OUTPUT_PIN  5     //choose a PWM pin
#define MIN_LED_VAL  (2)      //set the minimum LED brightness


const int freq = 5000;
const int ledchannel = 0;
const int resolution = 8;
ADS1299ESP32 ADS;
//SCLK = 14, MISO = 12, MOSI = 13, SS = 15, RESET = 32 , DRDY = 33 

void Contract_Fist(void);
void Relax_Fist(void);


Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);
 
// Define maximum and minimum number of "ticks" for the servo motors
// Range from 0 to 4095
// This determines the pulse width
 
#define SERVOMIN  80  // Minimum value
#define SERVOMAX  600  // Maximum value
 
// Define servo motor connections (expand as required)
#define SER0  0   //Servo Motor 1 on connector 0
#define SER1  1  //Servo Motor 2 on connector 1
#define SER2  2  //Servo Motor 3 on connector 2
#define SER3  3  //Servo Motor 4 on connector 3
#define SER4  4 //Servo Motor 5 on connector 4

 
// Variables for Servo Motor positions (expand as required)
int pwm0;
int pwm1;
int pwm2;
int pwm3;
int pwm4;



 
void setup() {
  // don't put anything before the initialization routine for recommended POR  
   ADS.initialize(); // (DRDY pin, RST pin, CS pin, SCK frequency in MHz);
 
  Serial.begin(115200);


  // Initialize PCA9685
  pca9685.begin();
 
  // Set PWM Frequency to 50Hz
  pca9685.setPWMFreq(50);
  
  delay(1000);             
  ADS.disp = false;      // when disp is true, there will be Serial feedback 
  ADS.reset();             // all registers set to default
  ADS.sdatac();            // stop Read Data Continuous mode to communicate with ADS
 
  // Blynk.begin(auth, ssid, pass);
  
  pinMode(5,OUTPUT);
    //Enabling BIAS measurement buffer for channel 1
 //enabling BIAS signal 
  ADS.wreg(CH1SET,0x68);  //0110 1000  PGA=24 for low noise PGA must be high and SPS should be low 
  ADS.wreg(BIAS_SENSP,0x01);
  ADS.wreg(BIAS_SENSN,0x01);
  ADS.wreg(CONFIG3,0xEC);  //1110 0000    Open BCI 0b1110 1100
  delay(1);
 // LED AND BUZZER
  //pinMode(LED_OUTPUT_PIN,OUTPUT);

  //ledcSetup(ledchannel, freq, resolution);
  //ledcAttachPin(LED_OUTPUT_PIN, ledchannel);
  //ledcWrite(ledchannel, 255);  
  
  delay(1000);
  //Reading all registers  
  ADS.rregs(ID,CONFIG4);         // read out what we just did to verify the write
  ADS.rdatac();                      // enter Read Data Continuous mode
  ADS.start();
  delay(30);
} // end of setup

int prev_LED_val = 0, LED_val = 0;  
void loop(){
//Blynk.run();

  while(digitalRead(ADS.DRDY)){}
    ADS.updatedata();
   
  //float ads_value =(ADS.channelData[0])*((2* 4.5/1)/(2^24)) ;  //1 LSB = (2 × VREF / Gain) / 2^24 = +FS / 2^23
  int ads_value = ADS.channelData[0];
  //Serial.println(ads_value);

  //process the data
    processData_wGuard();
    prev_LED_val = LED_val;
    LED_val = updateLED(EEG_inband_rms[0],EEG_guard_rms[0],prev_LED_val);
} // end of loop


void processData_wGuard(void) {
  float val,val2;
  
  //loop over each channel
  for (int Ichan=0; Ichan < N_EEG_CHANNELS; Ichan++) {
    //get the EEG data for this channel, convert to float
    val=(float)ADS.channelData[Ichan]; 
  
    //apply DC-blocking highpass filter
    val = stopDC_filter.process(val,Ichan);    //apply DC-blocking filter
    
    //apply 60Hz notch filter...twice to make it more effective
    val = notch_filter1.process(val,Ichan);     //apply 60Hz notch filter
    val = notch_filter2.process(val,Ichan);     //apply 60Hz notch again
    
    //put this data back into ADSManager in case we want to output i tusing the ADSManager buit-in output functions
    //ADSManager.channelData[Ichan+2] = (long)val; //
    
    float val_common = val;
    for (int Iband=0;Iband<2;Iband++) {
      val=val_common;
      switch (Iband) {
        case (0):
          bp1 = &bandpass_filter1;
          bp2 = &bandpass_filter2;
          lp1 = &rms_lowpass_filter1;
          lp2 = &rms_lowpass_filter2; 
          break;
       case (1):       
          bp1 = &bandpass_filter3;
          bp2 = &bandpass_filter4;
          lp1 = &rms_lowpass_filter3;
          lp2 = &rms_lowpass_filter4; 
          break;
      }   
      
      //apply bandpass filter to focus on bandwidth of interest
      val = bp1->process(val,Ichan);    //apply bandpass filter
      val = bp2->process(val,Ichan);    //do it again to make it even tighter
      
      //put this value back into the ADSManager, too.  utilize upper channels, if available
      if ((N_EEG_CHANNELS==1) | (Iband==0)) { if (N_EEG_CHANNELS <= 4) ADS.channelData[Ichan+4+Iband] = (long)val;}
      //if (N_EEG_CHANNELS <= 4) ADSManager.channelData[Ichan] = (long)val; //no, put it on top of the raw data
      
      //return to processing...get the RMS value by squaring, lowpass filtering (a type of average), and sqrt
      val2 = val*val;  ///square the data
      val2 = lp1->process(val2,Ichan);  //low pass filter to take the "mean"
      //val2 = lp2->process(val2,Ichan);  //do it again for smoother mean    
      float EEG_rms = sqrt(abs(val2));  //take the square root to finally get the RMS
  
      switch (Iband) {
        case (0):
          EEG_inband_rms[Ichan] = EEG_rms;
          break;
        case (1):
          EEG_guard_rms[Ichan] = EEG_rms;
          break;
      }
    
      //put this value back into the ADSManager, too
      if ((N_EEG_CHANNELS==1) | (Iband==0)) if (N_EEG_CHANNELS <= 2) ADS.channelData[Ichan+6+Iband] = (long)EEG_rms;
    }
  }
}


float prev_eeg = 0.0f;



int updateLED(const float &EEG_inband_rms, const float &EEG_guard_rms, const int &prev_LED_val) {
    static const float smooth_EEG_fac = 0.95f;  //should be [0 1.0]...bigger is more smoothing
    const float max_uV_10x = 12.0f*10.0f; //this is the EEG amplitude (times 10!) causing maximum LED intensity
    const float min_uV_10x = 2.0f*10.0f;  //this is the EEG amplitude (times 10!) required for minimum LED instensity
    float eeg_inband_uV = EEG_inband_rms * MICROVOLTS_PER_COUNT;
    float eeg_guard_uV = EEG_guard_rms * MICROVOLTS_PER_COUNT;
    float eeg_uV;
    if (eeg_guard_uV >10.0f) {
      eeg_uV = 0.0f;
    } 
    else {
    //use these brightness rules when the guard EEG amplitude is smaller
      eeg_uV = eeg_inband_uV - eeg_guard_uV;
      eeg_uV -= 0.5f;  //penalize it a bit to reduce false alarms
      //eeg_uV = constrain(eeg_uV,-5.0f,+5.0f);
    }
    eeg_uV = smooth_EEG_fac * prev_eeg + (1.0f - smooth_EEG_fac)*eeg_uV;
    prev_eeg = eeg_uV;
    
    
    int int_val = (constrain(eeg_uV*10.0f,min_uV_10x,max_uV_10x));
    LED_val=(int)(map(int_val,(int)(min_uV_10x),(int)(max_uV_10x),MIN_LED_VAL,255));
    
    LED_val = constrain(LED_val,MIN_LED_VAL,255);
    Serial.print("eeg: ");Serial.println(LED_val);
    if(LED_val>30) digitalWrite(5,HIGH);
    else digitalWrite(5,LOW);
   // Blynk.virtualWrite(0,LED_val);
    //ledcWrite(LED_OUTPUT_PIN,LED_val);//(byte)constrain(LED_val,MIN_LED_VAL,255))
    return LED_val;
}


void  Contract_Fist(void){
   for (int posDegrees = 0; posDegrees <= 50; posDegrees++) {
 
    // Determine PWM pulse width
    pwm0 = map(posDegrees, 0, 50, SERVOMIN, SERVOMAX);
    pwm1 = map(posDegrees, 0, 50, SERVOMIN, SERVOMAX);
    pwm2 = map(posDegrees, 0, 50, SERVOMIN, SERVOMAX);
    pwm3 = map(posDegrees, 0, 50, SERVOMIN, SERVOMAX);
    pwm4 = map(posDegrees, 0, 50, SERVOMIN, SERVOMAX);
    
    // Write to PCA9685
    pca9685.setPWM(SER0, 0, pwm0);
    pca9685.setPWM(SER1, 0, pwm1);
    pca9685.setPWM(SER2, 0, pwm2);
    pca9685.setPWM(SER3, 0, pwm3);
    pca9685.setPWM(SER4, 0, pwm4);
    // Print to serial monitor
    Serial.print("Motor 0 = ");
    Serial.println(posDegrees);
    Serial.print("Motor 1 = ");
    Serial.println(posDegrees);
    Serial.print("Motor 2 = ");
    Serial.println(posDegrees);
    Serial.print("Motor 3 = ");
    Serial.println(posDegrees);
    Serial.print("Motor 4 = ");
    Serial.println(posDegrees);
    delay(30);
  }

  delay(2000);
}

void  Relax_Fist(void){

   for (int posDegrees = 50; posDegrees >= 0; posDegrees--) {
 
    // Determine PWM pulse width
    pwm0 = map(posDegrees, 0, 50, SERVOMIN, SERVOMAX);
    pwm1 = map(posDegrees, 0, 50, SERVOMIN, SERVOMAX);
    pwm2 = map(posDegrees, 0, 50, SERVOMIN, SERVOMAX);
    pwm3 = map(posDegrees, 0, 50, SERVOMIN, SERVOMAX);
    pwm4 = map(posDegrees, 0, 50, SERVOMIN, SERVOMAX);
    
    // Write to PCA9685
    pca9685.setPWM(SER0, 0, pwm0);
    pca9685.setPWM(SER1, 0, pwm1);
    pca9685.setPWM(SER2, 0, pwm2);
    pca9685.setPWM(SER3, 0, pwm3);
    pca9685.setPWM(SER4, 0, pwm4);
    // Print to serial monitor
    Serial.print("Motor 0 = ");
    Serial.println(posDegrees);
    Serial.print("Motor 1 = ");
    Serial.println(posDegrees);
    Serial.print("Motor 2 = ");
    Serial.println(posDegrees);
    Serial.print("Motor 3 = ");
    Serial.println(posDegrees);
    Serial.print("Motor 4 = ");
    Serial.println(posDegrees);
    delay(30);
  }

  delay(2000);
  
}
