//------------------------------------------------------------------------//
#include <SoftwareSerial.h>
int rxPin = 2;
int txPin = 3; // Not used
SoftwareSerial SerialToRepeat(rxPin,txPin);
byte buff[3] = {0};
byte FrameData[37] = {0}; // 36 Data + 1 OF
byte Index = 0;
bool Framed = false;
bool Debug = false;
//------------------------------------------------------------------------//
void setup() {
    pinMode(4,OUTPUT);
    digitalWrite(4,LOW); // RX GND ref
    Serial.begin(9600);
    SerialToRepeat.begin(9600);
}
//------------------------------------------------------------------------//
void loop() {
    if (SerialToRepeat.available()) {
        byte valueOut = SerialToRepeat.read();
        // Capture if framed
        if(Framed) {
            FrameData[Index] = valueOut;
            if(Index < 36) Index++; // Gets cleared by new Frame, sticks at OF index
        }
        // Cycle buffer
        for(int i=2;i>=0;i--) buff[i] = (i==0)? valueOut : buff[i-1];
        // Frame
        if((buff[0]==32)&&(buff[1]==170)&&(buff[2]==170)) {
            for(uint8_t i=0;i<3;i++) buff[i]=0; // To be safe
            Index = 0;
            if(!Framed) Framed = true;
            else {
                if(CheckFrame()) ProcessAndSendFrameData();
                else {
                    Framed = false;
                    Serial.println("\nFrame ERROR");
                }
            }
        }
    }
}
//------------------------------------------------------------------------//
bool CheckFrame() {
    bool toReturn = true; // assume pass and mask out to fail
    toReturn &= (FrameData[35] == (byte) 32); // Header2
    toReturn &= (FrameData[34] == (byte)170); // Header1
    toReturn &= (FrameData[33] == (byte)170); // Header0
    toReturn &= (FrameData[ 0] == (byte)  2); // Signal Quality ID
    toReturn &= (FrameData[ 2] == (byte)131); // EEG Array ID
    toReturn &= (FrameData[ 3] == (byte) 24); // Number bits per EEG value
    toReturn &= (FrameData[28] == (byte)  4); // Attention ID
    toReturn &= (FrameData[30] == (byte)  5); // Meditation ID
    // Check data sum
    uint8_t CheckSum = 0;
    for(byte i=0;i<36;i++) if(i!=32) CheckSum += FrameData[i];
    if(CheckSum != (byte)(115-FrameData[32])) { // TODO: Figure out why it's 115
        Serial.println("\nChecksum Failure!");
        return false;
    }
    return toReturn;
}
//------------------------------------------------------------------------//
void ProcessAndSendFrameData() {
    // Process
    byte offset     = 4; // Where the EEG data starts (it's 8x24-bit numbers[accross 3bytes])
    byte Attention  = FrameData[29];
    byte Meditation = FrameData[31];
    byte SigQuality = (200-FrameData[1])>>1; // 200 Seems to be the worst, so quality=0
    int32_t EEGValues[8] = {0};
    for(byte i=0;i<8;i++) { // Stored in FrameData as 23:16,15:8,7:0 in index,index+,index++
        EEGValues[i] |= FrameData[i*3+offset+0]<<16; // 23:16
        EEGValues[i] |= FrameData[i*3+offset+1]<<8;  // 15:8
        EEGValues[i] |= FrameData[i*3+offset+2];     //  7:0
    }
    // Send
    if(Debug) Serial.print("Qual:");
    Serial.print(SigQuality);
    if(Debug) Serial.print(" Attn:");
    else      Serial.print("_");
    Serial.print(Attention);
    if(Debug) Serial.print(" Medt:");
    else      Serial.print("_");
    Serial.print(Meditation);
    if(Debug) Serial.print(" EEGValues:"); // 0->7 is: {Delta, Theta, LowAlpha, HighAlpha, LowBeta, HighBeta, LowGamma, HighGamma}
    else      Serial.print("_");
    for(byte i=0;i<8;i++) {
        Serial.print(EEGValues[i]);
        if(i!=7) Serial.print("_");
        else     Serial.println(" ");
    }
}
//------------------------------------------------------------------------//

