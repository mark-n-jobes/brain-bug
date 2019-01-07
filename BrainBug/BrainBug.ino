//------------------------------------------------------------------------//
#include <SoftwareSerial.h>
// Serial
int rxPin = 2;
int txPin = 3; // Not used
SoftwareSerial SerialToRepeat(rxPin,txPin);
byte buff[3] = {0};
byte FrameData[37] = {0}; // 36 Data + 1 OF
byte Index = 0;
bool Framed = false;
bool Debug = true;
bool TurningENB = false;
// FSM
char FSM[3] = {0,0,0};
// Brain Values
byte Attention  = 0;
byte AttentionValues[8] = {0}; // For averaging
byte AvgAttention = 0;
byte Meditation = 0;
byte MeditationValues[8] = {0};// For averaging
byte AvgMeditation = 0;
byte AvgIndex = 0;
byte SigQuality = 0;
int32_t EEGValues[8] = {0};
//------------------------------------------------------------------------//
void setup() {
    // RX GND Reference -> MindFlex
    pinMode(4,OUTPUT);
    digitalWrite(4,LOW);
    // IR LEDS pinModes (use GND for GND)
    DDRB |= B1111; // Remember this is for bursting
    // Startup the two software serials
    Serial.begin(9600);
    Serial.println("BrainBug Started!");
    SerialToRepeat.begin(9600);
}
//------------------------------------------------------------------------//
void loop() {
    if(SerialToRepeat.available()) {   // Move to INTR..?
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
    // Check other Serial for CMDs
    if(Serial.available()) CrunchFSM(Serial.read());
}
//------------------------------------------------------------------------//
void CrunchFSM(char newChar) {
    if(newChar != 0) {
        for(int i=2;i>=0;i--) {
            if(i==0) FSM[i] = newChar;
            else     FSM[i] = FSM[i-1];
        }
        if((FSM[0]==FSM[2])&&(FSM[2]=='_')) {
            if(FSM[1]=='t') {
                TurningENB = !TurningENB;
                Serial.print("Turning is now:");
                Serial.println((TurningENB)?"ON":"OFF");
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
    byte offset = 4; // Where the EEG data starts (it's 8x24-bit numbers[accross 3bytes])
    SigQuality  = (200-FrameData[1])>>1; // 200 Seems to be the worst, so quality=0
    Attention   = FrameData[29];
    Meditation  = FrameData[31];
    // Calculate Averages
    AttentionValues[AvgIndex] = Attention;
    MeditationValues[AvgIndex] = Meditation;
    long TempAttention = 0;
    long TempMeditation = 0;
    for(byte i=0;i<8;i++) {
        TempAttention += (uint16_t)AttentionValues[i];
        TempMeditation += (uint16_t)MeditationValues[i];
    }
    AvgAttention  = TempAttention>>3;
    AvgMeditation = TempMeditation>>3;
    if(AvgIndex == 7) AvgIndex=0;
    else              AvgIndex++;
    // EEGValues
    for(byte i=0;i<8;i++) { // Stored in FrameData as 23:16,15:8,7:0 in index,index+,index++
        EEGValues[i]  = FrameData[i*3+offset+2];     //  7:0
        EEGValues[i] |= FrameData[i*3+offset+1]<<8;  // 15:8
        EEGValues[i] |= FrameData[i*3+offset+0]<<16; // 23:16
    }
    // Send Info
    if(Debug) {
        Serial.print("Qual:");Serial.print(SigQuality);
        Serial.print(" Attn:");Serial.print(Attention);
        Serial.print(" AvgAttn:");Serial.print(AvgAttention);
        Serial.print(" Medt:");Serial.print(Meditation);
        Serial.print(" AvgMedt:");Serial.print(AvgMeditation);
        Serial.print(" EEGValues:"); // 0->7 is: {Delta, Theta, LowAlpha, HighAlpha, LowBeta, HighBeta, LowGamma, HighGamma}
        for(byte i=0;i<8;i++) {
            Serial.print(EEGValues[i]);
            if(i!=7) Serial.print("_");
            else     Serial.println(" ");
        }
    }
    // Send InchCode
    if(SigQuality == 100) {
        if((Attention > AvgAttention+10)||(Attention > 90)) {
            Xmit_InchCode('F');
            Serial.println("Move Forward");
        } else if((Attention < AvgAttention-10)||(Attention < 20)) {
            Xmit_InchCode('B');
            Serial.println("Move Backward");
        }
        if(TurningENB) {
            if(Meditation > AvgMeditation+15) {
                Xmit_InchCode('R');
                Serial.println("Move Right");
            } else if(Meditation < AvgMeditation-15) {
                Xmit_InchCode('L');
                Serial.println("Move Left");
            }
        }
    }
}
//------------------------------------------------------------------------//
void Xmit_InchCode(char Code) {
    int CodeArray[1] = {0};
    if      (Code == 'F') CodeArray[0] = 131;
    else if (Code == 'B') CodeArray[0] = 69; // heheh
    else if (Code == 'R') CodeArray[0] = 38;
    else if (Code == 'L') CodeArray[0] = 193;
    // Header
    pulseIR_38KHz(4000); delayMicroseconds(400);
    pulseIR_38KHz(2000); delayMicroseconds(800);
    // Body
    sendCharArray(CodeArray,1,400,1400,500);
    // Footer
    pulseIR_38KHz(400); delayMicroseconds(1400);
    pulseIR_38KHz(400); delayMicroseconds(1400);
    pulseIR_38KHz(400);
    delay(65);
}
//------------------------------------------------------------------------//
void sendCharArray(int CharArray[], uint16_t size, uint16_t PulseOn, uint16_t DelayTrue, uint16_t DelayFalse) {
    int tempc = 0;
    for(int i=0;i < size;i++) {
        tempc = CharArray[i];
        for(int j=7;j>=0;j--) {
            pulseIR_38KHz(PulseOn);
            if (((tempc>>j)%2) == 1) delayMicroseconds(DelayTrue);
            else                     delayMicroseconds(DelayFalse);
        }
    }
}
//------------------------------------------------------------------------//
void pulseIR_38KHz(long microsecs) {
    while (microsecs > 0) {
        // 38KHz = 26.316us -> 26us, so cut in 1/2 for IR pulse -> 13us high, 13us low
        PORTB = B1111;  // Takes ~1/2us
        delayMicroseconds(13);
        PORTB = B0000;  // Takes ~1/2us
        delayMicroseconds(12);
        microsecs -= 26;
    }
}
//------------------------------------------------------------------------//