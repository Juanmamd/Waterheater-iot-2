/*
Sending formatted data packet with nRF24L01.
Maximum size of data struct is 32 bytes.

1 - GND
2 - VCC 3.3V !!! NOT 5V
3 - CE to Arduino pin 9
4 - CSN to Arduino pin 10
5 - SCK to Arduino pin 13
6 - MOSI to Arduino pin 11
7 - MISO to Arduino pin 12
8 - UNUSED

*/

#include <EEPROM.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// For printf support
int serial_putc( char c, FILE * )
{
  Serial.write( c );
  return c;
}

const int LED_VERDE=2;
const int LED_ROJO=3;
const uint64_t pipe = 0xE8E8F0F0E1LL;
boolean sender;

RF24 radio(9, 10);

// The sizeof this struct should not exceed 32 bytes
struct MyData {
  byte byte0;
  byte byte1;
  byte byteArray[2];
  int int0;
  int int1;
  int intArray[2];
  float float0;
  float float1;
  float floatArray[2];
  unsigned long packetId; // 16 month rollover at 100 packets/sec
};

/**************************************************/



// Print some info to serial after this number of packets
unsigned long printRate = 500;

void setup()
{
  Serial.begin(115200);
  fdevopen( &serial_putc, 0 ); // for printf

  // Change this when uploading to each Arduino

// Ultima configuracion guardada en EEPROM
  int sender_int = EEPROM.read(0);
  if ( sender_int == 1 ){
      sender=true;
  }else if (sender_int == 2){
    sender=false;
  }else {
    sender = true;
    EEPROM.write(0,1);
    sender = false;
//    EEPROM.write(0,2);
//    printf("Initial setting\n");
    }

  // Set up radio module
  radio.begin();
  radio.setDataRate(RF24_250KBPS); // Both endpoints must have this set the same
  radio.setAutoAck(true);       // Either endpoint can set to false to disable ACKs
  radio.setPALevel(RF24_PA_MAX); // Power amplifier level. Also LOW,MED,HIGH (default is HIGH)
  radio.setChannel(0x66);
  radio.printDetails();

  if ( sender )
    radio.openWritingPipe(pipe);
  else {
    radio.openReadingPipe(1,pipe);
    radio.startListening();
  }

  int dataSize = sizeof(MyData);
  printf("Size of MyData: %d\n", dataSize);
  if ( dataSize > 32 )
    printf("*** MyData struct is too large ***\n");

  printf("This Arduino is set to be a %s\n", sender ? "sender" : "receiver");
}

/**************************************************/

unsigned long nextPacketId = 0;   // Stamp packets with their sequence
float floatSensor = 0;            // A fake 'sensor' so we can see some data changing
unsigned long unAckedPackets = 0; // To estimate how many packets were not ACKed
unsigned long lastPrintTime = 0;  // To measure time between printouts

void sendData()
{
  MyData data;

  // Set properties of your data here...
  data.float0 = floatSensor;
  data.packetId = nextPacketId;

  // Record a before and after time to see how long the write takes
  unsigned long beforeWrite = millis();

  radio.write(&data, sizeof(MyData));

  unsigned long afterWrite = millis();

  // Although the RF24 documentation states that the timeout for
  // receiving an ACK is 60ms, in my experience it seems to wait
  // for only about 2-3ms. The value 2 here was chosen by printing
  // out the time taken for 1000 writes when the receiver was non
  // existent (ie. all packets dropped) and this was around 2788ms
  // between two Arduino UNOs. So I figure that any writes taking
  // 3ms or more have likely been abandoned without receiving any
  // ACK. Note that this does not mean the packet was not actually
  // received at the other end, it just means that the sender could
  // not confirm that the packet arrived.
  // MOdificado de 2 a 6 ms
  if ( afterWrite - beforeWrite > 20 ) {
    unAckedPackets++;
    // Activar led rojo
    digitalWrite(LED_ROJO, HIGH);
    // Desactivar led verde
    digitalWrite(LED_VERDE, LOW);
  } else {
    // Desactivar led rojo
    digitalWrite(LED_ROJO, LOW);
    // ACtivar led verde
    digitalWrite(LED_VERDE,HIGH);
  }

  // Every n packets, print out some info. // Imprimir cada "printRate" número de paquetes
  if ( data.packetId % printRate == 0 ) {

    if ( data.packetId > 0 ) {

      // Use bytes/msec as an approximate kbytes/sec value
      unsigned long now = millis();
      unsigned long packetsec = ((printRate - unAckedPackets)*1000) / (now - lastPrintTime);
      int kbsec = (sizeof(MyData) * (printRate - unAckedPackets)) / (now - lastPrintTime);
      lastPrintTime = now;

      // Format the float for printing
      char buf[16];
      dtostrf(data.float0, 0, 6, buf);

      // As mentioned above, the dropped count is only a rough estimate.
      // You may see some non-zero values even when no packets were really
      // dropped, but it's not bad as a rough guide. You may need to alter
      // the value 2 used above depending on the speed of the microprocessors
      // you are using, to get a better estimate.
      printf("Wrote %ld packets, last id = %ld, float0 = %s, unAcked %ld, %ld packets/sec (%d kbyte/sec)\n", printRate, data.packetId, buf, unAckedPackets, packetsec, kbsec);
    }

    unAckedPackets = 0;
  }

  // Increment the packet id for next time
  nextPacketId++;

  // Change the 'sensor' value so we can check floats are working
  floatSensor += 0.000001 * random(1000);
}

/**************************************************/

unsigned long packetsRead = 0;      // Counts total packets read
unsigned long expectedPacketId = 0; // To track whether the packet id was what we expected
int missedPackets = 0;              // How many packets missed since last printout

void recvData()
{
  if ( radio.available() ) {
    MyData data;
    radio.read(&data, sizeof(MyData));
    packetsRead++;

    // Check if the packet id was what we expected. Unlike for the sender
    // this is not an estimate, it's a known value because we can look at
    // incoming packet ids to see how many we missed.
    if ( expectedPacketId > 0 ){
      missedPackets += data.packetId - expectedPacketId;
      if(missedPackets > 0) {
      // Activar led rojo
      digitalWrite(LED_ROJO,HIGH);
      // Desactivar led verde
      digitalWrite(LED_VERDE,LOW);
    }else {
      //Activar led verde
      digitalWrite(LED_VERDE,HIGH);
      // Desactivar led rojo
      digitalWrite(LED_ROJO, LOW);
      }
    }else{
      // Activar led rojo
      digitalWrite(LED_ROJO,HIGH);
      // Desactivar led verde
      digitalWrite(LED_VERDE,LOW);
      }

    // Set the packet id we will expect next time
    expectedPacketId = data.packetId + 1;

    // Every n packets, print out some info
    if ( data.packetId > 0 && packetsRead % printRate == 0 ) {

      // Use bytes/msec as an approximate kbytes/sec value
      unsigned long now = millis();
      unsigned long packetsec = ((printRate - unAckedPackets) * 1000) / (now - lastPrintTime);
      int kbsec = (sizeof(MyData) * printRate) / (now - lastPrintTime);
      lastPrintTime = now;

      // Format the float for printing
      char buf[16];
      dtostrf(data.float0, 0, 6, buf);

      printf("Read %ld packets, last id = %ld, float0 = %s, skipped = %d, %ld packets/sec  (%d kbyte/sec)\n", printRate, data.packetId, buf, missedPackets, packetsec, kbsec);

      missedPackets = 0;
    }
  }
}

/**************************************************/

void loop()
{
  if ( sender ){
    sendData();
//    Serial.print("Enviar\n");
  }else{
    recvData();
//    Serial.print("Recibir\n");
  }
}
