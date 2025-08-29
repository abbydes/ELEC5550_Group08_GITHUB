//Transmitter
//Link UART on Serial1 & USB debug on Serial

const char HANDSHAKE_REQUEST = 'R';
const char HANDSHAKE_ACK     = 'A';

#define LINK     Serial1
#define LINK_BAUD 9600
#define RX_PIN   18   //Change to photodiode output
#define TX_PIN   17   //Change to laser driver input

void setup() {
  Serial.begin(115200);
  //Optical link UART
  LINK.begin(LINK_BAUD, SERIAL_8N1, RX_PIN, TX_PIN); // Initialises that UART for your optical communication link
  //SERIAL_8N1 sets frame format: 8 data bits, no parity, 1 stop bit. Standard UART format. RX_PIN is the GPIO for photodiode and TX_PIN is for laser diode driver
  Serial.println("Transmitter Ready");
}
//Acknowledgement function: Returns 'true' if ACK is received on time, else, 'false'. 
//expected = byte value it is expecting back from the receiver ('A')
//timeout_ms = maximum time to wait (ms)
bool waitForAck(uint8_t expected, uint32_t timeout_ms) {
  uint32_t t0 = millis(); // Records current time in ms

  //Time out condition (sees if time difference between current and initial time within timeout time)
  while (millis() - t0 < timeout_ms) { 
    //LINK.available() - Checks to see if at least one byte is waiting in the optical UART's buffer receiver
    if (LINK.available() && (uint8_t)LINK.read() == expected) return true; //Reads the one byte from the buffer receiver to see if it matches with the expected (ie. 'A')
    }
  return false;
}

void loop() {
  //Clear any stray bytes (initialises receive buffer in the UART)
  while (LINK.available()) LINK.read();

  //Send handshake request on the optical link 'R'
  LINK.write(HANDSHAKE_REQUEST);
  Serial.println("Sent handshake request 'R'");

  //Wait for 'A'
  if (waitForAck(HANDSHAKE_ACK, 1000)) { //Waits up to 1000ms to receive 'A' from receiver. In this case, HANDSHAKE_ACK = 'A' & timeout_ms = 1000ms
    Serial.println("Handshake successful!");
  
    //THIS IS WHERE YOU PLACE ALL THE DATA TRANSFER CODE. THE REST OF THE SOFTWARE OPERATIONS //
  } else {
    Serial.println("Handshake failed: timeout");
    delay(200);
  }
  delay(500); 
}


