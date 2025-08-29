//Receiver
//Link UART on Serial1 & USB debug on Serial

const char HANDSHAKE_REQUEST = 'R';
const char HANDSHAKE_ACK     = 'A';

#define LINK      Serial1
#define LINK_BAUD 9600
#define RX_PIN    18  //Change to photodiode output
#define TX_PIN    17  //Change to laser driver input

void setup() {
  Serial.begin(115200);
  LINK.begin(LINK_BAUD, SERIAL_8N1, RX_PIN, TX_PIN); //Initialises that UART for your optical communication link
  //SERIAL_8N1 sets frame format: 8 data bits, no parity, 1 stop bit. Standard UART format. RX_PIN is the GPIO for photodiode and TX_PIN is for laser diode driver
  Serial.println("Receiver Ready");
}

void loop() {
  //Checks to see if at least one byte is waiting in the optical UART's buffer receiver
  if (LINK.available() > 0) {
    //Reads the byte and assigns it to character, c, so it can be compared to 'R' or 'A'
    char c = (char)LINK.read();

    //Checks to see if the character, c, matches 'R'. If it does, it sends back an 'A' back to the transmitter
    if (c == HANDSHAKE_REQUEST) {
      Serial.println("Received 'R', sending 'A'");
      LINK.write(HANDSHAKE_ACK); 

      //FILL THIS PART WITH THE RECEIVER LOGIC CODE//
      }
    } 
  }

