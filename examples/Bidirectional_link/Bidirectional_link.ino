#include <Arduino.h>
#include <cc1101.h>


using namespace CC1101;

Radio radio(/* cs pin */ 10);

uint8_t tx_channel = 2; // switch tx and rx channels on the second device to test cross-channel communication
uint32_t rx_channel = 1;
uint8_t device_id = 0x99; // set to 0 for broadcast, or any other value for address filtering of incoming packets


const uint32_t tx_delay = 3000;
uint32_t _tx_delay = tx_delay; // variable tx delay 
uint32_t tx_timer = 0;
int tx_counter = 0;



void setup() {
    Serial.begin(115200);
    delay(3000);
    Serial.println(F("Starting ..."));
    delay(1000);


    if (radio.begin() == STATUS_CHIP_NOT_FOUND) {
        Serial.println(F("Chip not found!"));
        while (true) {
            delay(5000);
            Serial.println(F("Chip not found!"));
        }
    }

    radio.setModulation(MOD_ASK_OOK);
    radio.setFrequency(433.97);
    radio.setDataRate(10);
    radio.setOutputPower(10);
    //radio.setRxBandwidth(200);
    if (radio.setFrequencyDeviation(25) != STATUS_OK) {  // no effect in MOD_ASK_OOK
        Serial.println(F("Invalid frequency deviation!"));
    }

    radio.setPacketLengthMode(PKT_LEN_MODE_VARIABLE);
    radio.setAddressFilteringMode(ADDR_FILTER_MODE_CHECK_BC_0); // address filtering with 0 as broadcast address, so that we can test both unicast and broadcast communication
    radio.setPreambleLength(64);
    radio.setSyncWord(0x1234);
    radio.setSyncMode(SYNC_MODE_16_16);
    radio.setCrc(true);
    radio.setDataWhitening(true);
    radio.setManchester(false);
    radio.setFEC(false);
}

void loop() {
    rx_data(100);

    if ((millis() - tx_timer) > tx_delay) {
        tx_data();
        tx_timer = millis();
    }
}


float readTemperature() {
    return 0.0; //dummy value
}



void tx_data() {
    float temp = readTemperature();
    String data = "Hello#" + String(tx_counter++) + " from ch " + String(tx_channel) + " temp " + String(temp, 2) + "°C";

    Serial.print(F("Transmitting: "));
    Serial.print(data);
    Serial.print(F(" "));
    radio.setChannel(tx_channel);
    Status status = radio.transmit(reinterpret_cast<const uint8_t *>(data.c_str()), data.length(), device_id);
    if (status == STATUS_OK) {
        Serial.println(F("[OK]"));
    } else {
        Serial.print(F("[ERROR "));
        Serial.print(status);
        Serial.println(F("]"));
    }

    _tx_delay = tx_delay - (temp * 10); // decrease tx delay as temperature increases, just so that all nodes don't transmit at the same time
}

void rx_data(uint32_t timeout) {
    char buff[256];
    size_t read;

    radio.setChannel(rx_channel);
    Status status = radio.receive(reinterpret_cast<uint8_t *>(buff), sizeof(buff) - 1, &read, device_id, timeout);

    if (status == STATUS_OK) {
        buff[read] = '\0';

        Serial.print(F("Data: "));
        Serial.println(buff);

        Serial.print(F("Length: "));
        Serial.println(read);

        Serial.print(F("RSSI: "));
        Serial.print(radio.getRSSI());
        Serial.println(F(" dBm"));

        Serial.print(F("LQI: "));
        Serial.println(radio.getLQI());
        Serial.println();
    } else if (status == STATUS_CRC_MISMATCH) {
        Serial.println(F("CRC mismatch!"));
    } else if (status != STATUS_TIMEOUT) {
        Serial.print(F("Error: "));
        Serial.println(status);
    }
}
