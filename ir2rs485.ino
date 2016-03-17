// Simple IR-to-RS485 bridge for somfy motors.

// The MIT License (MIT)
//
// Copyright (c) 2016 Mark Baysinger
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

// Notes:
// 1) Requires a device with a second hardware serial port. This program will
//    compile without it, but will not have RS-485 functionality.
// 2) Attach IR LED to pin 9 for Arduino Mega. See IRremoteInt.h (from IRremote
//    library) to find the correct pin for other devices, or to change the pin.
// 3) Attach IR sensor to pin 11.
// 4) Attach RS-485 tranceiver DI/RO pins to Arduino pins 18/19 (tx1/rx1), and
//    attach RE/DE pins on the tranceiver (both) to pin 2 on the Arduino.
// 5) Debug commands that can be sent over serial port from computer:
//    * 's', followed by a variable-length string of hexadecimal digits send a
//      raw somfy message (see process_debug_somfy_raw() comments).
//    * 'm', followed by 18 hexadecimal digits, sends a Somfy motor command
//      (see process_debug_somfy_command() comments).
//    * 'i', followed by 4 hexadecimal digits, sends an IR command with the
//      two-byte payload represented by the hexadecimal digits.
//    All the above command must be followed by a carriage return ('\r'). Set
//    the Arduino serial monitor accordingly.
// 6) All RS-485 data received is relayed to the computer via serial in
//    hexadecimal.
// 7) Particular IR commands received will trigger this code to send particular
//    Somfy commands. Modify code to send the appropriate Somfy commands for
//    particular IR payloads.

#include <IRremote.h>
#include <stdint.h>

IRsend irsend;
IRrecv irrecv(11);
decode_results results;

#define DEBUG_COMMAND_MAX 20

#define WRITE_SOMFY_BYTE(c) checksum += c; Serial1.write(c);

// Send a raw somfy message.
// The data array contains entire message, except for the two checksum bytes at
// the end of the message. This function will calculate the checksum and add it
// for you automatically.
void write_somfy_raw(uint8_t *data, int len)
{
#if defined(HAVE_HWSERIAL1)
  int checksum = 0;

  // put the RS-485 tranceiver into send mode
  digitalWrite(2, HIGH);

  for (; len > 0; len--, data++) {
    WRITE_SOMFY_BYTE(*data);
  }
  
  // write checksum
  Serial1.write((checksum >> 8) & 0xFF);
  Serial1.write(checksum & 0xFF);

  Serial1.flush();

  // put the RS-485 tranceiver back into receive mode
  digitalWrite(2, LOW);
#endif
}

// sends somfy motor command over RS-485 (one type of somfy message)
void write_somfy_command(unsigned long uGroup, unsigned long uAddr, int cmd, int a, int b)
{
#if defined(HAVE_HWSERIAL1)
  int checksum = 0;

  // put the RS-485 tranceiver into send mode
  digitalWrite(2, HIGH);

  // header
  WRITE_SOMFY_BYTE(0xAB);
  WRITE_SOMFY_BYTE(0xF1);
  WRITE_SOMFY_BYTE(0xFF);

  // group addr (ingored if FF FF FF)
  WRITE_SOMFY_BYTE((uGroup >> 16) & 0xFF);
  WRITE_SOMFY_BYTE((uGroup >> 8) & 0xFF);
  WRITE_SOMFY_BYTE(uGroup & 0xFF);

  // motor addr (ingored if FF FF FF)
  WRITE_SOMFY_BYTE((uAddr >> 16) & 0xFF);
  WRITE_SOMFY_BYTE((uAddr >> 8) & 0xFF);
  WRITE_SOMFY_BYTE(uAddr & 0xFF);

  // command
  WRITE_SOMFY_BYTE(cmd);

  // params
  WRITE_SOMFY_BYTE(a);
  WRITE_SOMFY_BYTE(b);

  // write checksum
  Serial1.write((checksum >> 8) & 0xFF);
  Serial1.write(checksum & 0xFF);

  Serial1.flush();

  // put the RS-485 tranceiver back into receive mode
  digitalWrite(2, LOW);
#endif
}

void setup() {
  // initialize both serial ports:
  Serial.begin(9600);
#if defined(HAVE_HWSERIAL1)
  Serial1.begin(4800, SERIAL_8O1);
#endif
  pinMode(2, OUTPUT); // RS-485 write-enable 
  digitalWrite(2, LOW);
  irrecv.enableIRIn();
}

uint32_t hex2uint32(char *hex, int len) {
  // note: does not check for null termination, call accordingly!
  int32_t u = 0;
  for (int i = 0; i < len; i++) {
    u <<= 4;
    if (hex[i] >= '0' && hex[i] <= '9') {
      u |= hex[i] - '0';
    } else {
      u |= hex[i] - 'a' + 0xa;
    }
  }
  return u;
}

// Takes a parameter string from the user and sends the appropriate somfy message
// over RS-485. The parameter string is a set of lowercase hexadecimal digits,
// which represent the entire message, excluding the checksum bytes, which will
// be added automatically.
void process_debug_somfy_raw(char *param, int len)
{
  uint8_t *buf = (uint8_t *)malloc((len + 1) / 2);
  int buflen = 0;

  for (; len > 0; len -= 2, param += 2) {
    buf[buflen++] = (uint8_t)hex2uint32(param, len > 2 ? 2 : len);
  }
  
  Serial.println("Sending raw Somfy message over RS-485");
  write_somfy_raw(buf, buflen);

  free(buf);
}

// Takes a parameter string from the user and sends the appropriate somfy motor
// command. The paraemeter string is a set of 18 lowercase hexadecimal digits,
// which represent the following 9 bytes:
//  - first three bytes are group
//  - second three bytes are motor address
//  - 7th byte is command
//  - 8th and 9th bytes are parameters
void process_debug_somfy_command(char *param, int len)
{
  uint32_t group;
  uint32_t addr;
  uint8_t cmd;
  uint8_t a;
  uint8_t b;
  
  if (len < 18) {
    Serial.println("Error: parmeter too short!");
    return;
  }

  group = hex2uint32(param, 6);
  param += 6;
  addr = hex2uint32(param, 6);
  param += 6;
  cmd = hex2uint32(param, 2);
  param += 2;
  a = hex2uint32(param, 2);
  param += 2;
  b = hex2uint32(param, 2);
  param += 2;

  Serial.println("Sending Somfy command over RS-485:");
  Serial.println(group, HEX);
  Serial.println(addr, HEX);
  Serial.println(cmd, HEX);
  Serial.println(a, HEX);
  Serial.println(b, HEX);

  write_somfy_command(group, addr, cmd, a, b);
}

// Takes a parameter string from the user and sends the appropriate IR command.
// The paraemeter string is a set of 4 lowercase hexadecimal digits, which represent
// the two-byte payload of the command.
void process_debug_ir(char *param, int len) {
  uint16_t u;
  
  if (len < 4) {
    Serial.println("Error: parmeter too short!");
    return;
  }

  u = hex2uint32(param, 4);

  Serial.println("Sending IR command:");
  Serial.println(u, HEX);

  irsend.sendJVC(u, 16, 0);
  delay(10);
  for (int i = 0; i < 10; i++) {
    irsend.sendJVC(u, 16, 1);
    delay(10);
  }

  irrecv.enableIRIn();
}

void loop() {
  static char debug_command[DEBUG_COMMAND_MAX + 1]; // +1 for null termination
  static int debug_command_len = 0;
  
#if defined(HAVE_HWSERIAL1)
  static uint32_t uLastTime = 0;

  if (Serial1.available()) {
    int inByte = Serial1.read();
    uLastTime = millis();

    Serial.print(inByte, HEX);
    Serial.write(' ');
  }

  // Try to print newline between commands by detecting pause
  if ((uLastTime > 0) && (millis() > uLastTime + 10)) {
    Serial.println();
    uLastTime = 0;
  }
#endif

  if (Serial.available()) {
    int inByte = Serial.read();

    if ('\r' == inByte) {
      debug_command[debug_command_len] = 0;

      // 's' followed by string of hexadecimal digits, represending raw Somfy message
      // to send over RS-485
      if (debug_command_len > 0 && 's' == debug_command[0]) {
        process_debug_somfy_raw(debug_command + 1, debug_command_len - 1);
      }
      
      // 'm' followed by 18 hexadecimal digits (representing 9 bytes)
      //  send Somfy motor command over RS-485
      if (debug_command_len > 0 && 'm' == debug_command[0]) {
        process_debug_somfy_command(debug_command + 1, debug_command_len - 1);
      }

      // 'i' followed by 4 hexadecimal digits
      // send IR command
      if (debug_command_len > 0 && 'i' == debug_command[0]) {
        process_debug_ir(debug_command + 1, debug_command_len - 1);
      }

      debug_command_len = 0;
    } else if (debug_command_len < DEBUG_COMMAND_MAX) {
      debug_command[debug_command_len++] = inByte;
    }
  }

  if (irrecv.decode(&results)) {
    int nCmd = 0;
    unsigned long uGrp = 0xFFFFFF;
    unsigned long uAddr = 0xFFFFFF;
    Serial.print("Received IR code: ");
    Serial.print(results.decode_type, HEX);
    Serial.write(' ');
    Serial.print(results.value, HEX);
    Serial.write(' ');
    Serial.println(results.bits);
    /*
    */
    if (JVC == results.decode_type) {
      switch (results.value >> 8) {
      case 0x10:
         Serial.print("all ");
         uGrp = 0x323456;
         break;
      case 0x20:
         Serial.print("most ");
         uGrp = 0x123456;
         break;
      case 0x30:
        Serial.print("dining room ");
        uGrp = 0x223456;
        break;
      case 0x1:
        Serial.print("sliding door ");
        uAddr = 0xDBF6F9;
        break;
      case 0x2:
        Serial.print("dining room left ");
        uAddr = 0x39F6F9;
        break;
      case 0x3:
        Serial.print("dining room mid ");
        uAddr = 0xA4F5F9;
        break;
      case 0x4:
        Serial.print("dining room right ");
        uAddr = 0xE0F6F9;
        break;
      case 0x5:
        Serial.print("living room ");
        uAddr = 0xD4F6F9;
        break;
      case 0x6:
        Serial.print("stair ");
        uAddr = 0x14F1F9;
        break;
      default:
        Serial.print("unknown ");
      }
      switch (results.value & 0xff) {
      case 0x1:
        Serial.println("up");
        nCmd = 0xFE;
        break;
      case 0x2:
        Serial.println("down");
        nCmd = 0xFD;
        break;
      case 0x3:
        Serial.println("stop");
        nCmd = 0xFC;
        break;
      default:
        Serial.println("unknown");
      }
      if (nCmd != 0 && (uGrp != 0xFFFFFF || uAddr != 0xFFFFFF)) {
        /*
        Serial.println(uGrp, HEX);
        Serial.println(uAddr, HEX);
        Serial.println(nCmd, HEX);
        */
        write_somfy_command(uGrp, uAddr, nCmd, 0, 0);
      }
    }
    irrecv.resume(); // Receive the next value
  }

}

