# Example program for ESPAVRISP.py
# Copyright (c) tab4moji <tab4moji@gmail.com>
#
# Original version:
#
#     AVR In-System Programming over WiFi for ESP8266
#     Copyright (c) Kiril Zyapkov <kiril@robotev.com>
#         https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266AVRISP

# *****
# before executing this program, you should connect your ESP with WiFi AP.
# *****

import ESPAVRISP

# ESPAVRISP.avrisp_debug = True
avrprog = ESPAVRISP.ESPAVRISP(328, 5, 150000)

def setup():
    # listen for avrdudes
    avrprog.begin()
    return

last_state = ESPAVRISP.AVRISP_STATE_IDLE

def loop():
    global last_state

    new_state = avrprog.update()

    if last_state != new_state:
        if new_state == ESPAVRISP.AVRISP_STATE_IDLE:
            print("[AVRISP] now idle")
            # Use the SPI bus for other purposes
        elif new_state == ESPAVRISP.AVRISP_STATE_PENDING:
            print("[AVRISP] connection pending")
            # Clean up your other purposes and prepare for programming mode
        elif new_state == ESPAVRISP.AVRISP_STATE_ACTIVE:
            print("[AVRISP] programming mode")
            # Stand by for completion

        last_state = new_state

    # Serve the client
    if last_state != ESPAVRISP.AVRISP_STATE_IDLE:
        avrprog.serve()

    return

def main():
    setup()
    while True:
        loop()
    return

