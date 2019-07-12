# ESPAVRISP.py : AVR In-System Programming over WiFi for MicroPython on ESP8266/ESP32
# Copyright (c) tab4moji <tab4moji@gmail.com>
#
# Original version:
#
#     AVR In-System Programming over WiFi for ESP8266
#     Copyright (c) Kiril Zyapkov <kiril@robotev.com>
#         https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266AVRISP
#
#     ArduinoISP version 04m3
#     Copyright (c) 2008-2011 Randall Bohn
#     If you require a license, see
#         http://www.opensource.org/licenses/bsd-license.php

import os
import socket
import gc
import machine
import time

AVRISP_STATE_IDLE         = 0 # no active TCP session
AVRISP_STATE_PENDING      = 1 # TCP connected, pending SPI activation
AVRISP_STATE_ACTIVE       = 2 # programmer is active and owns the SPI bus

STK_SIGN_ON_MESSAGE       = "AVR STK" # Sign on string for Cmnd_STK_GET_SIGN_ON
Resp_STK_OK               =  b'\x10'  # ' '
Resp_STK_FAILED           =  b'\x11'  # ' '
Resp_STK_UNKNOWN          =  b'\x12'  # ' '
Resp_STK_NODEVICE         =  b'\x13'  # ' '
Resp_STK_INSYNC           =  b'\x14'  # ' '
Resp_STK_NOSYNC           =  b'\x15'  # ' '
Resp_ADC_CHANNEL_ERROR    =  b'\x16'  # ' '
Resp_ADC_MEASURE_OK       =  b'\x17'  # ' '
Resp_PWM_CHANNEL_ERROR    =  b'\x18'  # ' '
Resp_PWM_ADJUST_OK        =  b'\x19'  # ' '
Sync_CRC_EOP              =  0x20  # 'SPACE'
Cmnd_STK_GET_SYNC         =  0x30  # ' '
Cmnd_STK_GET_SIGN_ON      =  0x31  # ' '
Cmnd_STK_RESET            =  0x32  # ' '
Cmnd_STK_SINGLE_CLOCK     =  0x33  # ' '
Cmnd_STK_STORE_PARAMETERS =  0x34  # ' '
Cmnd_STK_SET_PARAMETER    =  0x40  # ' '
Cmnd_STK_GET_PARAMETER    =  0x41  # ' '
Cmnd_STK_SET_DEVICE       =  0x42  # ' '
Cmnd_STK_GET_DEVICE       =  0x43  # ' '
Cmnd_STK_GET_STATUS       =  0x44  # ' '
Cmnd_STK_SET_DEVICE_EXT   =  0x45  # ' '
Cmnd_STK_ENTER_PROGMODE   =  0x50  # ' '
Cmnd_STK_LEAVE_PROGMODE   =  0x51  # ' '
Cmnd_STK_CHIP_ERASE       =  0x52  # ' '
Cmnd_STK_CHECK_AUTOINC    =  0x53  # ' '
Cmnd_STK_CHECK_DEVICE     =  0x54  # ' '
Cmnd_STK_LOAD_ADDRESS     =  0x55  # ' '
Cmnd_STK_UNIVERSAL        =  0x56  # ' '
Cmnd_STK_PROG_FLASH       =  0x60  # ' '
Cmnd_STK_PROG_DATA        =  0x61  # ' '
Cmnd_STK_PROG_FUSE        =  0x62  # ' '
Cmnd_STK_PROG_LOCK        =  0x63  # ' '
Cmnd_STK_PROG_PAGE        =  0x64  # ' '
Cmnd_STK_PROG_FUSE_EXT    =  0x65  # ' '
Cmnd_STK_READ_FLASH       =  0x70  # ' '
Cmnd_STK_READ_DATA        =  0x71  # ' '
Cmnd_STK_READ_FUSE        =  0x72  # ' '
Cmnd_STK_READ_LOCK        =  0x73  # ' '
Cmnd_STK_READ_PAGE        =  0x74  # ' '
Cmnd_STK_READ_SIGN        =  0x75  # ' '
Cmnd_STK_READ_OSCCAL      =  0x76  # ' '
Cmnd_STK_READ_FUSE_EXT    =  0x77  # ' '
Cmnd_STK_READ_OSCCAL_EXT  =  0x78  # ' '
Parm_STK_HW_VER           =  0x80  # ' ' - R
Parm_STK_SW_MAJOR         =  0x81  # ' ' - R
Parm_STK_SW_MINOR         =  0x82  # ' ' - R
Parm_STK_LEDS             =  0x83  # ' ' - R/W
Parm_STK_VTARGET          =  0x84  # ' ' - R/W
Parm_STK_VADJUST          =  0x85  # ' ' - R/W
Parm_STK_OSC_PSCALE       =  0x86  # ' ' - R/W
Parm_STK_OSC_CMATCH       =  0x87  # ' ' - R/W
Parm_STK_RESET_DURATION   =  0x88  # ' ' - R/W
Parm_STK_SCK_DURATION     =  0x89  # ' ' - R/W
Parm_STK_BUFSIZEL         =  0x90  # ' ' - R/W, Range {0..255}
Parm_STK_BUFSIZEH         =  0x91  # ' ' - R/W, Range {0..255}
Parm_STK_DEVICE           =  0x92  # ' ' - R/W, Range {0..255}
Parm_STK_PROGMODE         =  0x93  # ' ' - 'P' or 'S'
Parm_STK_PARAMODE         =  0x94  # ' ' - TRUE or FALSE
Parm_STK_POLLING          =  0x95  # ' ' - TRUE or FALSE
Parm_STK_SELFTIMED        =  0x96  # ' ' - TRUE or FALSE
Stat_STK_INSYNC           =  0x01  # INSYNC status bit, '1' - INSYNC
Stat_STK_PROGMODE         =  0x02  # Programming mode,  '1' - PROGMODE
Stat_STK_STANDALONE       =  0x04  # Standalone mode,   '1' - SM mode
Stat_STK_RESET            =  0x08  # RESET button,      '1' - Pushed
Stat_STK_PROGRAM          =  0x10  # Program button, '   1' - Pushed
Stat_STK_LEDG             =  0x20  # Green LED status,  '1' - Lit
Stat_STK_LEDR             =  0x40  # Red LED status,    '1' - Lit
Stat_STK_LEDBLINK         =  0x80  # LED blink ON/OFF,  '1' - Blink

AVRISP_HWVER = 2
AVRISP_SWMAJ = 1
AVRISP_SWMIN = 18
AVRISP_PTIME = 10
EECHUNK = 32

V_LOW  = 0    # from Arduino
V_HIGH = 1    # from Arduino

avrisp_debug = False
def AVRISP_DEBUG(x):
    if avrisp_debug:
        print(x)
    else:
        pass

# stk500 parameters
# devicecode;
# revision;
# progtype;
# parmode;
# polling;
# selftimed;
# lockbytes;
# fusebytes;
# flashpoll;
# eeprompoll;
# pagesize;
# eepromsize;
# flashsize;
param = {}

ESP_HSPI = 1
ESP_MISO = 12
ESP_MOSI = 13
ESP_SCK  = 14

class ESPAVRISP:

    def __init__(self, port, reset_pin, spi_freq = 150000, reset_state = False, reset_activehigh = False):
        self.buff = bytearray(256)
        self.error = 0
        self.pmode = 0
        self.here = 0

        # tcp connected or not :
        self.client_connected = False
        # micropython's socket doesn't have any exception.
        # XXX unix port has exceptions.

        try:
            # 'esp32' == os.uname().sysname
            self.SPI = machine.SPI(ESP_HSPI, baudrate = spi_freq, sck = machine.Pin(ESP_SCK), mosi = machine.Pin(ESP_MOSI), miso = machine.Pin(ESP_MISO))
        except:
            # SPI 'esp8266' == os.uname().sysname
            self.SPI = machine.SPI(ESP_HSPI, baudrate = spi_freq, polarity = 0, phase = 0)

        #_spi_freq(spi_freq)
        self._spi_freq = spi_freq

        # _server(WiFiServer(port))
        port_addr = socket.getaddrinfo('0.0.0.0', port)[0][-1]
        self._server = socket.socket()
        self._server.bind(port_addr)

        # _state(AVRISP_STATE_IDLE)
        self._state = AVRISP_STATE_IDLE

        # _reset_pin(reset_pin)
        self._reset_pin = reset_pin

        # _reset_state(reset_state)
        self._reset_state = reset_state

        # _reset_activehigh(reset_activehigh)
        self._reset_activehigh = reset_activehigh

        self.pin_reset = machine.Pin(self._reset_pin)
        self.pin_reset.init(mode = machine.Pin.OUT)
        self.setReset(self._reset_state)
        return

    def begin(self):
        self._server.listen(1)
        return

    def setSpiFrequency(self, freq):
        self._spi_freq = freq
        if self._state == AVRISP_STATE_ACTIVE:
            self.SPI.init(baudrate = freq)
        return

    def setReset(self, rst):
        self._reset_state = rst
        self.digitalWrite(self._reset_pin, self._resetLevel(self._reset_state))
        return

    def update(self):

        if self._state == AVRISP_STATE_IDLE:
            self._client, self.addr = self._server.accept()
            self.client_connected = True
            # XXX self._client.setNoDelay(True);
            # XXX AVRISP_DEBUG("client connect %s:%d", self._client.remoteIP().toString().c_str(), self._client.remotePort());
            AVRISP_DEBUG(self.addr)
            self._client.settimeout(10) # for self.getch()
            self._state = AVRISP_STATE_PENDING
            self._reject_incoming()

        elif self._state == AVRISP_STATE_PENDING or self._state == AVRISP_STATE_ACTIVE:
            # handle disconnect
            if not self.client_connected:
                self._client.close()
                self.client_connected = False
                gc.collect()
                AVRISP_DEBUG("client disconnect")
                if self.pmode:
                    self.SPI.deinit()
                    self.pmode = False
                self.setReset(self._reset_state)
                self._state = AVRISP_STATE_IDLE
            else:
                self._reject_incoming()

        return self._state

    def serve(self):
        res = self.update()

        if res == AVRISP_STATE_IDLE:
            # should not be called when idle, error?
            pass
        else:
            if res == AVRISP_STATE_PENDING or res == AVRISP_STATE_ACTIVE:
                if res == AVRISP_STATE_PENDING:
                    self._state = AVRISP_STATE_ACTIVE
                else: # should we do fallthrogh ?
                    while self.client_connected:
                        gc.collect()
                        self.avrisp()
                        gc.collect()

                # should we do return self.update() ?

        return self._state

    def _reject_incoming(self):
        # micropython doesn't need this.
        # while (_server.hasClient()) _server.available().stop();
        return

    def getch(self):
        # while not self.client_connected:
        #     pass

        try:
            byte = self._client.read(1)
        except:
            byte = bytes([0x00])

        return byte[0]

    def fill(self, n):
        AVRISP_DEBUG("fill(" + str(n) + ")")
        for x in range(n):
            self.buff[x] = self.getch();
        return

    def spi_transaction(self, a, b, c, d):
        spi_recvbuf = bytearray(1)
        self.SPI.write(bytearray([a]))
        self.SPI.write(bytearray([b]))
        self.SPI.write(bytearray([c]))
        self.SPI.write_readinto(bytearray([d]), spi_recvbuf)
        return spi_recvbuf

    def empty_reply(self):
        if Sync_CRC_EOP == self.getch():
            self._client.send(Resp_STK_INSYNC)
            self._client.send(Resp_STK_OK)
        else:
            self._client.send(Resp_STK_NOSYNC)
            self.error += 1
        return

    def breply(self, b):
        if Sync_CRC_EOP == self.getch():
            resp = bytearray([0x12,0x34,0x56])
            resp[0] = Resp_STK_INSYNC[0]
            resp[1] = b
            resp[2] = Resp_STK_OK[0]
            self._client.send(resp)
        else:
            self.error += 1
            self._client.send(Resp_STK_NOSYNC)
        return

    def get_parameter(self, c):
        if c == 0x80:
            self.breply(AVRISP_HWVER)
        elif c == 0x81:
            self.breply(AVRISP_SWMAJ)
        elif c == 0x82:
            self.breply(AVRISP_SWMIN)
        elif c == 0x93:
            self.breply(ord('S')) # serial programmer
        else:
            self.breply(0);
        return

    def set_parameters(self):
        # call this after reading paramter packet into buff[]
        param['devicecode'] = self.buff[0]
        param['revision']   = self.buff[1]
        param['progtype']   = self.buff[2]
        param['parmode']    = self.buff[3]
        param['polling']    = self.buff[4]
        param['selftimed']  = self.buff[5]
        param['lockbytes']  = self.buff[6]
        param['fusebytes']  = self.buff[7]
        param['flashpoll']  = self.buff[8]
        # ignore buff[9] (= buff[8])
        # following are 16 bits (big endian)
        param['eeprompoll'] = self.beget16(self.buff, 10)
        param['pagesize']   = self.beget16(self.buff, 12)
        param['eepromsize'] = self.beget16(self.buff, 14)

        # 32 bits flashsize (big endian)
        param['flashsize'] = self.buff[16] * 0x01000000 + self.buff[17] * 0x00010000 + self.buff[18] * 0x00000100 + self.buff[19]
        return

    def start_pmode(self):
        self.SPI.init(baudrate = self._spi_freq)
        # self.SPI.setHwCs(False);

        # try to sync the bus
        self.SPI.write(bytearray([0x00]))
        self.digitalWrite(self._reset_pin, self._resetLevel(False));
        time.sleep_us(50);
        self.digitalWrite(self._reset_pin, self._resetLevel(True));
        time.sleep_ms(30);

        self.spi_transaction(0xAC, 0x53, 0x00, 0x00)
        self.pmode = 1
        return

    def end_pmode(self):
        self.SPI.deinit();
        self.setReset(self._reset_state)
        self.pmode = 0
        return

    def universal(self):
        self.fill(4);
        ch = self.spi_transaction(self.buff[0], self.buff[1], self.buff[2], self.buff[3])
        self.breply(ch[0])
        return

    def flash(self, hilo, addr, data):
        self.spi_transaction(0x40 + 8 * hilo,
                        addr >> 8 & 0xFF,
                        addr & 0xFF,
                        data)
        return

    def commit(self, addr):
        self.spi_transaction(0x4C, (addr >> 8) & 0xFF, addr & 0xFF, 0)
        time.sleep_ms(AVRISP_PTIME);
        return

    # #define _addr_page(x) (here & 0xFFFFE0)
    def addr_page(self, addr):
        if param['pagesize'] == 32:
            return addr & 0xFFFFFFF0
        if param['pagesize'] == 64:
            return addr & 0xFFFFFFE0
        if param['pagesize'] == 128:
            return addr & 0xFFFFFFC0
        if param['pagesize'] == 256:
            return addr & 0xFFFFFF80
        AVRISP_DEBUG("unknown page size: " + str(param['pagesize']))
        return addr

    def write_flash(self, length):
        self.fill(length)

        if Sync_CRC_EOP == self.getch():
            self._client.send(Resp_STK_INSYNC)
            self._client.send(self.write_flash_pages(length))
        else:
            self.error += 1
            self._client.send(Resp_STK_NOSYNC)

        return

    def write_flash_pages(self, length):
        x = 0
        page = self.addr_page(self.here)
        while x < length:
            # XXX yield()
            if page != self.addr_page(self.here):
                self.commit(page)
                page = self.addr_page(self.here)
            
            self.flash(V_LOW, self.here, self.buff[x])
            x += 1
            self.flash(V_HIGH, self.here, self.buff[x])
            x += 1
            self.here += 1

        self.commit(page)
        return Resp_STK_OK

    def write_eeprom(self, length):
        # here is a word address, get the byte address
        start = self.here * 2;
        remaining = length;
        if length > param['eepromsize']:
            self.error += 1
            return Resp_STK_FAILED;
        
        while remaining > EECHUNK:
            self.write_eeprom_chunk(start, EECHUNK)
            start += EECHUNK;
            remaining -= EECHUNK;

        self.write_eeprom_chunk(start, remaining)
        return Resp_STK_OK

    def write_eeprom_chunk(self, start, length):
        # this writes byte-by-byte,
        # page writing may be faster (4 bytes at a time)
        self.fill(length)
        # prog_lamp(V_LOW);
        for x in range(length):
            addr = start + x;
            self.spi_transaction(0xC0, (addr >> 8) & 0xFF, addr & 0xFF, self.buff[x])
            time.sleep_ms(45)

        # prog_lamp(V_HIGH);
        return Resp_STK_OK;

    def program_page(self):
        result = Resp_STK_FAILED
        length = 256 * self.getch()
        length += self.getch();
        memtype = self.getch()
        # flash memory @here, (length) bytes

        if memtype == ord('F'):
            AVRISP_DEBUG("program_page: Flash Mode")
            self.write_flash(length)
            return

        if memtype == ord('E'):
            AVRISP_DEBUG("E")
            AVRISP_DEBUG("program_page: EEPROM Mode")
            result = self.write_eeprom(length)
            if Sync_CRC_EOP == self.getch():
                self._client.send(Resp_STK_INSYNC)
                self._client.send(result)
            else:
                self.error += 1
                self._client.send(Resp_STK_NOSYNC)
            return;
        
        AVRISP_DEBUG("program_page:Error !!!!!")
        self._client.send(Resp_STK_FAILED)
        return

    def flash_read(self, hilo, addr):
        return self.spi_transaction(0x20 + hilo * 8, (addr >> 8) & 0xFF, addr & 0xFF, 0)

    def flash_read_page(self, length):
        AVRISP_DEBUG("flash_read_page " + hex(length))
        data = bytearray(length + 1)
        for x_n in range(length/2):
            x = x_n*2
            data[x] = self.flash_read(V_LOW, self.here)[0]
            data[x + 1] = self.flash_read(V_HIGH, self.here)[0]
            self.here += 1
        
        data[length] = Resp_STK_OK[0]
        self._client.write(data)
        gc.collect()
        return

    def eeprom_read_page(self, length):
        # here again we have a word address
        data = bytearray(length + 1)
        start = self.here * 2
        for x in range(length):
            addr = start + x;
            ee = self.spi_transaction(0xA0, (addr >> 8) & 0xFF, addr & 0xFF, 0xFF)
            data[x] = ee[0]
        data[length] = Resp_STK_OK[0]
        self._client.write(data)
        return

    def read_page(self):
        length = 256 * self.getch()
        length += self.getch()
        memtype = self.getch()
        if not Sync_CRC_EOP == self.getch():
            self.error += 1
            self._client.send(Resp_STK_NOSYNC)
            return

        self._client.send(Resp_STK_INSYNC)
        if memtype == ord('F'):
            self.flash_read_page(length)
        if memtype == ord('E'):
            self.eeprom_read_page(length)
        return

    def read_signature(self):
        if Sync_CRC_EOP != self.getch():
            self.error += 1
            self._client.send(Resp_STK_NOSYNC)
            return

        self._client.send(Resp_STK_INSYNC)

        high = self.spi_transaction(0x30, 0x00, 0x00, 0x00)
        self._client.send(high)
        middle = self.spi_transaction(0x30, 0x00, 0x01, 0x00)
        self._client.send(middle)
        low = self.spi_transaction(0x30, 0x00, 0x02, 0x00)
        self._client.send(low)
        self._client.send(Resp_STK_OK)
        return

    # It seems ArduinoISP is based on the original STK500 (not v2)
    # but implements only a subset of the commands.
    def avrisp(self):
        # XXX uint8_t data, low, high;
        ch = self.getch()
        # Avoid set but not used warning.  Leaving them in as it helps document the code
        # XXX (void) data;
        # XXX (void) low;
        # XXX (void) high;
        AVRISP_DEBUG("CMD " + hex(ch))
        if ch == Cmnd_STK_GET_SYNC:
            AVRISP_DEBUG("Cmnd_STK_GET_SYNC CMD " + hex(ch))
            self.error = 0;
            self.empty_reply();

        elif ch == Cmnd_STK_GET_SIGN_ON:
            if self.getch() == Sync_CRC_EOP:
                self._client.send(Resp_STK_INSYNC)
                self._client.send("AVR ISP") # AVR061 says "AVR STK"?
                self._client.send(Resp_STK_OK)

        elif ch == Cmnd_STK_GET_PARAMETER:
            self.get_parameter(self.getch())

        elif ch == Cmnd_STK_SET_DEVICE:
            self.fill(20);
            self.set_parameters()
            self.empty_reply()

        elif ch == Cmnd_STK_SET_DEVICE_EXT: # ignored
            self.fill(5);
            self.empty_reply()

        elif ch == Cmnd_STK_ENTER_PROGMODE:
            self.start_pmode()
            self.empty_reply()

        elif ch == Cmnd_STK_LOAD_ADDRESS:
            self.here = self.getch();
            self.here += 256 * self.getch();
            AVRISP_DEBUG("here=" + hex(self.here))
            self.empty_reply()

        # XXX: not implemented!
        elif ch == Cmnd_STK_PROG_FLASH:
            low = self.getch()
            high = self.getch()
            self.empty_reply()

        # XXX: not implemented!
        elif ch == Cmnd_STK_PROG_DATA:
            data = self.getch()
            self.empty_reply()

        elif ch == Cmnd_STK_PROG_PAGE:
            self.program_page()

        elif ch == Cmnd_STK_READ_PAGE:
            self.read_page()

        elif ch == Cmnd_STK_UNIVERSAL:
            self.universal()

        elif ch == Cmnd_STK_LEAVE_PROGMODE:
            self.error = 0;
            self.end_pmode()
            self.empty_reply()
            time.sleep_ms(5);
            # if (self._client && self._client.connected())
            self._client.close();
            self.client_connected = False
            gc.collect()
            AVRISP_DEBUG("left progmode")

        elif ch == Cmnd_STK_READ_SIGN:
            self.read_signature()
            # expecting a command, not Sync_CRC_EOP
            # this is how we can get back in sync
        elif ch == Sync_CRC_EOP: # 0x20, space
            self.error += 1
            self._client.send(Resp_STK_NOSYNC)

        # anything else we will return STK_UNKNOWN
        else:
            AVRISP_DEBUG("?!?");
            self.error += 1
            try:
                if Sync_CRC_EOP == self.getch():
                    self._client.send(Resp_STK_UNKNOWN)
                else:
                    self._client.send(Resp_STK_NOSYNC)
            except:
                self.client_connected = False
        return

    def digitalWrite(self, pin, value):
        if value:
            self.pin_reset.value(255)
        else:
            self.pin_reset.value(0)
        return

    def _resetLevel(self, reset_state):
        return reset_state == self._reset_activehigh

    def beget16(self, addr, index):
        return (addr[index] * 256 + addr[index + 1])

    def __close__(self):
        self._client.close()
        self._client = None
        self._server.close()
        self._server = None
        self.buff = None
        time.sleep_ms(100);
        gc.collect()
        return

