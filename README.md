## Getting Started

Before getting started, make sure you have a proper Zephyr development
environment. You can follow the official
[Zephyr Getting Started Guide](https://docs.zephyrproject.org/latest/getting_started/index.html).

### Hardware
RAKWireless WisBlock Nordic nRF52840 BLE Core Module for LoRaWAN with LoRa SX1262 RAK4631 / RAK4631-R
https://docs.zephyrproject.org/latest/boards/arm/rak4631_nrf52840/doc/index.html

![RAK4631-NRF52840](doc/img/rak4631-front-parts.jpg)

### Initialization

The first step is to initialize the workspace folder where the
`helium_mapper` and needed Zephyr modules will be cloned. You can do
that by running:

```shell
# initialize workspace for the helium_mapper (main branch)
west init -m https://github.com/retfie/helium_mapper --mr main helium_mapper_project
# update Zephyr modules
cd helium_mapper_project/helium_mapper
west update
```

### Build & Run

The application can be built by running:

```shell
west build -b rak4631_nrf52840 -s app
```

Once you have built the application you can flash it by running:

```shell
west flash
```

### Serial terminal

```shell
screen /dev/ttyACM0 115200
```
Press ENTER, and you will get unix like shell prompt.\
Press TAB to see all available commands.

```shell
uart:~$
  adc                battery            clear              config
  date               device             devmem             flash
  help               history            hwinfo             i2c
  kernel             location           log                lora
  lorawan            nrf_clock_control  reboot             resize
  sensor             shell              status

uart:~$ battery
Battery: 4245 mV, 100 %

uart:~$ sensor get temp@4000c000
channel idx=12 die_temp =  24.500000

uart:~$ status
Device status:
  joined           false
  delayed active   false
  gps power on     false
  messages sent    0
  messages failed  0
  Accel events     0
  Total GPS ON     0 sec
  last msg sent    178 sec
  last acc event   178 sec
  Uptime           0000-00-00 00:02:58
```

```shell
uart:~$ config
Device config:
  RAK4631 Helium mapper: v3.2.0-rc3-2-g83d4ca7ac8df
  Dev EUI          0011223344556677
  APP EUI          0011223344556677
  APP key          00112233445566778899aabbccddeeff
  Auto join        false
  Data rate/DR+    3
  Confirmed msgs   true
  Send interval    3600 sec
  Min delay        30 sec
  Max GPS ON time  300 sec
```

```shell
uart:~$ lorawan --help
lorawan - lorawan commands
Subcommands:
  dev_eui          :Get/set dev_eui [0011223344556677]
  app_eui          :Get/set app_eui [0011223344556677]
  app_key          :get/set app_key [00112233445566778899aabbccddeeff]
  auto_join        :Auto join true/false
  confirmed_msg    :Confirmed messages true/false
  send_interval    :Send interval in seconds
  min_delay        :Min delay between 2 messages in ms
  max_gps_on_time  :Max time GPS is ON if no one using it in seconds
```

Show dev_eui
```shell
uart:~$ lorawan dev_eui
dev_eui 0011223344556677
```

Set dev_eui
```shell
uart:~$ lorawan dev_eui 0123456789abcdef
uart:~$ lorawan dev_eui
dev_eui 0123456789ABCDEF

```

After set all LoRaWAN parameters, just reboot device to take them into effect:
```shell
uart:~$ reboot
```


```shell
...

[00:00:00.253,387] <inf> max7q: GPS enable gpio configured
[00:00:00.253,387] <inf> max7q: UART for GPS ready
[00:00:00.299,194] <inf> lis2dh: lis3dh@18: int2 on gpio@50000000.09
[00:00:00.301,300] <inf> lis2dh: fs=2, odr=0x4 lp_en=0x8 scale=9576
[00:00:00.359,832] <inf> fs_nvs: 8 Sectors of 4096 bytes
[00:00:00.359,832] <inf> fs_nvs: alloc wra: 7, f78
[00:00:00.359,863] <inf> fs_nvs: data wra: 7, 5ac
[00:00:00.393,463] <inf> hellium_mapper_ble: Starting Bluetooth NUS shell transport example
[00:00:00.396,148] <inf> hellium_mapper_ble: Bluetooth ready. Advertising started.
[00:00:00.397,399] <inf> helium_mapper: lis3dh@18: 0, -0.153216, 3.677184, -8.580096 (m/s^2)
[00:00:00.398,284] <inf> helium_mapper: Sampling at 10 Hz
[00:00:00.398,803] <inf> lis2dh: int2_ths=0x3 range_g=2 ums2=523597
[00:00:00.399,230] <inf> lis2dh: int2_dur=0x4
[00:00:00.401,062] <inf> helium_mapper_gps: max7q device is ready.
[00:00:00.401,092] <inf> helium_mapper_gps: gps trigger handler set
[00:00:00.401,092] <inf> helium_mapper: Joining network over OTAA
[00:00:05.880,859] <inf> lorawan: Joined network! DevAddr: 260bc520
[00:00:05.887,512] <inf> helium_mapper: New Datarate: DR_3, Max Payload 115
[00:00:05.887,542] <inf> lorawan: Datarate changed: DR_3
[00:00:05.887,603] <inf> helium_mapper: Waiting for events...
[00:00:34.837,524] <inf> helium_mapper: ACC trigger handler
[00:00:34.837,615] <inf> helium_mapper: Event ACC
[00:00:34.838,714] <inf> helium_mapper: lis3dh@18: 1, -1.072512, 8.426880, -12.870144 (m/s^2)
[00:00:34.838,775] <inf> helium_mapper: Event NMEA_TRIG_ENABLE
[00:00:34.838,775] <inf> max7q: GPS power ON
[00:00:34.838,806] <inf> max7q: NMEA Trigger: 1
[00:00:34.838,806] <inf> helium_mapper: GPS off timer start for 300 sec
[00:00:34.838,836] <inf> helium_mapper: Waiting for events...
[00:00:34.838,867] <inf> helium_mapper: Waiting for events...
[00:00:35.532,409] <inf> helium_mapper: ACC trigger handler

...

[00:02:43.177,642] <inf> helium_mapper: GPS trigger handler
[00:02:43.177,673] <inf> max7q: NMEA Trigger: 0
[00:02:43.177,734] <inf> helium_mapper: Event SEND
[00:02:43.182,525] <inf> helium_mapper: Lora send -------------->
[00:02:48.571,044] <inf> helium_mapper: Port 0, Pending 0, RSSI -42dB, SNR 11dBm
[00:02:48.577,941] <inf> helium_mapper: Data sent!
[00:02:48.577,972] <inf> helium_mapper: Waiting for events...

```

### Bluetooth terminal
To view, set and control over Bluetooth, use any application that supports The Nordic UART service UUID,
for example "Serial Bluetooth Terminal" for Android.

![BT NUS Terminal](doc/img/bt_nus.png)

### RTT terminal
RTT console is supported and verified with J-link jtag adapter.
