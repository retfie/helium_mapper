## Getting Started

Before getting started, make sure you have a proper Zephyr development
environment. You can follow the official
[Zephyr Getting Started Guide](https://docs.zephyrproject.org/latest/getting_started/index.html).

### Hardware
RAKWireless WisBlock Nordic nRF52840 BLE Core Module for LoRaWAN with LoRa SX1262 RAK4631 / RAK4631-R
https://docs.zephyrproject.org/latest/boards/arm/rak4631_nrf52840/doc/index.html

![RAK4631-NRF52840](doc/img/rak4631-front-parts.jpg)

BOM:
 * [RAK4631](https://store.rakwireless.com/products/rak4631-lpwan-node) Nordic nRF52840 BLE Core Module for LoRaWAN with LoRa SX1262.
 * [RAK19003](https://store.rakwireless.com/products/wisblock-base-board-rak19003) WisBlock Mini Base Board.
 * [RAK1904](https://store.rakwireless.com/products/rak1904-lis3dh-3-axis-acceleration-sensor) 3-Axis Acceleration Sensor STMicroelectronics LIS3DH.
 * [RAK1910](https://store.rakwireless.com/products/rak1910-max-7q-gnss-location-sensor) GNSS GPS Location Module u-Blox MAX-7Q.
 * [Battery Connector Cable](https://store.rakwireless.com/products/battery-connector-cable).
 * Li-Ion battery.

### Initialization

The first step is to initialize the workspace folder where the
`helium_mapper` and needed Zephyr modules will be cloned. You can do
that by running:


To avoid issues with system wide python versions, it's better to prepare Virtual Env
```shell
python3 -m venv venv
. venv/bin/activate
(.venv) pip3 install west
```

```shell
# initialize workspace for the helium_mapper (main branch)
(.venv) west init -m https://github.com/retfie/helium_mapper --mr main helium_mapper_project
# update Zephyr modules
(.venv) cd helium_mapper_project/helium_mapper
(.venv) west update
(.venv) west zephyr-export
(.venv) west packages pip --install
(.venv) . ../zephyr/zephyr-env.sh
```

### Build & Run

The application can be built by running:

```shell
. venv/bin/activate
. ../zephyr/zephyr-env.sh
(.venv) west build -p -b rak4631 --sysbuild -s app
```

If BT or/and shell login support is needed:
```
(.venv) west build -p -b rak4631 --sysbuild -s app -- -Dapp_SNIPPET="bt;shell-login"
```

Disable any CONFIG_xxx feature from app domain:
```
(.venv) west build -p -b rak4631 --sysbuild -s app -- -Dapp_SNIPPET="bt;shell-login" -Dapp_CONFIG_LOG_BACKEND_UART=n
```

or from mcuboot domain:
```
(.venv) west build -p -b rak4631 --sysbuild -s app -- -Dapp_SNIPPET="bt;shell-login" -Dmcuboot_CONFIG_LOG_BACKEND_UART=n
```

Add -DSB_CONFIG_xxx for both domains

Look into build/domains.yaml for list of doimains.

Once you have built the application you can flash it by running:

```shell
(.venv) west flash
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
  boot status      BOOT_COMPLETE
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
  RAK4631 Helium mapper (built: Dec 21 2024 16:46:59)
    Kernel ver:    v4.0.0-1996-gf17b82ed4334
    App ver:       app-v0.6-g5fb1ed072ec9
  Dev EUI          0011223344556677
  APP EUI          0011223344556677
  APP key          00112233445566778899aabbccddeeff
  Auto join        false
  Data rate/DR+    3
  Confirmed msgs   true
  Send interval    3600 sec
  Min delay        30 sec
  Max GPS ON time  300 sec
  Payload key      00000000000000000000000000000000
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
  data_rate        : Get/set data rate 0-15
  payload_key      : get/set payload_key [00000000000000000000000000000000]
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

### Bluetooth serial console from PC
```
pip install ble-serial
ble-scan
Started general BLE scan

F1:44:8D:xx:xx:xx (RSSI=-48): Hellium_mapper
....
```

connect to device
```
ble-serial -d F1:44:8D:xx:xx:xx
16:29:56.569 | INFO | linux_pty.py: Port endpoint created on /tmp/ttyBLE -> /dev/pts/66
...
```

Open another console and open /tmp/ttyBLE (at 115200 for example) with screen, or other terminal program.
```
screen /tmp/ttyBLE 115200
uart:~$ config
Device config:
  RAK4631 Helium mapper: zephyr-v3.2.0-1753-g179de6d0d08c, Mar 20 2023 11:50:51
  Dev EUI          0011223344556677
  APP EUI          0011223344556677
  APP key          00112233445566778899aabbccddeeff
  Auto join        true
  Data rate/DR+    3
  Confirmed msgs   false
  Max failed msgs  120
  Inactive window  10800 sec
  Send interval    3600 sec
  Min delay        15 sec
  Max GPS ON time  300 sec
uart:~$
uart:~$ status
Device status:
  joined           true
  delayed active   false
  gps power on     false
  messages sent    19
  messages failed  0
  msg failed total 0
  Accel events     4
  Join retry sess  0
  Total GPS ON     5756 sec
  last msg sent    2034 sec
  last msg sent OK 2034 sec
  last acc event   4754 sec
  Uptime           0000-00-00 18:38:57
uart:~$
```

### RTT terminal
RTT console is supported and verified with J-link jtag adapter.

### Bootloader and DFU support
Update Zephyr modules
```
cd helium_mapper_project
west update
```

With migrating to sysbuilds, bootloader (mcuboot) is build as secondary image alongside the app image (helium_mapper).
west flash command flashes both images to device with help of choosen runner (J-Link/OpenOCD), and after this all consequitive flashes could be done via DFU:


```
sudo dfu-util --alt 1 -d 2fe3:0100 --download build/zephyr/zephyr.signed.bin
dfu-util 0.9

Copyright 2005-2009 Weston Schmidt, Harald Welte and OpenMoko Inc.
Copyright 2010-2016 Tormod Volden and Stefan Schmidt
This program is Free Software and has ABSOLUTELY NO WARRANTY
Please report bugs to http://sourceforge.net/p/dfu-util/tickets/

dfu-util: Invalid DFU suffix signature
dfu-util: A valid DFU suffix will be required in a future dfu-util release!!!
Opening DFU capable USB device...
ID 2fe3:0100
Run-time device DFU version 0110
Claiming USB DFU Runtime Interface...
Determining device status: state = appIDLE, status = 0
Device really in Runtime Mode, send DFU detach request...
Device will detach and reattach...
Opening DFU USB Device...
Claiming USB DFU Interface...
Setting Alternate Setting #1 ...
Determining device status: state = dfuIDLE, status = 0
dfuIDLE, continuing
DFU mode device DFU version 0110
Device returned transfer size 128
Copying data from PC to DFU device
Download        [=========================] 100%       345272 bytes
Download done.
state(4) = dfuDNBUSY, status(0) = No error condition is present
Done!
```

Once new image boots, open serial console to device, and use mcuboot command to confirm new image if its OK, and then reboot device to verify that new image is loaded.
```
mcuboot confirm
reboot
```

Example output from device serial console:
```
uart:~$ mcuboot
swap type: revert
confirmed: 0

primary area (1):
  version: 0.0.0+0
  image size: 344424
  magic: good
  swap type: test
  copy done: set
  image ok: unset

secondary area (2):
  version: 0.0.0+0
  image size: 344424
  magic: unset
  swap type: none
  copy done: unset
  image ok: unset
[00:01:12.956,359] <inf> mcuboot_util: Swap type: revert

uart:~$ mcuboot confirm

uart:~$ mcuboot
swap type: none
confirmed: 1

primary area (1):
  version: 0.0.0+0
  image size: 344424
  magic: good
  swap type: test
  copy done: set
  image ok: set

secondary area (2):
  version: 0.0.0+0
  image size: 344424
  magic: unset
  swap type: none
  copy done: unset
  image ok: unset
[00:01:40.986,907] <inf> mcuboot_util: Swap type: none
```

Note! In case new image not confirmed, or fails to boot, it will automatically revert to last working image into slot-0

### Payload Encryption

The optional payload encryption support is documented [here](./PAYLOAD-ENCRYPTION.md).
