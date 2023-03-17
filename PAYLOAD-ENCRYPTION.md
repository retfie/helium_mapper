# Optional payload encryption

If a symmetric key is configured from the Zephyr shell, and the feature is
enabled during build, then payload is sent encrypted with an AES128 key over
LoRaWan. Payload will not be readable in the Helium console. Only your
private Helium integration server would have the AES key, and thus be able
to decrypt the payload data.

Note that the LoRaWan metadata (e.g. which hotspot relayed a packet) would
not be protected by that AES128 key. The latter is only for the payload.

## Adding support to the firmware

Simply enable the PAYLOAD_ENCRYPTION config. Example:
```
west build --pristine  -b rak4631_nrf52840 -s app -- -DCONFIG_USB_DFU_REBOOT=y -DCONFIG_BT=n -DCONFIG_PAYLOAD_ENCRYPTION=y
```

## Enabling encryption

First create an AES key:
```
$ openssl rand -hex 16 | tee payload_aes_key.hex
```
Then save it to the device using the Zephyr shell, e.g.:
```
uart:~$ lorawan payload_key f4d1cbcae025b1ab6fa124f99e1f21d3
```

To erase the key, and thus fallback to sending payload in clear, simply
write all-zeros as a key:
```
uart:~$ lorawan payload_key 00000000000000000000000000000000
```

## Decrypting

Here is an example Python snippet to decrypt the payload on your Helium integration server:
```
import base64
import binascii
from Crypto.Cipher import AES

payload_enc = base64.b64decode(payload_base64_str)
with open('payload_aes_key.hex') as f:
    key = binascii.unhexlify(f.readline().strip())
iv = payload_enc[:16]
cipher = AES.new(key, AES.MODE_CBC, iv)
payload_clear = cipher.decrypt(payload_enc[16:])
```
