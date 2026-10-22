# Installation for Arduino IDE

```bash

cp -r signalk-mini/ /cygdrive/c/Users/$USER_ID/Documents/Arduino/libraries/
cp -r signalk-mini/modules/*/src/ /cygdrive/c/Users/$USER_ID/Documents/Arduino/libraries/signalk-mini/

```

Sending "subscribe-all"

```
socat -d -d STDIO TCP:192.168.x.x:20223
{"subscribe":"all"}

```
