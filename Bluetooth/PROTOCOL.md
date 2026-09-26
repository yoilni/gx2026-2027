# Bluetooth control protocol

UART2 transport is configured as 115200 baud, 8 data bits, no parity, and one
stop bit.

Each command is exactly five bytes:

```text
A1 A2 CMD B1 B2
```

| CMD | Meaning |
| --- | --- |
| `11` | forward button pressed |
| `12` | forward button released |
| `21` | backward button pressed |
| `22` | backward button released |
| `31` | left button pressed |
| `32` | left button released |
| `41` | right button pressed |
| `42` | right button released |
| `55` | stop button pressed |

All values above are hexadecimal bytes. Commands are accepted only while the
debounced PE15 input is high. Changing control mode always stops both wheels
and discards bytes received under the previous authority. A release command
stops only its matching active motion; `55` always stops.

This first protocol revision has no checksum, sequence counter, heartbeat, or
acknowledgement. Consequently it cannot detect a lost release frame or a lost
Bluetooth link while the vehicle is moving. Add a periodic phone heartbeat and
command timeout before operating outside a controlled test area.
