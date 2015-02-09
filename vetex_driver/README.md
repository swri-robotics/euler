### CANOpenShell Utility

#### Run the following in a terminal to access the utility

```
cd /usr/local/lib/
CANOpenShell load#libcanfestival_can_socket.so,can0,250K,10,0
```

#### Read Value
```
rsdo#1,2300,15
```

#### Write Value
```
wsdo#1,2300,15,02,3f
```

#### Help with commands
```
help
```

#### Reading Current Max Speed Settings
- Enter one line at a time:
```
rsdo#1,2300,f
rsdo#1,2300,10
rsdo#1,2301,7
rsdo#1,2301,8
rsdo#1,2301,15
rsdo#1,2301,16
rsdo#1,2300,19
```

#### Writing Max Speed Settings
```
wsdo#1,2300,f,02,1e0
wsdo#1,2300,10,02,140
wsdo#1,2301,7,02,140
wsdo#1,2301,8,01,6c
wsdo#1,2301,15,02,140
wsdo#1,2301,16,02,6c
wsdo#1,2300,19,01,3c
```

### Settings Table
|  Read Command  | SubIndex(dec) | SubIndex(hex) | Default (dec) | Default (hex) |Desired (dec) | Desired (hex) |Size |
|:--------------:|:-------------:|:-------------:|:-------------:|:-------------:|:------------:|:-------------:|:---:|
| rsdo#1,2300,f  |      15       |       f       |      320      |      140      |     480      |       1e0     |  02 |
| rsdo#1,2300,10 |      16       |      10       |      190      |       be      |     320      |       140     |  02 | 
| rsdo#1,2301,7  |       7       |       7       |       90      |       5a      |     320      |       140     |  02 |
| rsdo#1,2301,8  |       8       |       8       |       40      |       28      |     108      |        6c     |  01 |
| rsdo#1,2301,15 |      21       |      15       |       90      |       5a      |      64      |       140     |  02 |
| rsdo#1,2301,16 |      22       |      16       |       40      |       28      |     320      |        6c     |  02 |
| rsdo#1,2300,19 |      25       |      19       |       75      |       4b      |      60      |        3c     |  01 |

### Saving Parameters
wsdo#1,1010,4,02,65766173 (Doesn't work??)
