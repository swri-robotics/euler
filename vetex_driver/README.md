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
rsdo#1,2300,7
rsdo#1,2300,8
rsdo#1,2300,15
rsdo#1,2300,16
```

#### Writing Max Speed Settings
```
wsdo#1,2300,f,02,1e0
wsdo#1,2300,10,02,140
wsdo#1,2300,7,02,140
wsdo#1,2300,8,01,6c
wsdo#1,2300,15,02,40
wsdo#1,2300,16,02,140
```
