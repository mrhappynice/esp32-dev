AIPI Lite brand esp32 with screen tester code 

in idf.py menuconfig set size to 16mb and enable psram, also psram setting alloc to heap, set to oct vs quad

Will test screen and connecting to wifi, ssid is set to wifinet and password is password. change in main.c 

```sh
idf.py set-target esp32s3
idf.py menuconfig
idf.py build
idf.py flash monitor
```
