# Lệnh mavlink bắt tay giữa Raspberry và module test

## Connection
```
HEARTBEAT (#0)
 1. Sau khi khởi động, module test gửi HEARTBEAT định kỳ (1Hz) cho Pi, nếu không nhận được tín hiệu này, Pi sẽ chờ mãi mãi.
 2. Pi sẽ gửi định kỳ 1Hz cho module test để check connection 
```

## Start test
```
HEARTBEAT (#0)
 1. Sau khi quét barcode ok, Pi sẽ send HEARTBEAT cho module test. 
   Param 'custom_mode' sẽ được dùng cho lệnh này.
    - custom_mode = 1, start test
    - custom_mode = 2, stop test
    - custom_mode = 3, reset jig


   Param 'system_status' dùng báo kết quả
    - system_status = 1, push data to Firebase ok
    - system_status = 2, no Internet, cant push data 
    - system_status = 3, ready to scan barcode

   Param 'base_mode' dùng cho login
    - base_mode = 1, wait for login
    - base_mode = 2, đã login, bắt đầu scan barcode để test
   
 2. Khi nhận được lênh start test, module sẽ gửi HEARTBEAT kèm trạng thái cho Pi. Param 'system_status' sẽ được dùng cho lệnh này.
    - system_status = 1, standby
    - system_status = 2, running test
    - system_status = 3, test done - ok
    - system_status = 4, test done - failed
    - system_status = 5, jig reseting
    - system_status = 6, jig got result from Pi
```

## Pi read params
```
```
PARAM_REQUEST_READ (#20)
```
  1. Pi sẽ request param dùng lệnh PARAM_REQUEST_READ
  - param id
     + "GYROX_OFFSET" : gyrox offset of gimbal
     + "GYROY_OFFSET" : gyroy offset of gimbal
     + "GYROZ_OFFSET" : gyroz offset of gimbal
     + "ACCELX_OFFSET" : accelx offset of gimbal
     + "ACCELY_OFFSET" : accely offset of gimbal
     + "ACCELZ_OFFSET" : accelz offset of gimbal
     + "UDID" : UDID of gimbal
     + "UDID2" : UDID2 of gimbal
  - param value
     + "GYROX_OFFSET" : float
     + "GYROY_OFFSET" : float
     + "GYROZ_OFFSET" : float
     + "ACCELX_OFFSET" : float
     + "ACCELY_OFFSET" : float
     + "ACCELZ_OFFSET" : float
     + "UDID" : 
     + "UDID2" : 
  - param index
     + "GYROX_OFFSET" : 1
     + "GYROY_OFFSET" : 2
     + "GYROZ_OFFSET" : 3
     + "ACCELX_OFFSET" : 4
     + "ACCELY_OFFSET" : 5
     + "ACCELZ_OFFSET" : 6
     + "UDID" : 7
     + "UDID2" : 8
```    
PARAM_VALUE (#22)
```
  2. Module test sẽ gửi giá trị cho Pi bằng lênh PARAM_VALUE
  - param id
     + "GYROX_OFFSET" : gyrox offset of gimbal
     + "GYROY_OFFSET" : gyroy offset of gimbal
     + "GYROZ_OFFSET" : gyroz offset of gimbal
     + "ACCELX_OFFSET" : accelx offset of gimbal
     + "ACCELY_OFFSET" : accely offset of gimbal
     + "ACCELZ_OFFSET" : accelz offset of gimbal
     + "UDID" : UDID of gimbal
     + "UDID2" : UDID2 of gimbal
  - param value
     + "GYROX_OFFSET" : float
     + "GYROY_OFFSET" : float
     + "GYROZ_OFFSET" : float
     + "ACCELX_OFFSET" : float
     + "ACCELY_OFFSET" : float
     + "ACCELZ_OFFSET" : float
     + "UDID" : 
     + "UDID2" : 
  - param index
     + "GYROX_OFFSET" : 1
     + "GYROY_OFFSET" : 2
     + "GYROZ_OFFSET" : 3
     + "ACCELX_OFFSET" : 4
     + "ACCELY_OFFSET" : 5
     + "ACCELZ_OFFSET" : 6
     + "UDID" : 7
     + "UDID2" : 8
     
```
## Các param cần lưu trên cloud
```
  - GYRO  : x, y, z   ---> đọc từ module test
  - ACCEL : x, y, z   ---> đọc từ module test
  - UDID  :           ---> đọc từ module test
  - UDID2 :           ---> đọc từ module test
  - SerialNumer       ---> đọc từ tay quét barcode


