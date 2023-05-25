# cyberdog_grpc协议

## 摘要

### 名词约定

- 下行指令/下行：指APP发送到NV板的指令或方向；
- 上行指令/上行：指NV板发送到APP的指令或方向；

### 方案设计

[cyberdog_grpc设计文档](./cyberdog_grpc_cn.md) 


## 运动控制

### 伺服控制（摇杆功能）

#### 下行指令

ros接口： topic，类型：protocol/msg/MotionServoCmd，话题名："motion_servo_cmd"

```Nginx
nameCode = 1002；

params
{
    int32 motion_id;     # 需查阅上述接口文档
    int32 cmd_type;      # 指令类型约束 0或1: Data, 2: End；（最后一帧发2）
    int32 cmd_source;    # 指令来源，app填0
    float32 vel_des[3];  # x方向， y方向， 旋转, "vel_des": [x, y, z]   
    float32 pos_des[3];  # [1]值有效, "pos_des": [0, value, 0] 
    float32 rpy_des[3];  # [2]值有效, "rpy_des": [0, 0, value]
    float32 step_height[2];  # 行走时有效， 推荐值[0.05, 0.05]
    int32 value;         # 0 - 内八步态， 2 - 垂直步态， 4 - 运控开始标定
}
```

#### 上行数据

ros接口：topic， "motion_servo_response"

```Nginx
nameCode = 1003

params
{
    int32  motion_id    # 下发的motion_id原路返回
    int32  cmd_id       # 如果下发有该字段，则返回该字段
    bool   result       # true： 正常执行； false： 执行失败，继续下发也不会执行                
    int32  code                
}
```



### 结果指令控制

#### 下行指令

ros接口：service，"motion_result_cmd"

```Nginx
nameCode = 1004

params
{
    int32 motion_id;
    int32 cmd_source;    # 指令来源，app填0
    float32 vel_des[3];  # x方向， y方向， 旋转, "vel_des": [x, y, z]
    float32 pos_des[3];  # [1]值有效, "pos_des": [0, value, 0]
    float32 rpy_des[3];  # [2]值有效, "rpy_des": [0, 0, value]
    float32 ctrl_point[3];  # 当前暂不开放。pose ctrl point  m
    float32 foot_pose[3];  # 当前暂不开放。front/back foot pose x,y,z  m
    float32 step_height[3];  # 抬腿高度，，最大值运控组不确定，当前可按0.06m设定
    int32 contact;
    int32 duration       # 执行时间
    int32 value
}  
```

#### 返回结果

```Nginx
nameCode = 1004

data
{
    int32  motion_id  # 下发的motion_id原路返回
    bool   result     # true： 执行成功； false： 执行失败；
    int32  code       # 运动模块的结果编码， 用于调试及快速定位问题
}
```

## OTA

### 状态上报

#### 下行指令

```Nginx
nameCode = 5001

params
{}
```



#### 上行指令

ros接口：topic， "ota_status" ？？？

```Nginx
nameCode = 5001

params
{
    string status    # idle: 空闲状态
                     # update： 升级中状态
                     # download： 下载中状态
    int32  code      # 0正常，其它值为错误码
}
```


### 开始下载 

#### 下行指令

ros接口：service，"ota_service_start_download"

```C%23
nameCode = 5003

params
{
    {
            "modules": [{
                "product": "l91",
                "device": "l91.NX",
                "module": "l91.NX.OTA",
                "current": {
                    "version": null,
                    "versionSerial": null,
                    "packages": null,
                    "logInfo": null,
                    "historyLogInfo": null,
                    "forceUpdateFlag": false
                },
                "latest": {
                    "version": "1.0.0.4",
                    "versionSerial": 1655173627000,
                    "packages": [{
                        "version": "1.0.0.4",
                        "type": "FULL_PACKAGE",
                        "md5": "b7e2a750b0",
                        "fileSize": 0,
                        "fileName": "L91%2FNX%2Fdaily-release%2F2022-06-14%2F122441%2Fcarpo_galactic_V1.0.0.1.20220614.102737_release_b7e2a750b0.tgz",
                        "description": null,
                        "downloadOption": {
                            "mirrors": ["https://cnbj2m.fds.api.xiaomi.com/version-release/L91%2FNX%2Fdaily-release%2F2022-06-14%2F122441%2Fcarpo_galactic_V1.0.0.1.20220614.102737_release_b7e2a750b0.tgz?Expires=1970540922760&GalaxyAccessKeyId=AKKF3F5Z2NCBBWT7IL&Signature=vPoyrIZTvJbpsRtS4PKFfrBJXlI="]
                        }
                    }],
                    "logInfo": null,
                    "historyLogInfo": [],
                    "forceUpdateFlag": false
                }
            }]
        }
}
```



#### 上行指令

```Nginx
nameCode = 5003

params
{
    bool response    # true : OTA执行开始下载命令成功
                    # false: OTA执行开始下载命令失败
    int32  code      # 0正常，其它值为错误码
}
```



### 开始升级

#### 下行指令

ros接口：service，"ota_service_start_upgrade"

```Nginx
nameCode = 5004

params
{    
    "md5":xxxxxx
}
```



#### 上行指令

```Nginx
nameCode = 5004

params
{
    bool reponse    # true : OTA执行开始升级命令成功
                    # false: OTA执行开始升级命令失败
    int32  code     # 0正常，其它值为错误码
}
```



### 进度上报

#### 下行指令

ros接口：service，"ota_service_process"

```Nginx
nameCode = 5005

params
{
}
```

#### 上行指令

```Nginx
nameCode = 5005

params
{
    int32  download_progress # 进度 下载失败: -1,  剩余空间小于2G: -2
    int32  upgrade_progress # 升级 升级 失败: -1,  剩余空间小于2G: -2
    int32  code             # 0正常，其它值为错误码
}
```

### 预计升级时间查询

#### 下行指令

ros接口：topic，"ota_topic_estimate_upgrade_time"

```Nginx
nameCode = 5006

params
{
}
```



#### 上行指令

```Nginx
nameCode = 5006

params
{
    int32 time     # 预估升级总时间 -1： 计算失败  -2：当前状态无法获取
}
```



### NX重启

#### 上行指令(主动上报)

```Nginx
nameCode = 5007

params
{
}
```

### 升级结果

#### 上行指令(主动上报)

```Nginx
nameCode = 5008

params
{
    bool success     # true : 成功； false : 失败
    int32 code       # 错误码，0-无错误
}
```

## AUDIO

### 鉴权请求

#### 下行指令

ros接口：service，"get_authenticate_didsn"

```Bash
nameCode = 3001

params
{
}
```

#### 上行指令

```Bash
nameCode = 3001

params
{
    string did
    string sn
}
```

### 鉴权回复

#### 下行指令

ros接口：service，"set_authenticate_token"

```Bash
nameCode = 3002

params
{
    uint64 uid
    string token_access
    string token_fresh
    string token_expires_in
}
```

#### 上行指令

```Bash
nameCode = 3002
params
{
    bool result
}
```

### 声纹训练开始

#### 下行指令

```Bash
nameCode = 3011

params
{
    string nick_name
    string voiceprint_id
}
```

#### 上行指令

```Bash
nameCode = 3011
params
{
}
```

### 声纹训练取消

#### 下行指令

```Bash
nameCode = 3012

params
{
}
```

#### 上行指令

```Bash
nameCode = 3012

params
{
}
```

### 声纹训练结果通知

#### 上行指令

```Bash
nameCode = 3013

params
{
    int32 code # 0正常，非0异常
    string voiceprint_id # 声纹id
}
```

### 声纹信息查询

#### 上行指令

```Bash
nameCode = 3014

params
{
}
```

#### 下行指令

```Bash
nameCode = 3014

params
{
    string voiceprints_data
}
```

## 机器人状态

### 心跳

```ProtoBuf
Ticks
{
    string ip = 1;
    fixed32 wifi_strength = 2;
    fixed32 battery_power = 3;
    bool internet = 4;
    string sn = 5;
    MotionStatus motion_status = 6;
    TaskStatus task_status = 7;
    SelfCheckStatus self_check_status = 8;
    StateSwitchStatus state_switch_status = 9;
    ChargingStatus charging_status = 10;
    bool audio = 11;
}
```

### 信息查询

##### 下行指令

```C%2B%2B
nameCode = 1001

params
{
    bool is_version               # 是否返回当前版本信息， true: 是； false: 否；
    bool is_sn                    # 是否返回sn，true: 是； false: 否；
    bool is_nick_name             # 是否返回昵称，true: 是； false: 否；
    bool is_volume                # 是否返回音量，true: 是； false: 否；
    bool is_mic_state             # 是否返回MIC状态，true: 是； false: 否；
    bool is_voice_control         # 是否返回语音控制状态，true：是； false：否；
    bool is_wifi                  # 是否返回wifi信息
    bool is_bat_info              # 是否返回电池信息，true: 是； false: 否；
    bool is_motor_temper          # 是否返回电机温度
    bool is_audio_state           # 是否返回音箱板激活状态
    bool is_device_model          # 是否返回设备型号
    bool is_stand                 # 是否站立
    bool is_lowpower_control      # 是否返回低功耗状态
    string uid                    # 当前用户uid
}

# 在配置is_audio_state获取音箱板激活状态时，传uid数据
```

##### 上行指令

```C%2B%2B
nameCode = 1001

params    # json字符串。
{
    string  version # 当前版本信息。包含所有板子软件的当前版本信息还有服务端最新版本信息
    string  sn      # 机器sn
    string  nick_name     # 昵称
    int     volume        # 音量
    bool    mic_state     # 当前麦客状态。true: enable； false: disable；
    bool    voice_control # 语音控制状态。true: enable； false: disable；
    bool    wifi          # WiFi信息
    string  bat_info      # 电池相关信息
    string  motor_temper  # 电机温度
    bool    audio_state   # 当前音箱板是否已激活。true:已激活；false:未激活
    string  device_model  # 设备型号，字符串
    bool    stand         # 是否站立
    bool    lowpower_control # 低功耗开关状态。true: enable； false: disable；
}
例：
{
    # 当前版本具体内容如下：
    "version": {
          "new_version":"1.0.0.0.2020202",
          "nx_debs": {
            "sw-version" : "1.0.6"  # NX的版本
          },
          "nx_mcu" : {
            "MCU_HEAD_MINILED" : "0.0.0.1_20220712",   #mini-led的版本
            "MCU_HEAD_TOF" : "0.0.0.1_20220712",       #头部的tof版本
            "MCU_IMU" : "0.0.0.1_20220712",            #imu版本
            "MCU_POWER" : "0.0.0.1_20220712",          #电源版本
            "MCU_REAR_TOF" : "0.0.0.1_20220712"        #尾部tof版本
          },
          "r329" : {
            "l91_audio" : "1.0.0.1.20220712.112619"   #语音板版本
          },
          "mr813" : {
            "l91_loco" : "1.0.0.1.20220712.022049"   #运控板版本
          },
          "mr813_mcu" : {
            "MCU_SPIE0" : "0.0.0.1_20220712",    #
            "MCU_SPIE1" : "0.0.0.1_20220712"    #
          },
          "motors" : {
            "MCU_Motor" : "0.2.1.2_20220606"     #电机版本
          }
    }，    
    "sn": "zxw88****166",
    "nick_name": {
          "default_name" : "铁蛋",
          "current_name" : "旺财" 
    },
    "volume": 50，
    "mic_state": true，
    "voice_control": true，
    "wifi": {
        "name": "***",
        "ip": "***",
        "mac": "***",
        "strength": 50
    },
    "bat_info": {
        "capacity": 97，      # 电池容量
        "power": 97，         # 电池电量
        "voltage": 220,       # 电池电压
        "temperature": 100    # 电池温度
        "is_charging": true   # 充电状态，true: 在充电； false: 未充电；
        "discharge_time": 120 # 放电时长，单位分钟。
    }，
    "motor_temper": {
        "hip": [97, 97, 97, 97],     # [髋部左前, 髋部右前, 髋部左后, 髋部右后]
        "thigh": [97, 97, 97, 97],   # [大腿左前, 大腿右前, 大腿左后, 大腿右后]
        "crus": [97, 97, 97, 97]     # [小腿左前, 小腿右前, 小腿左后, 小腿右后]           
    },
    "audio_state": false，
    "device_model": "MS2241CN"，
    "stand": true，
    "lowpower_control": true               
}
```

### 昵称开关

##### 下行指令

```C%2B%2B
nameCode = 1010

params
{
    bool enable         # 开：true，关：false
}
```

##### 上行指令

```C%2B%2B
nameCode = 1010

params
{
    bool success        # 成功：true，失败：false
}
```

### 设置昵称

##### 下行指令

```C%2B%2B
nameCode = 1011

params
{
    string nick_name    # 昵称
    string wake_name    # 唤醒词。昵称2个、3个字，唤醒词=昵称*2；昵称3个字以上，唤醒词=昵称。
}
```

##### 上行指令

```C%2B%2B
nameCode = 1011

params
{
    bool success        # 成功：true，失败：false
}
```

### 设置音量

##### 下行指令

```C%2B%2B
nameCode = 1012

params
{
    int volume    # 音量
}
```

##### 上行指令

```C%2B%2B
nameCode = 1012

params
{
    bool success        # 成功：true，失败：false
}
```

### 设置麦克风状态

##### 下行指令

```C%2B%2B
nameCode = 1013

params
{
    bool enable    # true: enable； false: disable；
}
```

##### 上行指令

```C%2B%2B
nameCode = 1013

params
{
    bool success        # 成功：true，失败：false
}
```

### 设置语音控制开关

##### 下行指令

```C%2B%2B
nameCode = 1014

params
{
    bool enable    # true: enable； false: disable；
}
```

##### 返回结果

```C%2B%2B
nameCode = 1014

data
{
    bool success        # 成功：true，失败：false
}
```

### 设置眼灯

#### 下行指令

```C%2B%2B
nameCode = 1015

params
{
    int  eyelight   # 亮度
}
```

#### 返回结果

```C%2B%2B
nameCode = 1015

data
{
    bool success        # 成功：true，失败：false
}
```



### 成员添加

##### 下行指令

```C%2B%2B
nameCode = 1020

params
{
    string member
}
```

##### 上行指令

```C%2B%2B
nameCode = 1020

params
{
    bool success        # 成功：true，失败：false
}
```

### 成员查询

##### 下行指令

```C%2B%2B
nameCode = 1021
params
{
    string account    # 空: 返回所有成员信息； 具体某一成员: 此成员信息；
}
```

##### 上行指令

```Bash
nameCode = 1021

params
{
    string accounts
}

{  
    "code": 0, 
    
    "accounts": [
        {
            "name": "xiao_ming",
            "face_state": 0,  # 0-未录入，1-已录入，2-正在录入中
            "voice_state" 0   # 0-未录入，1-已录入，2-正在录入中
        },
        {
            "name": "da_qiang",
            "face_state": 0,
            "voice_state" 0
        }      
    ]
}

# code = 0 查询成功; code = 121  用户查询失败，当前用户不存在，请删除后重新添加用户
# "accounts"：查询到的用户信息
```

### 成员删除

##### 下行指令

```C%2B%2B
nameCode = 1022

params
{
    string member
}
```

##### 上行指令

```C%2B%2B
nameCode = 1022

params
{
    bool success        # 成功：true，失败：false
}
```

### 成员同步

nameCode = 1023

### 成员修改昵称

##### 下行指令

```C%2B%2B
nameCode = 1024

params
{
    string pre_name
    string new_name
}
```

##### 上行指令

```C%2B%2B
nameCode = 1024

params
{
    bool success        # 成功：true，失败：false
}
```

### 停止声音播放

ros接口：service，类型：std_srvs/srv/Empty，服务名："stop_play"

#### 下行指令

```Nginx
nameCode = 1025

params
{
}
```

#### 返回结果

```Nginx
nameCode = 1025

data
{
}
```

## 图传

### 启动图传

webrtc的信令握手

grpc服务：GrpcApp.sendMsg

ros接口：topic，类型：std_msgs/msg/String，上行名称："img_trans_signal_out"，下行名称："img_trans_signal_in"

#### 下行指令

sdp:

```Nginx
nameCode = 4001

params
{
    string offer_sdp或者answer_sdp #其中包含type和sdp字段
    string uid #app端的一个标识名称，如果只连接一个app，可以不要此项
    int32 height #手机分辨率高，不填默认为1280
    int32 width #手机分辨率宽，不填默认为720
    string alignment #对齐方式，top顶对齐，middle水平中线对齐，bottom底对齐，不填默认为水平中线对齐
}
```

##### offer_sdp和answer_sdp

```Nginx
string type
string sdp
```

ice candidate:

```Nginx
nameCode = 4001

params
{
    string c_sdp #其中包含sdpMid、sdpMLineIndex和candidate字段
    string uid #app端的一个标识名称，如果只连接一个app，可以不要此项
}
```

##### c_sdp

```Nginx
string sdpMid
int32 sdpMLineIndex
string candidate
```

#### 上行指令

与上行指令相同，如果app端没设置uid项，则上行指令的uid为“default_uid”

### 停止图传

grpc服务：GrpcApp.sendMsg

ros接口：topic，类型：std_msgs/msg/String，上行名称："img_trans_signal_out"，下行名称：“img_trans_signal_in”

#### 下行指令

```Nginx
nameCode = 4001

params
{
    string uid #app端的一个标识名称，如果只连接一个app，可以不要此项
}
```

#### 返回结果

```Nginx
nameCode = 4001

data
{
    bool is_closed #是否已断开PeerConnection
    string uid #app端的一个标识名称，如果只连接一个app，可以不要此项
}
```

## 拍照

grpc服务：GrpcApp.getFile

ros接口：service，类型protocol/srv/CameraService，名称："camera_service"

#### 下行指令

```Nginx
nameCode = 4002

params
{
    uint8 command; #1为拍照
}
```

#### 返回结果

```Nginx
fixed32 error_code #错误码，0为正常，其它值为错误
string file_name #文件名
fixed32 file_size #文件大小（字节）
bytes buffer #文件数据块（最大为4MB）
```

## 录像

ros接口：service，类型protocol/srv/CameraService，名称："camera_service"

### 启动录像

grpc服务：GrpcApp.sendMsg

#### 下行指令

```Nginx
nameCode = 4002

params
{
    uint8 command; #2为开始录像
}
```

#### 返回结果

```Nginx
nameCode = 4002

data
{
    uint8 result #启动录像结果，0为成功，其它值为失败
}
```

### 停止录像

grpc服务：GrpcApp.getFile

#### 下行指令

```Nginx
nameCode = 4002

params
{
    uint8 command; #3为停止录像
}
```

#### 返回结果

```Nginx
fixed32 error_code #错误码，0为正常，其它值为错误
string file_name #文件名
fixed32 file_size #文件大小（字节）
bytes buffer #文件数据块（最大为4MB）
```

### 未完成传送的录像和拍照文件

如果在文件传输过程中grpc中断连接，则在重新连接后nx将上发此请求，app需要按照此请求重新获取文件

grpc服务：GrpcApp.sendMsg

#### 上行指令

```Nginx
nameCode = 4003

params
{
    string file_name[]  # 需要重新上传的文件列表
}
```

### 获取已保存的录像和拍照文件

APP在连接成功如果收到4003消息，说明上次连接有未传输完的文件或者录像过程中app断连自动保存的文件，需要重新传输，此时每一个文件都调用此请求

grpc服务：GrpcApp.getFile

#### 下行指令

```Nginx
nameCode = 4004

params
{
    string file_name  # 需要重新上传的文件
}
```

#### 返回结果

```Nginx
fixed32 error_code #错误码，0为正常，其它值为错误
string file_name #文件名
fixed32 file_size #文件大小（字节）
bytes buffer #文件数据块（最大为4MB）
```

### 空间不足警告

在nx板的磁盘空间低于某一预设值（暂定500MB）时，nx会上发此提示，app在接到此提示时应提示用户或立即停止录像

grpc服务：GrpcApp.sendMsg

#### 上行指令

```Nginx
nameCode = 4005

params
{
    int remain_size #剩余空间字节
}
```

## AI

### 人脸录入请求

##### 下行指令

```C%23
nameCode = 7001 #FACE_ENTRY_REQUEST

params
{
    int32 command
    string username //当前名字
    string oriname  // 老名字
    bool ishost //是不是主人
}
/* int32 command
int32 ADD_FACE = 0
int32 CANCLE_ADD_FACE = 1
int32 CONFIRM_LAST_FACE = 2
int32 UPDATE_FACE_ID = 3
int32 DELETE_FACE = 4
int32 GET_ALL_FACES = 5
*/
```

##### 上行指令

```C%2B%2B
nameCode = 7001
params
{
    int32 result
    string allfaces // 暂留
}
/*
int32 RESULT_SUCCESS = 0 // 除了add，其余的就是真实的返回结果
int32 RESULT_INVALID_ARGS = 5910    //参数不合法
int32 RESULT_UNSUPPORTED = 5908    //不支持参数
int32 RESULT_TIMEOUT = 5907
int32 RESULT_BUSY = 5911   //不支持同时进行人脸录入和人脸识别功能，请关闭图形化编程中人脸识别功能
int32 RESULT_INVALID_STATE = 5903
int32 RESULT_INNER_ERROR = 5904
int32 RESULT_UNDEFINED_ERROR = 5901
*/
```

### 人脸录入结果

#### 上行指令

```C%2B%2B
nameCode = 7003  #FACE_ENTRY_RESULT_PUBLISH 
params
{
    int32 result
    string username //添加人脸的名字
}
/*
int32 RESULT_SUCCESS =0
int32 RESULT_TIMEOUT =5907
int32 RESULT_FACE_ALREADY_EXIST = 5921
*/
```

### 人脸识别请求

#### 下行指令

```Go
nameCode = 7002 #FACE_RECORDING_REQUEST  

params
{
    int32 command
    string username //待识别人姓名
    int32 id
    int32 timeout #有效时间1～300，default = 60 
}
/* int32 command
int32 COMMAND_RECOGNITION_ALL = 0
int32 COMMAND_RECOGNITION_SINGLE = 1
*/
```

#### 上行指令

```C%2B%2B
nameCode = 7002
params
{
    int32 result
}
/*
int32 ENABLE_SUCCESS = 0
int32 ENABLE_FAIL = 5901
*/
```

### 人脸识别结果

#### 上行指令

```Go
nameCode = 7004  #FACE_RECOGNITION_RESULT_PUBLISH
params
{
    string username
    int32 result
    int32 id
    float32 age
    float32 emotion
}
/*
int32 RESULT_SUCCESS =0
int32 RESULT_TIMEOUT =5907   
*/
```

## SLAM相关

无法复制加载中的内容

### 获取地图列表

![](./image/grpc_protocol/grpc_slam.svg)

### 地图数据上报

ros接口：topic，类型：nav_msgs/msg/OccupancyGrid，名称："map"

#### 上行指令

```Nginx
nameCode = 6001

params
{
    float32 resolution #地图分辨率
    uint32 width #地图宽度
    uint32 height #地图高度
    float64 px #地图原点x
    float64 py #地图原点y
    float64 pz #地图原点z
    float64 qx #地图原点四元数x
    float64 qy #地图原点四元数y
    float64 qz #地图原点四元数z
    float64 qw #地图原点四元数w
    int8 data[] #地图数据
}
```

### 设置标签

ros接口：service，类型：protocol/srv/SetMapLabel，名称："set_label"

#### 下行指令

```Nginx
nameCode = 6002

params
{
  string mapName #地图名称
  uint64 map_id  # 地图唯一标识码
  bool is_outdoor #是否是室外建图
  label locationLabelInfo[] #标签列表
}
```

##### label结构

```Nginx
string labelName #标签名称
float64 physicX #X坐标
float64 physicY #Y坐标
```

#### 返回结果

```Nginx
NameCode = 6002

data
{
  uint8 success  #0为成功，1为失败
}
```

### 删除地图

ros接口：service，类型：protocol/srv/SetMapLabel，名称："set_label"

#### 下行指令

```Nginx
nameCode = 6002

params
{
  string mapName #地图名称
  uint64 map_id
  bool only_delete #此值为真时删除目标地图
}
```

#### 返回结果

```Nginx
NameCode = 6002

data
{
  uint8 success  #0为成功，1为失败
}
```

### 删除标签

ros接口：service，类型：protocol/srv/SetMapLabel，名称："set_label"

#### 下行指令

```Nginx
nameCode = 6002

params
{
  string mapName #地图名称
  uint64 map_id
  label locationLabelInfo[] #要删除的标签列表，如果只删一个就填一个，如果不填则认为是删除地图和所属标签
  bool only_delete #此值为真时删除目标标签
}
```

#### 返回结果

```Nginx
NameCode = 6002

data
{
  uint8 success  #0为成功，1为失败
}
```

### 获取标签和地图数据

ros接口：service，类型：protocol/srv/GetMapLabel，名称："get_label"

#### 下行指令

```Nginx
nameCode = 6003

params
{
  string mapName #地图名称
  uint64 map_id
}
```

#### 返回结果

```Nginx
nameCode = 6003

data
{
    string mapName #地图名称
    uint8 success  #0为成功，1为失败
    bool is_outdoor #是否是室外建图
    label locationLabelInfo[] #标签列表，标签结构同上
    Map map
}
```

##### Map结构

```Nginx
float32 resolution #地图分辨率
uint32 width #地图宽度
uint32 height #地图高度
float64 px #地图原点x
float64 py #地图原点y
float64 pz #地图原点z
float64 qx #地图原点四元数x
float64 qy #地图原点四元数y
float64 qz #地图原点四元数z
float64 qw #地图原点四元数w
int8 data[] #地图数据
```

label结构同上

### 建图任务

开始建图，但不会立即返回结果，会有中间反馈

ros接口：action，类型：protocol/action/Navigation，名称："start_algo_task"

所有6004开启的任务都可由6009进行手动停止

#### 下行指令

```Nginx
NameCode = 6004

params
{
    uint8 type   # 5 开始建图
    bool outdoor  #启动建图时确定是否为室外建图，true为室外建图，false为室内建图
}
```

#### 请求被接受

```Nginx
NameCode = 6004

data
{
  uint8 accepted #请求被接受时为1且任务继续运行；请求未被接受时为2且任务终止
}
```

#### 中间反馈

这三个反馈码适用于所有6004任务：1000正在激活依赖节点，1001激活依赖节点成功，1002激活依赖节点失败

```Nginx
NameCode = 6004

data
{
  int32 feedback_code #反馈码，6启动成功，7启动失败，8重定位成功，9重定位失败
  string feedback_msg # 额外消息，目前冗余
}
```

#### 返回结果

```Nginx
NameCode = 6004

data
{
    uint8 result #建图结果，0成功，非0失败
}
```

### 机器狗位姿上报

ros接口：topic，类型：geometry_msgs/msg/PoseStamped，名称："dog_pose"

#### 上行指令

```Nginx
nameCode = 6005

params
{
  float64 px #位置x坐标
  float64 py #位置y坐标
  float64 pz #位置z坐标
  float64 qx #位置四元数x
  float64 qy #位置四元数y
  float64 qz #位置四元数z
  float64 qw #位置四元数w
}
```

## 导航

![](./image/grpc_protocol/grpc_nav.svg)

### 重定位任务

所有6004开启的任务都可由6009进行手动停止

此请求在使用AB点导航之前务必调用（即进入导航页面时）。调用此请求后不会立即返回结果，机器狗会开启重定位功能，重定位过程中可能会上发反馈信息，重定位结束后会返回请求的结果，如果重定位成功则定位功能会开启并一直持续至退出导航页面。

ros接口：action，类型：protocol/action/Navigation，名称："start_algo_task"

#### 下行指令

```Nginx
NameCode = 6004

params
{
  uint8 type #7是启动重定位任务
  bool outdoor  #是否是室外
}
```

#### 请求被接受

```Nginx
NameCode = 6004

data
{
  uint8 accepted #请求被接受时为1且任务继续运行；请求未被接受时为2且任务终止
}
```

#### 中间反馈

这三个反馈码适用于所有6004任务：1000正在激活依赖节点，1001激活依赖节点成功，1002激活依赖节点失败

```Nginx
NameCode = 6004

data
{
  int32 feedback_code #反馈码，0成功，100失败继续尝试，遥控狗向前走一段距离，200失败，提示未在地图内或者地图错误，8传感器开启成功，9传感器开启失败
  string feedback_msg # 额外消息，目前冗余
}
```

地图检查服务feedback_code ：

- 正在检查地图：3100
- 地图检查成功： 3101
- 地图检查不可用，请重新建图： 3102
- 地图后台构建中，请稍后： 3110
- 地图检查服务出现异常，请重启机器狗： 3111

启动视觉SLAM相关服务feedback_code ：

- 正在启动重定位依赖服务：3103
- 启动重定位依赖服务成功： 3104
- 启动重定位依赖服务失败，请重新尝试导航功能： 3105

启动依赖节点feedback_code：

- 正在启动传感器依赖节点： 1000
- 启动传感器依赖节点成功： 1001
- 启动传感器依赖节点失败： 1002

重定位feedback_code：

- 重定位功能超时，请重试导航功能： 3106
- 重定位功能失败继续尝试，遥控狗向前走一段距离： 3107
- 重定位功能定位成功，点击地图或地图中的标签进行导航：3108
- 重定位功能定位失败，请重试导航功能：3109

#### 返回结果

```Nginx
NameCode = 6004

data
{
  uint8 result #启动导航结果，即重定位结果，0成功，非0失败
}
```

### AB点导航任务

所有6004开启的任务都可由6009进行手动停止

ros接口：action，类型：protocol/action/Navigation，名称："start_algo_task"

#### 下行指令

```Nginx
NameCode = 6004

params
{
  uint8 type #1是AB点导航
  float64 goalX #X坐标
  float64 goalY #Y坐标
  float64 theta #偏航角[-π，π)，如果此目标点没有角度，不要出现这个字段
}
```

#### 请求被接受

```Nginx
NameCode = 6004

data
{
  uint8 accepted #请求被接受时为1且任务继续运行；请求未被接受时为2且任务终止
}
```

#### 中间反馈

这三个反馈码适用于所有6004任务：1000正在激活依赖节点，1001激活依赖节点成功，1002激活依赖节点失败

```Nginx
NameCode = 6004

data
{
  int32 feedback_code #反馈码，成功，失败，正在运行（持续发送）,路径规划失败
  string feedback_msg # 额外消息，目前冗余
}
```

- 成功 
  - 导航启动成功，设置目标点成功，正在规划路径： 300
  - 正在导航中： 307
  - 到达目标点：308
- 失败
  - 底层导航失败：
    - 底层导航功能服务连接失败，请重新发送目标：302
    - 发送目标点失败，请重新发送目标：303
    - 底层导航功能失败，请重新发送目标：304
    - 目标点为空，请重新选择目标：305
    - 规划路径失败，请重新选择目标： 306
- 地图检查服务feedback_code ：
  - 正在检查地图：309
  - 地图检查成功： 310
  - 地图不存在，请重新建图： 311



#### 返回结果

```Nginx
NameCode = 6004

data
{
  uint8 result #导航结果，0成功，3失败，4拒绝，5取消，2未获取到，10超时没到终点
}
```

### 路径上传

用于AB点导航期间APP显示规划的路径

ros接口：topic，类型：nav_msgs/msg/Path，名称：“plan”

#### 上行指令

```Nginx
nameCode = 6006

params
{
  Point2D path_point[]  #二维点的数组
}
```

Point2D结构

```Nginx
float64 px #位置x坐标
float64 py #位置y坐标
```

### 激光点上报

ros接口：topic，类型：sensor_msgs/msg/LaserScan，名称："scan"

#### 上行指令

```Nginx
nameCode = 6011

params
{
  Point2D laser_point[]  #二维点的数组
}
```

## 充电

### 自动上桩任务

所有6004开启的任务都可由6009进行手动停止

ros接口：action，类型：protocol/action/Navigation，名称："start_algo_task"

#### 下行指令

```Nginx
NameCode = 6004

params
{
  uint8 type #9是自动上桩
}
```

#### 请求被接受

```Nginx
NameCode = 6004

data
{
  uint8 accepted #请求被接受时为1且任务继续运行；请求未被接受时为2且任务终止
}
```

#### 中间反馈

暂时未定义，可能会反馈自动上桩的中间状态、阶段信息

#### 返回结果

```Nginx
NameCode = 6004

data
{
  uint8 result #上桩结果，0成功，非0失败
}
```

## 视觉跟随

![](./image/grpc_protocol/grpc_vision_tracking.svg)

### 视觉跟随任务

所有6004开启的任务都可由6009进行手动停止

分为人体跟随和万物跟随，中间有反馈，正常运行时永远不会返回结果。

ros接口：action，类型：protocol/action/Navigation，名称："start_algo_task"

#### 下行指令

```Nginx
NameCode = 6004

params
{
  uint8 type #13是启动视觉识别
  bool object_tracking #true为万物跟随，false为人体目标跟随
  uint8 relative_pos # 相对方位，200自主选择跟随位置，201在目标后侧跟随，202在目标的左侧跟随，203在目标的右侧跟随
  float32 keep_distance # 与跟随目标所保持的距离
}
```

#### 请求被接受

```Nginx
NameCode = 6004

data
{
  uint8 accepted #请求被接受时为1且任务继续运行；请求未被接受时为2且任务终止
}
```

#### 中间反馈

这三个反馈码适用于所有6004任务：1000正在激活依赖节点，1001激活依赖节点成功，1002激活依赖节点失败

```Nginx
NameCode = 6004

data
{
  int32 feedback_code #反馈码，表示跟随任务的状态, 500表示视觉跟随功能启动中，501表示人体识别已启动，等待用户选择跟随目标，502表示跟随目标启动中，503表示跟随中，504暂时跟丢目标，尝试寻找原目标，504人体与万物跟随目标丢失, 寻找目标，505人体与万物跟随目标彻底丢失，重启跟随
  string feedback_msg # 额外消息，目前冗余
}
```

#### 返回结果

```Nginx
NameCode = 6004

data
{
  uint8 result #视觉跟随任务结果，0为成功（不会出现），非0为其它
}
```

### 跟随目标框列表

在开启人体识别任务请求成功后会上发跟随目标列表（万物跟随时不会上发），选中目标后跟随过程中会上发选中的目标框

#### 坐标转换

框的坐标是以640×480作为图像全尺寸的，与图传和手机分辨率不同，需要转换

ros接口：topic，类型：protocol/msg/Person，名称："person"

#### 上行指令

```Nginx
NameCode = 6007

param
{
  SelectTrackObject body_info_roi[]  #视觉人体跟随的人体候选目标框列表
  RegionOfInterest track_res_roi  #视觉跟随的目标框，如果未成功选中目标则高宽为0
}
```

##### SelectTrackObject结构

```Nginx
RegionOfInterest roi # 识别的框
string reid # 识别号
```

##### RegionOfInterest结构

```Nginx
uint32 x_offset # 框的起始像素坐标x
uint32 y_offset # 框的起始像素坐标y
uint32 height   # 框的像素高
uint32 width    # 框的像素宽
```

### 选中跟随目标框

6007上发的过程中，且还没有选择成功时，调用此请求，目标框的参数与上发的参数一致

ros接口：service，类型：protocol/srv/BodyRegion，名称："tracking_object_srv"

#### 下行指令

```Nginx
NameCode = 6008

param
{
  RegionOfInterest roi #选中的视觉跟随目标
}
```

#### 返回结果

```Nginx
NameCode = 6008

data
{
  bool success #是否成功选中，大概需要20~30秒
}
```

## UWB跟随

![](./image/grpc_protocol/grpc_uwb.svg)

### UWB跟随任务

所有6004开启的任务都可由6009进行手动停止

在机器狗和手环连接后（蓝牙连接后手环的UWB会自动和机器狗上的UWB连接）调用此请求

ros接口：action，类型：protocol/action/Navigation，名称："start_algo_task"

#### 下行指令

```Nginx
NameCode = 6004

params
{
  uint8 type #11是启动UWB跟随
  uint8 relative_pos # 相对方位，200自主选择跟随位置，201在目标后侧跟随，202在目标的左侧跟随，203在目标的右侧跟随
  float32 keep_distance # 与跟随目标所保持的距离
}
```

#### 请求被接受

```Nginx
NameCode = 6004

data
{
  uint8 accepted #请求被接受时为1且任务继续运行；请求未被接受时为2且任务终止
}
```

#### 中间反馈

这三个反馈码适用于所有6004任务：1000正在激活依赖节点，1001激活依赖节点成功，1002激活依赖节点失败

```Nginx
NameCode = 6004

data
{
  int32 feedback_code #反馈码，表示跟随任务的状态, 10正常，11检测器异常，12坐标转换异常，13规划器异常，14控制器异常，15UWB启动时目标为空，16UWB跟随中触发了跳台阶，17UWB跟随中触发了自主行为，18UWB跟随中的自主行为异常
  string feedback_msg # 额外消息，目前冗余
}
```

#### 返回结果

```Nginx
NameCode = 6004

data
{
  uint8 result #视觉跟随任务结果，0为成功（不会出现），非0为其它
}
```

## 接入进行中的任务（接入6004启动的任务）

6004开启的任务，如果中间APP退出，再次进入时如果任务仍然在进行，则可以通过此请求接入，接口与6004类似

ros接口：action，类型：protocol/action/Navigation，名称："start_algo_task"

#### 下行指令

```Nginx
NameCode = 6010

params
{
  uint8 type # 任务的类型
}
```

#### 请求被接受

```Nginx
NameCode = 6010

data
{
  uint8 accepted #1为接受，如果为3，则说明接入时任务已结束，或没有记录此类型任务（可能在之前app连接状态时已结束）
}
```

#### 中间反馈

与执行的任务一致

```Nginx
NameCode = 6010

data
{
  int32 feedback_code
  string feedback_msg
}
```

#### 返回结果

与执行的任务一致

```Nginx
NameCode = 6010

data
{
  uint8 result
}
```

## 停止任务（停止6004启动的任务）

所有6004开启的任务都可由6009进行手动停止

ros接口：service，类型：protocol/srv/StopAlgoTask，名称：“stop_algo_task”

### 停止当前任务

#### 下行指令

```Nginx
NameCode = 6009

params
{
  uint8 type #与对应任务启动时的type一致，如果停止所有任务就填0
  string map_name #如果是建图任务，则停止时需要写入地图名
}
```

类型定义参考：[算法任务管理接口](https://xiaomi.f.mioffice.cn/docs/dock4nE0rvOLin8zA5xjN1MSlpc) 

#### 返回结果

```Nginx
NameCode = 6009

data
{
  int8 result #停止指令是否被接受，0已停止，1停止失败，10请求超时无响应
}
```

## 蓝牙和UWB

UWB的连接需要通过蓝牙来建立，此处提供对蓝牙的操作，蓝牙连接和中断会包括UWB的相应操作

### 扫描蓝牙设备

扫描低功耗蓝牙设备，并返回设备信息列表（没有名称的设备不会返回），蓝牙已连接状态下调此请求会返回上一次扫描的结果（因为实验过连接状态下扫描会导致之后蓝牙断开时异常）

ros接口：service，类型：protocol/srv/BLEScan，名称："scan_bluetooth_device"

#### 下行指令

```Nginx
NameCode = 8001

param
{
  float64 scan_seconds #扫描时间（秒）
}
```

#### 返回结果

```Nginx
NameCode = 8001

data
{
  DeviceInfo device_info_list[] #扫描结果信息列表
}
```

##### DeviceInfo结构

```Nginx
string mac #mac地址
string name #蓝牙设备名
string addr_type #mac地址类型，可能为random或者public
uint8 device_type #uwb设备类型，0未知，16手环，17充电桩，未连接过的设备类型未知
string firmware_version #固件版本，已连接的设备才有正确的版本读数
float battery_level #电池电量，已连接的设备才有正确读数，范围：0~1
```

### 获取记录连接过的蓝牙设备信息

ros接口：service，类型：protocol/srv/BLEScan，名称："scan_bluetooth_device"

#### 下行指令

```Nginx
NameCode = 8001

param
{
  float64 scan_seconds #填0为获取历史记录
}
```

#### 返回结果

```Nginx
NameCode = 8001

data
{
  DeviceInfo device_info_list[] #历史设备信息列表
}
```

### 连接蓝牙设备

在扫描了蓝牙设备之后，连接选中的蓝牙设备，连接上蓝牙设备后会进行UWB设备连接

ros接口：service，类型：protocol/srv/BLEConnect，名称："connect_bluetooth_devices"

#### 下行指令

```Nginx
NameCode = 8002

param
{
  DeviceInfo selected_device #选中的设备名称
}
```

#### 返回结果

```Nginx
NameCode = 8002

data
{
  uint8 result #连接结果，0成功，1蓝牙连接失败，2UWB连接失败，3获取UWB的mac和session id失败
}
```

### 断开蓝牙设备

断开蓝牙设备，会一并断开对应的UWB设备

ros接口：service，类型：protocol/srv/BLEConnect，名称："connect_bluetooth_device"

#### 下行指令

```Nginx
NameCode = 8002

param
{
 #为空表示断开设备
}
```

#### 返回结果

```Nginx
NameCode = 8002

data
{
  uint8 result #断开结果，0成功，1蓝牙通信错误，2UWB断开失败，3当前蓝牙未连接
}
```

### 连接状态更新

蓝牙外设异常断开和自动重连，上报出来

ros接口：topic，类型：std_msgs/msg/Bool，名称："bluetooth_disconnected_unexpected"

#### 上发指令

```Nginx
NameCode = 8003

param
{
    bool disconnected #true为异常断开，false为自动回连成功
}
```

### 获取当前连接的设备

ros接口：service，类型：protocol/srv/BLEScan，名称："get_connected_bluetooth_info"

#### 下行指令

```Nginx
NameCode = 8004

param
{
}
```

#### 返回结果

```Nginx
NameCode = 8004

data
{
  DeviceInfo device_info_list[] #目前只支持连接一个设备，所以此列表只有一个或零个元素
}
```

### 获取当前连接的设备固件版本

ros接口：service，类型：std_srvs/srv/Trigger，名称："ble_device_firmware_version"

#### 下行指令

```Nginx
NameCode = 8005

param
{
}
```

#### 返回结果

```Nginx
NameCode = 8005

data
{
  bool success #true表示成功获取，false表示未连接设备
  string message #版本号
}
```

### 获取当前连接设备电量

ros接口：service，类型：protocol/srv/GetBLEBatteryLevel，名称："ble_device_battery_level"

#### 下行指令

```Nginx
NameCode = 8006

param
{
}
```

#### 返回结果

```Nginx
NameCode = 8006

data
{
  bool connected #true表示已连接设备，false表示未连接设备
  float32 persentage #电量：0.0~1.0
}
```

### 删除历史连接记录

ros接口：service，类型：nav2_msgs/srv/SaveMap，名称："delete_ble_devices_history"

#### 下行指令

```Nginx
NameCode = 8007

param
{
    string mac #要删除设备的mac地址，如果不填写则为删除所有记录
}
```

#### 返回结果

```Nginx
NameCode = 8007

data
{
  bool success #是否删除成功
}
```

### 蓝牙固件升级提示

在蓝牙外设连接且手机APP连接的状态下会检查是否有适配的固件，如果有则上发此提示

ros接口：topic，类型：std_msgs/msg/String，名称："ble_firmware_update_notification"

#### 上行指令

```Nginx
NameCode = 8008

param
{
  string data  # 格式为“旧版本 新版本”，中间用空格隔开
}
```

### 更新蓝牙固件

收到提示后可以下发升级确认，启动升级流程

ros接口：service，类型：std_srvs/srv/Trigger，名称："update_ble_firmware"

#### 下行指令

```Nginx
NameCode = 8009

param
{
}
```

#### 返回结果

```Nginx
NameCode = 8009

data
{
  bool success  # 是否升级成功
  string message  # 错误信息
}
```

### 蓝牙固件更新进度

在蓝牙固件升级过程中持续上发此信息

ros接口：topic，类型：protocol/msg/BLEDFUProgress，名称："ble_dfu_progress"

#### 上行指令

```Nginx
NameCode = 8010

param
{
  uint8 status  # 状态：0正在升级，1升级成功，2升级失败，3正在解压文件，4解压文件失败，5正在发送初始化包，6发送初始化包失败，7正在发送固件镜像，8发送固件镜像失败，9正在更新固件并等待重启，10未重启成功
  float64 progress  # 进度：0~1
  string message  # 状态信息，和状态相关的消息，比如：具体的升级步骤，失败原因等，如果不需要可以忽略
}
```

### 蓝牙遥控器速度挡位相关

#### 设置遥控器速度挡位

ros接口：topic，类型：std_msgs/msg/Int8，名称："set_bluetooth_tread"

##### 下行指令

```Nginx
NameCode = 8011

param
{
    int8 data #0低挡，1中挡，2高挡
}
```

#### 遥控器修改挡位上报通知

ros接口：topic，类型：std_msgs/msg/Int8，名称："update_bluetooth_tread"

##### 上行指令

```Nginx
NameCode = 8012

param
{
    int8 data #0低挡，1中挡，2高挡
}
```

#### 查询遥控器速度挡位

ros接口：service，类型：std_srvs/srv/Trigger，名称："get_bluetooth_tread"

##### 下行指令

```Nginx
NameCode = 8013

param
{
}
```

##### 返回结果

```Nginx
NameCode = 8013

data
{
    int8 data #0低挡，1中挡，2高挡
}
```

## 机器人状态

### 状态查询

心跳中包含的状态可通过此接口以查询的形式获得，心跳的格式请参考

此请求只涉及grpc节点，所以没有ros接口

#### 下行指令

```Nginx
NameCode = 10001

param
{
}
```

#### 返回结果

```Nginx
NameCode = 10001

data
{
  MotionStatus motion_status
  TaskStatus task_status
  SelfCheckStatus self_check_status
  StateSwitchStatus state_switch_status
  ChargingStatus charging_status
}
```

其中：

##### MotionStatus结构

```Nginx
int32 motion_id
```

##### TaskStatus结构

```Go
uint32 task_status
int32 task_sub_status
```

##### SelfCheckStatus结构

```Go
int32 code
string description
```

##### StateSwitchStatus结构

```Go
int32 state
int32 code
```

##### ChargingStatus结构

```C%2B%2B
bool wired_charging
bool wireless_charging
```

### 低功耗

#### 退出低功耗

APP可以手动退出低功耗而不用唤醒的方式退出，注意：此请求返回时低功耗彻底退出，建议超时设置15秒

ros接口：service，类型：std_srvs/srv/Trigger，名称："low_power_exit"

##### 下行指令

```Nginx
NameCode = 10002

param
{
}
```

##### 返回结果

```Nginx
NameCode = 10002

data
{
    bool success #是否退出成功
}
```

#### 开启/关闭自动进入低功耗

关闭之后，机器狗趴下不会自动进入低功耗模式

ros接口：service，类型：std_srvs/srv/SetBool，名称："low_power_onoff"

##### 下行指令

```Nginx
NameCode = 10003

param
{
    bool data #true为开启，false为关闭
}
```

##### 返回结果

```Nginx
NameCode = 10003

data
{
    bool success #是否设置成功
}
```

## 环境切换

ros接口：service，类型：bridge/protocol/ros/srv/Trigger.srv，名称：“set_work_environment”

### 下行指令

```Nginx
NameCode = 10004

param
{
    string data # 生产环境：pro；测试环境：test
}
```

### 返回结果

```Nginx
NameCode = 10004

data
{
    bool success # true：成功；false：失败
    string message
}
```

## Syslog日志上报

ros接口：service，类型：bridge/protocol/ros/srv/BesHttpSendFile.srv，名称：“upload_syslog”

### 下行指令

```Nginx
NameCode = 10005

param
{
}
```

### 返回结果

```Nginx
NameCode = 10005

data
{
    bool success # true：成功；false：失败
}
```

## 测试用的协议

### 断开app连接

ros接口：topic，类型：std_msgs/msg/Bool，名称："disconnect_app"

#### 下行指令

```Nginx
NameCode = 55001

params
{
}
```

#### 返回结果

```Nginx
NameCode = 55001

data
{
}
```

## 开发者权限

### 解锁指令

#### 下行指令

```C%2B%2B
NameCode = 9030

params
{
   string = httplink
}
```

#### 返回结果

```C%2B%2B
NameCode = 9030

params
{
   int result ;
}
```

### 重启指令

#### 下行指令

```C%2B%2B
NameCode = 9031

params
{
   string = SystemRestart
}
```

#### 返回结果

```C%2B%2B
NameCode = 9031

params
{
   int result ; #
                # 255 failed
}
```

### 关机和重启指令

ros接口：service，类型："std_srvs/srv/SetBool"，名称："enable_elec_skin"

## 电致变色

### 变色功能使能

ros接口：service，类型："std_srvs/srv/SetBool"，名称："enable_elec_skin"

#### 下行指令

```YAML
NameCode = 11001

param
{
    bool data # true开始 false关闭
}
```

#### 返回结果

```Go
NameCode = 11001

data
{
    bool success
    string message
}
```

### 设置变色模式

ros接口：service，类型："protocol/srv/ElecSkin"，名称："set_elec_skin"

#### 下行指令

```Nginx
NameCode = 11002

param
{
    int32 mode # 0-全黑 1-全白 2-前向后渐变 3-后向前渐变 4-闪烁 5-随机 6-动态（随落地腿变色）
    int32 wave_cycle_time # 0-5静态效果模式下表示变色时间ms（0和1参考值50，2~5参考值2000），6动态模式下0值表示落地腿变白，非零表示变深色
}
```

#### 返回结果

```Nginx
NameCode = 11002

data
{
    bool success 
}
```

## 狗腿校准

用户手动校准狗腿，先趴下断电，摆正后上电站起

ros接口：service，类型："std_srvs/srv/SetBool"，名称："dog_leg_calibration"

### 下行指令

```YAML
NameCode = 12001

param
{
    bool data # false趴下断电 true上电站起
}
```

### 返回结果

```Go
NameCode = 12001

data
{
    bool success
    string message
}
```
