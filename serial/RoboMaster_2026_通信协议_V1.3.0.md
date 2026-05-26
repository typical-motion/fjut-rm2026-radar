# RoboMaster 2026 机甲大师高校系列赛通信协议 V1.3.0

> 协议版本：V1.3.0（2026.03.27）
> 适用范围：RMUC（超级对抗赛）& RMUL（高校联盟赛）

---

## 1 串口协议

### 1.1 串口协议格式

**串口参数**：8 位数据位，1 位停止位，无硬件流控，无校验位

**波特率**：

| 物理接口 | 波特率 |
|---|---|
| 电源管理模块 ←→ 机器人 | 115200 |
| 自定义控制器 ←→ 选手端 | 115200 |
| 图传发送端 ←→ 机器人 | 921600 |

**通信协议格式**：

| frame_header | cmd_id | data | frame_tail |
|---|---|---|---|
| 5-byte | 2-byte | n-byte | 2-byte，CRC16 整包校验 |

**frame_header 格式**：

| SOF | data_length | seq | CRC8 |
|---|---|---|---|
| 1-byte | 2-byte | 1-byte | 1-byte |

**帧头详细定义**：

| 域 | 偏移位置 | 大小（字节） | 详细描述 |
|---|---|---|---|
| SOF | 0 | 1 | 数据帧起始字节，固定值 **0xA5** |
| data_length | 1 | 2 | 数据帧中 data 的长度 |
| seq | 3 | 1 | 包序号 |
| CRC8 | 4 | 1 | 帧头 CRC8 校验 |

**数据链路分类**：

裁判系统串口数据链路有三种：

- **常规链路**：由裁判系统服务器和主控模块进行数据转发，从电源管理模块的 User 串口收发数据
- **图传链路**：由裁判系统选手端和图传模块进行数据转发，从图传模块（发送端）的串口接收数据
- **雷达无线链路**：由裁判系统信号发射源进行数据发送，从雷达接收电磁波并解析信息

> 正常工作状态下，裁判系统数据延迟约为 130ms，丢包率小于 1%；赛场网络环境较恶劣时，数据延迟约为 200ms，丢包率约为 3%。

---

### 1.2 命令码 ID 和常规链路数据说明

#### 命令码 ID 一览

| 命令码 | 数据段长度 | 说明 | 发送方/接收方 | 数据链路 |
|---|---|---|---|---|
| 0x0001 | 11 | 比赛状态数据，固定以 1Hz 频率发送 | 服务器 → 全体机器人 | 常规链路 |
| 0x0002 | 1 | 比赛结果数据，比赛结束触发发送 | 服务器 → 全体机器人 | 常规链路 |
| 0x0003 | 16 | 机器人血量数据，固定以 3Hz 频率发送 | 服务器 → 全体机器人 | 常规链路 |
| 0x0101 | 4 | 场地事件数据，固定以 1Hz 频率发送 | 服务器 → 己方全体机器人 | 常规链路 |
| 0x0104 | 3 | 裁判警告数据，己方判罚/判负时触发发送 | 服务器 → 被判罚方全体机器人 | 常规链路 |
| 0x0105 | 3 | 飞镖发射相关数据，固定以 1Hz 频率发送 | 服务器 → 己方全体机器人 | 常规链路 |
| 0x0201 | 13 | 机器人性能体系数据，固定以 10Hz 频率发送 | 主控模块 → 对应机器人 | 常规链路 |
| 0x0202 | 14 | 实时底盘缓冲能量和射击热量数据，固定以 10Hz 频率发送 | 主控模块 → 对应机器人 | 常规链路 |
| 0x0203 | 16 | 机器人位置数据，固定以 1Hz 频率发送 | 主控模块 → 对应机器人 | 常规链路 |
| 0x0204 | 8 | 机器人增益和底盘能量数据，固定以 3Hz 频率发送 | 服务器 → 对应机器人 | 常规链路 |
| 0x0206 | 1 | 伤害状态数据，伤害发生后发送 | 主控模块 → 对应机器人 | 常规链路 |
| 0x0207 | 7 | 实时射击数据，弹丸发射后发送 | 主控模块 → 对应机器人 | 常规链路 |
| 0x0208 | 6 | 允许发弹量，固定以 10Hz 频率发送 | 服务器 → 己方英雄、步兵、哨兵、空中机器人 | 常规链路 |
| 0x0209 | 5 | 机器人 RFID 模块状态，固定以 3Hz 频率发送 | 服务器 → 己方装有 RFID 模块的机器人 | 常规链路 |
| 0x020A | 6 | 飞镖选手端指令数据，固定以 3Hz 频率发送 | 服务器 → 己方飞镖机器人 | 常规链路 |
| 0x020B | 40 | 地面机器人位置数据，固定以 1Hz 频率发送 | 服务器 → 己方哨兵机器人 | 常规链路 |
| 0x020C | 2 | 雷达标记进度数据，固定以 1Hz 频率发送 | 服务器 → 己方雷达机器人 | 常规链路 |
| 0x020D | 6 | 哨兵自主决策信息同步，固定以 1Hz 频率发送 | 服务器 → 己方哨兵机器人 | 常规链路 |
| 0x020E | 1 | 雷达自主决策信息同步，固定以 1Hz 频率发送 | 服务器 → 己方雷达机器人 | 常规链路 |
| 0x0301 | 118 | 机器人交互数据，发送方触发发送，频率上限 30Hz | — | 常规链路 |
| 0x0302 | 30 | 自定义控制器与机器人交互数据，频率上限 30Hz | 自定义控制器 → 选手端图传连接的机器人 | 图传链路 |
| 0x0303 | 15 | 选手端小地图交互数据，选手端触发发送 | 选手端 → 服务器 → 己方机器人 | 常规链路 |
| 0x0305 | 48 | 选手端小地图接收雷达数据，频率上限 5Hz | 雷达 → 服务器 → 己方所有选手端 | 常规链路 |
| 0x0306 | 8 | 自定义控制器与选手端交互数据，频率上限 30Hz | 自定义控制器 → 选手端 | — |
| 0x0307 | 103 | 选手端小地图接收路径数据，频率上限 1Hz | 哨兵/半自动控制机器人 → 对应操作手选手端 | 常规链路 |
| 0x0308 | 34 | 选手端小地图接收机器人数据，频率上限 3Hz | 己方机器人 → 己方选手端 | 常规链路 |
| 0x0309 | 30 | 自定义控制器接收机器人数据，频率上限 10Hz | 己方机器人 → 对应操作手选手端连接的自定义控制器 | 图传链路 |
| 0x0310 | 300 | 机器人发送给自定义客户端的数据，频率上限 50Hz | 己方机器人 → 图传链路 → 自定义客户端 | 图传链路 |
| 0x0311 | 30 | 自定义客户端发送给机器人的自定义指令，频率上限 75Hz | 自定义客户端 → 图传链路 → 己方机器人 | 图传链路 |
| **0x0A01** | **24** | **对方机器人的位置坐标，以 10Hz 频率持续发送** | **信号发射源 → 雷达** | **雷达无线链路** |
| **0x0A02** | **12** | **对方机器人的血量信息，以 10Hz 频率持续发送** | **信号发射源 → 雷达** | **雷达无线链路** |
| **0x0A03** | **10** | **对方机器人的剩余发弹量信息，以 10Hz 频率持续发送** | **信号发射源 → 雷达** | **雷达无线链路** |
| **0x0A04** | **8** | **对方队伍的宏观状态信息，以 10Hz 频率持续发送** | **信号发射源 → 雷达** | **雷达无线链路** |
| **0x0A05** | **36** | **对方各机器人当前增益效果，以 10Hz 频率持续发送** | **信号发射源 → 雷达** | **雷达无线链路** |
| **0x0A06** | **6** | **对方干扰波密钥，以 10Hz 频率持续发送** | **信号发射源 → 雷达** | **雷达无线链路** |

#### 关键命令码详细定义

##### 0x0001 — 比赛状态数据

```c
typedef _packed struct {
    uint8_t game_type : 4;      // bit 0-3: 比赛类型
    uint8_t game_progress : 4;  // bit 4-7: 当前比赛阶段
    uint16_t stage_remain_time; // 当前阶段剩余时间(秒)
    uint64_t SyncTimeStamp;     // UNIX 时间
} game_status_t;
```

**比赛类型**：
- 1: RoboMaster 机甲大师超级对抗赛
- 2: RoboMaster 机甲大师高校单项赛
- 3: ICRA RoboMaster 高校人工智能挑战赛
- 4: RoboMaster 机甲大师高校联盟赛 3V3 对抗
- 5: RoboMaster 机甲大师高校联盟赛步兵对抗

**比赛阶段**：
- 0: 未开始比赛
- 1: 准备阶段
- 2: 十五秒裁判系统自检阶段
- 3: 五秒倒计时
- 4: 比赛中
- 5: 比赛结算中

##### 0x0002 — 比赛结果

```c
typedef _packed struct {
    uint8_t winner;  // 0: 平局, 1: 红方胜利, 2: 蓝方胜利
} game_result_t;
```

##### 0x0003 — 机器人血量

```c
typedef _packed struct {
    uint16_t ally_1_robot_HP;    // 己方 1 号英雄机器人血量
    uint16_t ally_2_robot_HP;    // 己方 2 号工程机器人血量
    uint16_t ally_3_robot_HP;    // 己方 3 号步兵机器人血量
    uint16_t ally_4_robot_HP;    // 己方 4 号步兵机器人血量
    uint16_t reserved;           // 保留位
    uint16_t ally_7_robot_HP;    // 己方 7 号哨兵机器人血量
    uint16_t ally_outpost_HP;    // 己方前哨站血量
    uint16_t ally_base_HP;       // 己方基地血量
} game_robot_HP_t;
```

##### 0x0201 — 机器人性能体系数据

```c
typedef _packed struct {
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
} robot_status_t;
```

##### 0x0203 — 机器人位置数据

```c
typedef _packed struct {
    float x;      // 本机器人位置 x 坐标，单位：m
    float y;      // 本机器人位置 y 坐标，单位：m
    float angle;  // 本机器人测速模块的朝向，单位：度，正北为 0 度
} robot_pos_t;
```

##### 0x0301 — 机器人交互数据

```c
typedef _packed struct {
    uint16_t data_cmd_id;    // 子内容 ID
    uint16_t sender_id;      // 发送者 ID
    uint16_t receiver_id;    // 接收者 ID
    uint8_t user_data[x];    // 内容数据段，最大 112 字节
} robot_interaction_data_t;
```

**子内容 ID**：

| 子内容 ID | 数据段长度 | 功能说明 |
|---|---|---|
| 0x0200~0x02FF | ≤112 | 机器人之间通信 |
| 0x0100 | 2 | 选手端删除图层 |
| 0x0101 | 15 | 选手端绘制一个图形 |
| 0x0102 | 30 | 选手端绘制两个图形 |
| 0x0103 | 75 | 选手端绘制五个图形 |
| 0x0104 | 105 | 选手端绘制七个图形 |
| 0x0110 | 45 | 选手端绘制字符图形 |
| 0x0120 | 4 | 哨兵自主决策指令 |
| 0x0121 | 1 | 雷达自主决策指令 |

---

### 1.3 雷达无线链路数据说明

#### 0x0A01 — 对方机器人位置坐标（24 字节，10Hz）

```c
typedef _packed struct {
    int16_t hero_x;        // 对方英雄机器人位置 x 坐标，单位：cm
    int16_t hero_y;        // 对方英雄机器人位置 y 坐标，单位：cm
    int16_t engineer_x;    // 对方工程机器人位置 x 坐标，单位：cm
    int16_t engineer_y;    // 对方工程机器人位置 y 坐标，单位：cm
    int16_t standard_3_x;  // 对方 3 号步兵机器人位置 x 坐标，单位：cm
    int16_t standard_3_y;  // 对方 3 号步兵机器人位置 y 坐标，单位：cm
    int16_t standard_4_x;  // 对方 4 号步兵机器人位置 x 坐标，单位：cm
    int16_t standard_4_y;  // 对方 4 号步兵机器人位置 y 坐标，单位：cm
    int16_t air_force_x;   // 对方空中机器人位置 x 坐标，单位：cm
    int16_t air_force_y;   // 对方空中机器人位置 y 坐标，单位：cm
    int16_t sentry_x;      // 对方哨兵机器人位置 x 坐标，单位：cm
    int16_t sentry_y;      // 对方哨兵机器人位置 y 坐标，单位：cm
} enemy_position_t;        // 总计 24 字节
```

#### 0x0A02 — 对方机器人血量信息（12 字节，10Hz）

```c
typedef _packed struct {
    uint16_t hero_HP;       // 对方 1 号英雄机器人血量
    uint16_t engineer_HP;   // 对方 2 号工程机器人血量
    uint16_t standard_3_HP; // 对方 3 号步兵机器人血量
    uint16_t standard_4_HP; // 对方 4 号步兵机器人血量
    uint16_t reserved;      // 保留位
    uint16_t sentry_HP;     // 对方 7 号哨兵机器人血量
} enemy_HP_t;               // 总计 12 字节
```

#### 0x0A03 — 对方机器人剩余发弹量信息（10 字节，10Hz）

| 偏移 | 大小 | 说明 |
|---|---|---|
| 0 | 2 | 对方 1 号英雄机器人允许发弹量 |
| 2 | 2 | 对方 3 号步兵机器人允许发弹量 |
| 4 | 2 | 对方 4 号步兵机器人允许发弹量 |
| 6 | 2 | 对方 6 号空中机器人允许发弹量 |
| 8 | 2 | 对方 7 号哨兵机器人允许发弹量 |

#### 0x0A04 — 对方队伍宏观状态信息（8 字节，10Hz）

| 偏移 | 大小 | 说明 |
|---|---|---|
| 0 | 2 | 对方剩余金币数 |
| 2 | 2 | 对方累计总金币数 |
| 4 | 4 | 位域状态（见下方） |

**位域状态 bit 定义**：
- bit 0：对方补给区占领状态
- bit 1-2：对方中央高地的占领状态（1=被对方占领，2=被己方占领）
- bit 3：对方梯形高地的占领状态
- bit 4-5：对方堡垒增益点的占领状态
- bit 6-7：对方前哨站增益点的占领状态
- bit 8：对方基地增益点的占领状态
- bit 9-15：各场地交互模块卡检测状态

#### 0x0A05 — 对方各机器人当前增益效果（36 字节，10Hz）

按机器人分组，每组 6 字节（回血增益 + 热量冷却增益 + 防御增益 + 负防御增益 + 攻击增益），共 6 组：

| 偏移 | 大小 | 说明 |
|---|---|---|
| 0-5 | 6 | 对方英雄机器人增益 |
| 6-11 | 6 | 对方工程机器人增益 |
| 12-17 | 6 | 对方 3 号步兵机器人增益 |
| 18-23 | 6 | 对方 4 号步兵机器人增益 |
| 24-29 | 6 | 对方哨兵机器人增益 |
| 30-35 | 1 | 对方哨兵机器人当前姿态 |

**每组增益格式**：
| 偏移(组内) | 大小 | 说明 |
|---|---|---|
| 0 | 1 | 回血增益（百分比） |
| 1 | 2 | 射击热量冷却增益（直接值） |
| 3 | 1 | 防御增益（百分比） |
| 4 | 1 | 负防御增益（百分比） |
| 5 | 2 | 攻击增益（百分比） |

**哨兵姿态**：1=进攻姿态，2=防御姿态，3=移动姿态

#### 0x0A06 — 对方干扰波密钥（6 字节，10Hz）

每个字节均为 ASCII 码编码的字母或数字。

---

## 2 自定义客户端协议

- **数据格式**：Protobuf v3（Protocol Buffers），支持版本 3.15 以上
- **传输协议**：MQTT，作为发布/订阅消息传输协议
- **服务器 IP**：固定 IP `192.168.12.1`，端口 `3333`
- **自定义客户端 IP**：`192.168.12.2`
- **图传码流**：可通过 UDP 监听 3334 端口获取，编码格式 HEVC，每个 UDP 包的前 8 字节固定为帧编号(2B) + 分片序号(2B) + 总字节数(4B)

**通信流程**：
1. 结合文档中的 Protobuf 消息格式编写 proto 文件，用 protoc 生成不同语言的类代码
2. 在发布者端，将对象序列化成二进制
3. 通过 MQTT publish 把二进制发送到对应 Topic
4. 在订阅者端，接收消息并用 Protobuf 反序列化

**指令概览**（部分关键指令）：

| 指令名 | 用途 | 发送方/接收方 | 最高 QoS | 频率 |
|---|---|---|---|---|
| KeyboardMouseControl | 传输鼠标键盘输入 | 自定义客户端 → 图传链路 → 机器人 | 1 | 75Hz |
| CustomControl | 自定义控制器指令 | 自定义客户端 → 图传链路 → 机器人 | 1 | 75Hz |

---

## 附录一：CRC 校验代码示例

### CRC8 校验

生成多项式：G(x) = x⁸ + x⁵ + x⁴ + 1

```c
const unsigned char CRC8_INIT = 0xff;
const unsigned char CRC8_TAB[256] = {
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8) {
    unsigned char ucIndex;
    while (dwLength--) {
        ucIndex = ucCRC8 ^ (*pchMessage++);
        ucCRC8 = CRC8_TAB[ucIndex];
    }
    return ucCRC8;
}

unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength) {
    unsigned char ucExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2)) return 0;
    ucExpected = Get_CRC8_Check_Sum(pchMessage, dwLength - 1, CRC8_INIT);
    return (ucExpected == pchMessage[dwLength - 1]);
}

void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength) {
    unsigned char ucCRC = 0;
    if ((pchMessage == 0) || (dwLength <= 2)) return;
    ucCRC = Get_CRC8_Check_Sum((unsigned char *)pchMessage, dwLength - 1, CRC8_INIT);
    pchMessage[dwLength - 1] = ucCRC;
}
```

### CRC16 校验

```c
uint16_t CRC_INIT = 0xffff;
const uint16_t wCRC_Table[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78,
};

uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC) {
    uint8_t chData;
    if (pchMessage == NULL) return 0xFFFF;
    while (dwLength--) {
        chData = *pchMessage++;
        (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
    }
    return wCRC;
}

uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength) {
    uint16_t wExpected = 0;
    if ((pchMessage == NULL) || (dwLength <= 2)) return 0;
    wExpected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC_INIT);
    return ((wExpected & 0xff) == pchMessage[dwLength - 2] &&
            ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength) {
    uint16_t wCRC = 0;
    if ((pchMessage == NULL) || (dwLength <= 2)) return;
    wCRC = Get_CRC16_Check_Sum((uint8_t *)pchMessage, dwLength - 2, CRC_INIT);
    pchMessage[dwLength - 2] = (uint8_t)(wCRC & 0x00ff);
    pchMessage[dwLength - 1] = (uint8_t)((wCRC >> 8) & 0x00ff);
}
```

---

## 附录二：ID 编号说明

### 机器人 ID

| ID | 红方 | ID | 蓝方 |
|---|---|---|---|
| 1 | 英雄机器人 | 101 | 英雄机器人 |
| 2 | 工程机器人 | 102 | 工程机器人 |
| 3/4/5 | 步兵机器人 | 103/104/105 | 步兵机器人 |
| 6 | 空中机器人 | 106 | 空中机器人 |
| 7 | 哨兵机器人 | 107 | 哨兵机器人 |
| 8 | 飞镖 | 108 | 飞镖 |
| 9 | 雷达 | 109 | 雷达 |
| 10 | 前哨站 | 110 | 前哨站 |
| 11 | 基地 | 111 | 基地 |

### 选手端 ID

| ID | 说明 |
|---|---|
| 0x0101 | 红方英雄机器人选手端 |
| 0x0102 | 红方工程机器人选手端 |
| 0x0103/0x0104/0x0105 | 红方步兵机器人选手端 |
| 0x0106 | 红方空中机器人选手端 |
| 0x0165 | 蓝方英雄机器人选手端 |
| 0x0166 | 蓝方工程机器人选手端 |
| 0x0167/0x0168/0x0169 | 蓝方步兵机器人选手端 |
| 0x016A | 蓝方空中机器人选手端 |
| 0x8080 | 裁判系统服务器（用于哨兵和雷达自主决策指令） |
