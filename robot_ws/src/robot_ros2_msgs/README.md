# 通用消息文件软件包

ROS包 消息文件说明

## AckermannCurrent.msg

阿克曼底盘 电流

| 类型     | 名称     | 单位 | 含义     | 说明       |
|----------|----------|------|----------|------------|
| float32  | steering | A    | 转向电机 | 左正  右负 |
| float32  | left     | A    | 左电机   | 前正  后负 |
| float32  | right    | A    | 右电机   | 前正  后负 |

## AckermannSpeed.msg

阿克曼底盘 轮速 + 角度

| 类型     | 名称     | 单位   | 含义     | 说明       |
|----------|----------|--------|----------|------------|
| float32  | steering | rad    | 转向角度 | 左正  右负 |
| int16    | left     | mm/s   | 左轮速   | 前正  后负 |
| int16    | right    | mm/s   | 右轮速   | 前正  后负 |

## AutoCharge.msg

回充模块

| 类型  | 名称          | 单位    | 含义           | 说明                           |
|-------|---------------|---------|----------------|--------------------------------|
| bool  | online        | boolean | 回充在线       | false:离线  true:在线          |
| uint8 | state         | number  | 回充状态       | 0:空闲  1:红外回充  2:激光回充 |
| bool  | relays        | boolean | 回充头继电器   | false:关闭  true:打开          |
| bool  | limit\_switch | boolean | 回充桩行程开关 | false:未触发 true:触发         |
| bool  | voltage       | boolean | 回充头电压     | false:未检测到 true:检测到     |
| uint8 | infrared      | number  | 红外回充状态   | 0:寻找中心信号<br> 1:已找到中心信号<br> 2:信号丢失<br> 3:铜片接触<br> 4:对接成功<br> 5:对接错误（碰撞或充电过程中脱落）<br> 7:关闭回充底盘退出<br> 8:回充流程任务结束/红外回充任务为关闭状态 |

## BatteryState.msg

主板电池采集

| 类型    | 名称    | 单位 | 含义       | 说明             |
|---------|---------|------|------------|------------------|
| uint8   | percent | %    | 电量百分比 | 根据电压换算而来 |
| float32 | voltage | V    | 电池电压   | 单位  0.1V       |

## Bms.msg

主板BMS采集

| 类型      | 名称                        | 单位    | 含义       | 说明                      |
|-----------|-----------------------------|---------|------------|---------------------------|
| bool      | is\_offline                 | boolean | 是否离线   | false:在线  true:离线     |
| bool      | is\_warning                 | boolean | 是否告警   | false:无告警  true:有告警 |
| bool      | charge\_mos\_is\_open       | boolean | 充电MOS    | false:关闭  true:开启     |
| bool      | discharge\_mos\_is\_open    | boolean | 放电MOS    | false:关闭  true:开启     |
| uint8     | state                       | number  | 充放电状态 | 0:静止  1:充电  2:放电    |
| float32   | voltage                     | V       | 电压       |                           |
| float32   | current                     | A       | 电流       |                           |
| float32   | soc                         | %       | 电量       |                           |
| float32   | remain\_capacity            | Ah      | 剩余容量   |                           |
| int8      | maximum\_temperature\_value | C       | 最高温     |                           |
| int8      | minimum\_temperature\_value | C       | 最低温     |                           |
| uint8\[\] | warning                     | array   | 告警信息   |                           |

## ChassisDate.msg

底盘出货日期

| 类型  | 名称  | 单位   | 含义 | 说明       |
|-------|-------|--------|------|------------|
| uint8 | year  | number | 年   | 省略20开头 |
| uint8 | month | number | 月   | 1 \-> 12   |
| uint8 | day   | number | 日   | 1 \-> 31   |

## ChassisParameter.msg

底盘参数

| 类型   | 名称            | 单位   | 含义       | 说明 |
|--------|-----------------|--------|------------|------|
| uint16 | track\_width    | mm     | 轮距       |      |
| uint16 | wheel\_base     | mm     | 轴距       |      |
| uint16 | wheel\_diameter | mm     | 直径       |      |
| uint16 | gear\_ratio     | number | 减速比     |      |
| uint16 | encoder\_line   | number | 编码器线数 |      |

## ChassisState.msg

底盘状态

| 类型  | 名称                    | 单位    | 含义         | 说明                                       |
|-------|-------------------------|---------|--------------|--------------------------------------------|
| uint8 | control\_mode           | number  | 控制模式     | 0:空闲  1:遥控控制  2:通讯控制  3:外部控制 |
| bool  | auto\_charge\_enable    | boolean | 回充开启     | false:关闭  true:开启                      |
| bool  | motor\_drive\_error     | boolean | 驱动器报警   | false:未触发  true:触发                    |
| bool  | motor\_encoder\_error   | boolean | 编码器异常   | false:未触发  true:触发                    |
| bool  | stop\_button            | boolean | 急停按钮     | false:弹起  true:按下                      |
| bool  | remote\_control\_stop   | boolean | 遥控急停     | false:解除  true:生效                      |
| bool  | software\_stop          | boolean | 软件急停     | false:解除  true:生效                      |
| bool  | front\_collision        | boolean | 前防撞杆触发 | false:未触发  true:触发                    |
| bool  | back\_collision         | boolean | 后防撞杆触发 | false:未触发  true:触发                    |
| bool  | remote\_control\_online | boolean | 遥控在线     | false:离线  true:在线                      |
| bool  | motor\_drive\_online    | boolean | 驱动器在线   | false:离线  true:在线                      |

## ChassisVelocity.msg

底盘线速度角速度

| 类型    | 名称    | 单位    | 含义         | 说明 |
|---------|---------|---------|--------------|------|
| float64 | linear  | m/s     | 线速度       |      |
| float64 | angular | rad/s   | 角速度       |      |

## DaliBms.msg

电池BMS信息

| 类型        | 名称                                 | 单位    | 含义                      | 说明                   |
|-------------|--------------------------------------|---------|---------------------------|------------------------|
| float32     | cumulative\_total\_voltage           | V       | 累计总压                  |                        |
| float32     | gather\_total\_voltage               | V       | 采集总压                  |                        |
| float32     | current                              | A       | 电流                      | 充电正  放电负         |
| float32     | soc                                  | %       | SOC                       |                        |
| float32     | maximum\_cell\_voltage\_value        | V       | 最高单体电压值            |                        |
| uint8       | no\_of\_cell\_with\_maximum\_voltage | number  | 最高单体cell号            |                        |
| float32     | minimum\_cell\_voltage\_value        | V       | 最低单体电压值            |                        |
| uint8       | no\_of\_cell\_with\_minimum\_voltage | number  | 最低单体cell号            |                        |
| int8        | maximum\_temperature\_value          | C       | 最高单体温度值            |                        |
| uint8       | maximum\_temperature\_cell\_no       | number  | 最高单体温度cell号        |                        |
| int8        | minimum\_temperature\_value          | C       | 最低单体温度值            |                        |
| uint8       | minimum\_temperature\_cell\_no       | number  | 最低单体温度cell号        |                        |
| uint8       | state                                | number  | 充放电状态                | 0:静止  1:充电  2:放电 |
| uint8       | charge\_mos\_state                   | number  | 充电MOS管状态             |                        |
| uint8       | discharge\_mos\_status               | number  | 放电MOS管状态             |                        |
| uint8       | bms\_life                            | number  | 循环                      |                        |
| float32     | remain\_capacity                     | Ah      | 剩余容量                  |                        |
| uint8       | no\_of\_battery\_string              | number  | 电池串数                  |                        |
| uint8       | no\_of\_temperature                  | number  | 温度个数                  |                        |
| bool        | charger\_status                      | boolean | 充电器状态                | 0:断开  1:接入         |
| bool        | load\_status                         | boolean | 负载状态                  | 0:断开  1:接入         |
| bool\[4\]   | di\_state                            | array   | DI状态                    |                        |
| bool\[4\]   | do\_state                            | array   | DO状态                    |                        |
| float32\[\] | cell\_voltage                        | array   | 单体电压                  |                        |
| int8\[\]    | cell\_temperature                    | array   | 单体温度                  |                        |
| bool\[48\]  | cell\_balance\_state                 | array   | 均衡状态                  | 0:关闭  1:开启         |
| bool        | cell\_volt\_high\_level\_1           | boolean | 单体电压过高一级告警      |                        |
| bool        | cell\_volt\_high\_level\_2           | boolean | 单体电压过高二级告警      |                        |
| bool        | cell\_volt\_low\_level\_1            | boolean | 单体电压过低一级告警      |                        |
| bool        | cell\_volt\_low\_level\_2            | boolean | 单体电压过低二级告警      |                        |
| bool        | sum\_volt\_high\_level\_1            | boolean | 总压过高一级告警          |                        |
| bool        | sum\_volt\_high\_level\_2            | boolean | 总压过高二级告警          |                        |
| bool        | sum\_volt\_low\_level\_1             | boolean | 总压过低一级告警          |                        |
| bool        | sum\_volt\_low\_level\_2             | boolean | 总压过低二级告警          |                        |
| bool        | chg\_temp\_high\_level\_1            | boolean | 充电温度过高一级告警      |                        |
| bool        | chg\_temp\_high\_level\_2            | boolean | 充电温度过高二级告警      |                        |
| bool        | chg\_temp\_low\_level\_1             | boolean | 充电温度过低一级告警      |                        |
| bool        | chg\_temp\_low\_level\_2             | boolean | 充电温度过低二级告警      |                        |
| bool        | dischg\_temp\_high\_level\_1         | boolean | 放电温度过高一级告警      |                        |
| bool        | dischg\_temp\_high\_level\_2         | boolean | 放电温度过高二级告警      |                        |
| bool        | dischg\_temp\_low\_level\_1          | boolean | 放电温度过低一级告警      |                        |
| bool        | dischg\_temp\_low\_level\_2          | boolean | 放电温度过低二级告警      |                        |
| bool        | chg\_overcurrent\_level\_1           | boolean | 充电过流一级告警          |                        |
| bool        | chg\_overcurrent\_level\_2           | boolean | 充电过流二级告警          |                        |
| bool        | dischg\_overcurrent\_level\_1        | boolean | 放电过流一级告警          |                        |
| bool        | dischg\_overcurrent\_level\_2        | boolean | 放电过流二级告警          |                        |
| bool        | soc\_high\_level\_1                  | boolean | SOC过高一级告警           |                        |
| bool        | soc\_high\_level\_2                  | boolean | SOC过高二级告警           |                        |
| bool        | soc\_low\_level\_1                   | boolean | SOC过低一级告警           |                        |
| bool        | soc\_low\_level\_2                   | boolean | SOC过低二级告警           |                        |
| bool        | diff\_volt\_level\_1                 | boolean | 压差过大一级告警          |                        |
| bool        | diff\_volt\_level\_2                 | boolean | 压差过大二级告警          |                        |
| bool        | diff\_temp\_level\_1                 | boolean | 温差过大一级告警          |                        |
| bool        | diff\_temp\_level\_2                 | boolean | 温差过大二级告警          |                        |
| bool        | chg\_mos\_temp\_high\_alarm          | boolean | 充电MOS过温警告           |                        |
| bool        | dischg\_mos\_temp\_high\_alarm       | boolean | 放电MOS过温警告           |                        |
| bool        | chg\_mos\_temp\_sensor\_err          | boolean | 充电MOS温度检测传感器故障 |                        |
| bool        | dischg\_mos\_temp\_sensor\_err       | boolean | 放电MOS温度检测传感器故障 |                        |
| bool        | chg\_mos\_adhesion\_err              | boolean | 充电MOS粘连故障           |                        |
| bool        | dischg\_mos\_adhesion\_err           | boolean | 放电MOS粘连故障           |                        |
| bool        | chg\_mos\_open\_circuit\_err         | boolean | 充电MOS断路故障           |                        |
| bool        | dischg\_mos\_open\_circuit\_err      | boolean | 放电MOS断路故障           |                        |
| bool        | afe\_collect\_chip\_err              | boolean | AFE采集芯片故障           |                        |
| bool        | voltage\_collect\_dropped            | boolean | 单体采集掉线              |                        |
| bool        | cell\_temp\_sensor\_err              | boolean | 单体温度传感器故障        |                        |
| bool        | eeprom\_err                          | boolean | EEPROM存储故障            |                        |
| bool        | rtc\_err                             | boolean | RTC时钟故障               |                        |
| bool        | precharge\_failure                   | boolean | 预充失败                  |                        |
| bool        | communication\_failure               | boolean | 整车通信故障              |                        |
| bool        | internal\_communication\_failure     | boolean | 内网通信模块故障          |                        |
| bool        | current\_module\_fault               | boolean | 电流模块故障              |                        |
| bool        | sum\_voltage\_detect\_fault          | boolean | 内总压检测模块故障        |                        |
| bool        | short\_circuit\_protect\_fault       | boolean | 短路保护故障              |                        |
| bool        | low\_volt\_forbidden\_chg\_fault     | boolean | 低压禁止充电故障          |                        |
| uint8       | fault\_code                          | boolean | 故障码                    |                        |

## FourWheelDiffCurrent.msg

四轮差速底盘 电流

| 类型    | 名称         | 单位 | 含义 | 说明       |
|---------|--------------|------|------|------------|
| float32 | front\_left  | A    | 前左 | 前正  后负 |
| float32 | front\_right | A    | 前右 | 前正  后负 |
| float32 | rear\_left   | A    | 后左 | 前正  后负 |
| float32 | rear\_right  | A    | 后右 | 前正  后负 |

## FourWheelDiffSpeed.msg

四轮差速底盘 轮速

| 类型  | 名称         | 单位 | 含义 | 说明       |
|-------|--------------|------|------|------------|
| int16 | front\_left  | mm/s | 前左 | 前正  后负 |
| int16 | front\_right | mm/s | 前右 | 前正  后负 |
| int16 | rear\_left   | mm/s | 后左 | 前正  后负 |
| int16 | rear\_right  | mm/s | 后右 | 前正  后负 |

## FourWheelSteerAngle.msg

四轮独立转向底盘 角度

| 类型    | 名称         | 单位 | 含义     | 说明       |
|---------|--------------|------|----------|------------|
| float32 | front\_left  | rad  | 前左转向 | 左正  右负 |
| float32 | front\_right | rad  | 前右转向 | 左正  右负 |
| float32 | rear\_left   | rad  | 后左转向 | 左正  右负 |
| float32 | rear\_right  | rad  | 后右转向 | 左正  右负 |

## FourWheelSteerCurrent.msg

四轮独立转向底盘 电流

| 类型    | 名称                   | 单位 | 含义     | 说明       |
|---------|------------------------|------|----------|------------|
| float32 | front\_steering\_left  | A    | 前左转向 | 左正  右负 |
| float32 | front\_steering\_right | A    | 前右转向 | 左正  右负 |
| float32 | rear\_steering\_left   | A    | 后左转向 | 左正  右负 |
| float32 | rear\_steering\_right  | A    | 后右转向 | 左正  右负 |
| float32 | front\_left            | A    | 前左动力 | 前正  后负 |
| float32 | front\_right           | A    | 前右动力 | 前正  后负 |
| float32 | rear\_left             | A    | 后左动力 | 前正  后负 |
| float32 | rear\_right            | A    | 后右动力 | 前正  后负 |

## FourWheelSteerEncoder.msg

四轮独立转向底盘 编码器计数

| 类型   | 名称         | 单位   | 含义     | 说明 |
|--------|--------------|--------|----------|------|
| uint32 | front\_left  | number | 前左计数 |      |
| uint32 | front\_right | number | 前右计数 |      |
| uint32 | rear\_left   | number | 后左计数 |      |
| uint32 | rear\_right  | number | 后右计数 |      |

## FourWheelSteerMotion.msg

四轮独立转向底盘 运动状态

| 类型    | 名称         | 单位   | 含义       | 说明                               |
|---------|--------------|--------|------------|------------------------------------|
| uint8   | motion\_mode | number | 运动模式   | 0:阿克曼/自转  1:四轮独立旋转模式  |
| float32 | linear       | m/s    | 线速度     | 阿克曼模式有效 前正后负            |
| float32 | steering     | rad    | 角度       | 阿克曼模式有效 左正右负            |
| float32 | angular      | rad/s  | 角速度     | 阿克曼模式有效 自转优先 逆时针为正 |
| float32 | linear\_x    | m/s    | 前后线速度 | 四转模式有效 前正后负              |
| float32 | linear\_y    | m/s    | 左右线速度 | 四转模式有效 左正右负              |
| float32 | angular\_z   | rad/s  | 角速度     | 四转模式有效 逆时针为正            |

## FourWheelSteerSpeed.msg

四轮独立转向底盘 速度

| 类型  | 名称         | 单位 | 含义     | 说明       |
|-------|--------------|------|----------|------------|
| int16 | front\_left  | mm/s | 前左动力 | 前正  后负 |
| int16 | front\_right | mm/s | 前右动力 | 前正  后负 |
| int16 | rear\_left   | mm/s | 后左动力 | 前正  后负 |
| int16 | rear\_right  | mm/s | 后右动力 | 前正  后负 |

## HardwareVersion.msg

硬件版本

| 类型       | 名称    | 单位  | 含义     | 说明 |
|------------|---------|-------|----------|------|
| uint8\[8\] | version | array | 硬件版本 | 保留 |

## LedStrip.msg

灯条参数

| 类型    | 名称       | 单位   | 含义     | 说明                            |
|---------|------------|--------|----------|---------------------------------|
| uint8   | id         | number | 灯条编号 | 范围 1\-4                       |
| uint8   | mode       | number | 灯光模式 | 0:常亮  1:闪烁  2:呼吸          |
| uint8   | brightness | %      | 亮度     | 范围 0\-100                     |
| float32 | interval   | s      | 间隔时间 | 范围 0\.0 \- 25\.5 一位小数有效 |
| uint8   | r          | number | 红       |                                 |
| uint8   | g          | number | 绿       |                                 |
| uint8   | b          | number | 蓝       |                                 |


## LedStrips.msg

多路 灯条参数

| 类型       | 名称 | 单位   | 含义     | 说明 |
|------------|------|--------|----------|------|
| struct\[\] | leds | struct | 多路灯条 | 保留 |

## RemoteControl.msg

遥控数据

| 类型  | 名称              | 单位    | 含义         | 说明                     |
|-------|-------------------|---------|--------------|--------------------------|
| bool  | online            | boolean | 遥控在线状态 | false:离线  true:在线    |
| int16 | rocker\_left\_x   | number  | 左摇杆左右   | 左:\-584  中:0  右:583   |
| int16 | rocker\_left\_y   | number  | 左摇杆上下   | 下:\-584  中:0  上:583   |
| int16 | rocker\_right\_x  | number  | 右摇杆左右   | 左:\-584  中:0  右:583   |
| int16 | rocker\_right\_y  | number  | 右摇杆上下   | 下:\-584  中:0  上:583   |
| int16 | round\_left\_vra  | number  | 左拨轮前后   | 后:\-584  中:0  前:583   |
| int16 | round\_right\_vrb | number  | 右拨轮前后   | 后:\-584  中:0  前:583   |
| uint8 | key\_swa          | number  | 拨杆swa      | 0:下  1:中  2:上  3:保留 |
| uint8 | key\_swb          | number  | 拨杆swb      | 0:下  1:中  2:上  4:保留 |
| uint8 | key\_swc          | number  | 拨杆swc      | 0:下  1:中  2:上  5:保留 |
| uint8 | key\_swd          | number  | 拨杆swd      | 0:下  1:中  2:上  6:保留 |

## SoftwareVersion.msg

软件版本

| 类型       | 名称     | 单位   | 含义         | 说明                 |
|------------|----------|--------|--------------|----------------------|
| uint8      | major    | number | 主版本号     |                      |
| uint8      | minor    | number | 次版本号     |                      |
| uint8      | revision | number | 修订版本号   |                      |
| uint8      | reserved | number | 保留         |                      |
| uint8      | year     | number | 年           |                      |
| uint8      | month    | number | 月           |                      |
| uint8      | day      | number | 日           |                      |
| uint8      | other    | number | 其他         |                      |
| uint8\[8\] | version  | array  | 软件版本信息 | 软件版本原始数据数组 |

## TwoWheelDiffCurrent.msg

两轮差速底盘 电流

| 类型    | 名称  | 单位 | 含义 | 说明       |
|---------|-------|------|------|------------|
| float32 | left  | A    | 左   | 前正  后负 |
| float32 | right | A    | 右   | 前正  后负 |

## TwoWheelDiffSpeed.msg

两轮差速底盘 轮速

| 类型  | 名称  | 单位 | 含义 | 说明       |
|-------|-------|------|------|------------|
| int16 | left  | mm/s | 左   | 前正  后负 |
| int16 | right | mm/s | 右   | 前正  后负 |

## 版本信息

当前: V 1.0.2 <br>

V 1.0.2 <br>
++增加主板采集BMS消息文件

V 1.0.1 <br>
++增加BMS消息文件

V 1.0.0 <br>
++首次创建