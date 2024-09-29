# EtherStorage

这是一个用于嵌入式设备的数据流存储器，在任务执行过程中将串口或者以太网的数据流存储到该设备的SD存储设备中，在任务结束后可通过串口或者以太网将存储的数据流读取出来。

## 接口外设

UART：460800bps

Ethernet：100Mbps

SDIO：48Mbps

## 开发环境

[STM32CubeMX](https://www.st.com.cn/zh/development-tools/stm32cubemx.html) (本项目使用6.12.0版本)

[IAR 9.50.2](https://pan.baidu.com/s/1OK0k6JNU_-GRZYnANJ5B-g?pwd=wata)

## 测试脚本及工具

[EthAssistant](https://github.com/dwgan/EthAssistant)

## 使用建议

### 测试UART

1. 给设备上电并下载程序，使用XCOM连接串口助手，观察输出

### 测试Ethernet

1. 下载程序并运行，连接串口和以太网
2. 设置网络连接，本项目中IP设置为`192.168.88.10`，子网掩码`255.255.255.0`，网关`192.168.88.1`
3. 使用命令提示符的`ipconfig`测试网络连接
4. 使用`python EthTestAssistant.py`脚本发送文件到以太网，并通过串口接收并比对



## 测试情况

```c
Start testing 
System Clock Frequency: 84000000 Hz
SDIO Clock Frequency: 21000000 Hz
SD 卡初始化成功！
开始写入数据到 SD 卡...
数据写入完成！
总写入时间: 2.22 秒
写入速度: 4.5106 MB/s
开始读取数据从 SD 卡...
已读取 4 MB...
已读取 9 MB...
数据读取完成！
总读取时间: 4.28 秒
读取速度: 2.33 MB/s
开始读取数据从 SD 卡...
已读取 4 MB...
已读取 9 MB...
数据读取和验证完成，写入和读取的数据一致！
```

