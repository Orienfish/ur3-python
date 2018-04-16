# ur3-python
能够读取ur3机械臂位置信息、控制ur3移动、控制机械爪开关的python API。<br>
以Ｃ语言作为底层交互代码可以保证执行速度，因此将之前完成的基于modbus的C代码生成动态链接库，在python中调用，以此达到python接口控制机械臂和读取机械臂信息的目的。<br>

# 文件概览
* **pycontrol.py** <br>
核心python源码，提供的python函数接口如下：<br>
  * read_pos：读取基座坐标系下机械臂的位置。<br>无输入参数，返回一个c_float类型的数组，为当前基座坐标系中ur3 TCP的位置坐标[x,y,z,rx,ry,rz]，其中x,y,z的单位是米，rx,ry,rz的单位是rad，均保留到小数点后三位小数。<br>
  * read_wrist：读取机械臂的各关节的角度。<br>无输入参数，返回一个c_float类型的数组，为当前ur3机械臂六个关节的角度值[base,shoulder,elbow,wrist1,wrist2,wrist3]，单位是rad，保留到小数点后三位小数。由于modbus传输的只能是无符号16bit数，所以将机械臂的安全范围设定为：
![image](https://github.com/Orienfish/ur3-python/raw/master/safety.jpg)
  * send_movel_instruct：在基座坐标系下控制机械臂移动。<br>输入参数是一个c_float类型的指针，指向基座坐标系下目标位置的坐标[x,y,z,rx,ry,rz],其中x,y,z的单位是米，rx,ry,rz的单位是rad。无返回值，错误信息会显示在终端。<br>
  * send_movej_instruct：控制各关节角度移动到目标位置。<br>输入参数是一个c_float类型的指针，指向目标位置的各关节角度[base,shoulder,elbow,wrist1,wrist2,wrist3]，单位是rad。无返回值，错误信息会显示在终端。<br>
  * gripper_activate：激活机械爪，需要在每次使用机械爪前调用。<br>无输入参数，返回０表示成功，返回-１表示失败。<br>
  * gripper_open：打开机械爪。<br>输入参数是速度和力，两者作为全局变量在pycontrol.py的第11-12行定义。返回０表示成功，返回-1表示失败。<br>
  * gripper_close：闭合机械爪。<br>输入参数同样是速度和力。返回０表示成功，返回-1表示失败。<br>
* **modbustcp.c modbustcp.h** <br>
基于modbus TCP协议控制的Ｃ语言代码。主要建立TCP连接，并根据相关协议读取实时位置信息、发送控制指令给机械臂。<br>
* **modbusrtu.c modbusrtu.h** <br>
基于modbus RTU协议控制的Ｃ语言代码。主要通过USB控制机械爪开、关、激活。<br>
* **main.h** 保存很多宏定义变量的文件。<br>
* **lib.so** 由Ｃ代码生成的动态链接库。<br>生成指令为gcc -o lib.so -fPIC -shared modbustcp.c modbusrtu.c.<br>
