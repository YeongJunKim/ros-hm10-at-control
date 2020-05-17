# ROS-HM10 command
<img src="./etc/img/hm10.jpg" height="300px" width="300px">

ROS에서 hm10 AT 명령어 사용하기.

hm10 제작사 홈페이지, 
```
http://jnhuamao.cn/bluetooth.asp?id=1
```


### 사용법/usage
#### run
```
$: rosrun hm10 hm10 /dev/tty${device name}
```
<img src="./etc/img/usage_command.png">

#### topic name : std_msgs::String
```
/HM/command
```