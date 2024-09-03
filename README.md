## 1. Compile
```
cd prio_ros2
colcon build 
source install/setup.bash
```

## 2. How to use?
```
//!! Publisher side의 topic 이름과 Subscriber side의 topic 이름이 동일해야한다. 
//!! $표시되어있는 부분은 원하는 값으로 변경하면 된다. 

//Publisher side
ros2 run motivation talker -r __node:=pub_$node -t sub_$node -p $period -s $pkt_num > $file_name

//Subscriber side
ros2 run motivation listener -r __node:=sub_$node -t sub_$node -u $util -p $period > $file_name
```
