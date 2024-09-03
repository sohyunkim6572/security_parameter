## 1. Compile
```
cd prio_ros2
colcon build 
```

## 2. How to use?
```
# Publisher side
ros2 run motivation talker -r __node:=pub_$node -t sub_$node -p $period -s $pkt_num > $file_name

# Subscriber side
ros2 run motivation listener -r __node:=sub_$node -t sub_$node -u $util -p $period > $file_name
```
