import subprocess

# 设置脚本文件路径
script_path = "/home/haoran/catkin_ws/src/RFID_Based_Robot_Navigation/scripts/run_rfid_script.sh"

# 使用subprocess运行脚本
try:
    output = subprocess.check_output(["bash", script_path], text=True, stderr=subprocess.STDOUT)
    print(output)
except subprocess.CalledProcessError as e:
    print("Error:", e.output)