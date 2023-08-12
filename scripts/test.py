#!/usr/bin/env python
import io
import os
import time
import speech_recognition as sr

from datetime import datetime, timedelta
from queue import Queue
from time import sleep
from sys import platform

import requests
import json
import base64
import subprocess

import atexit
import rospy
from std_msgs.msg import Int32

import pygame  # 导入用于播放音频的库

# 初始化pygame mixer，这是播放音频所必需的
pygame.mixer.init()

def play_audio(audio_file="/home/haoran/audio/welcome1.wav"):
    pygame.mixer.music.load(audio_file)  # 加载你的音频文件
    pygame.mixer.music.play()  # 播放音频文件

headers = {"Content-Type": "application/json"}
url = "http://124.221.141.85:42200/transcribe"
clear_url = "http://124.221.141.85:42200/clear_cache"

revive_keyword = ['丁真同学']
forward_keyword = ['前进', '前方', '前面', '向前', '前行']
backward_keyword = ['后退', '后方', '后面', '向后', '后行']
left_keyword = ['左转', '左边', '向左', '左行', '左方']
right_keyword = ['右转', '右边', '向右', '右行', '右方']
stop_keyword = ['停止', '停下', '停车', '不要动', '别动', '暂停']
specific_keyword = ['开启导航', '开始导航', '运行导航', '执行导航', '启动导航']
mama_keyword = ['哪个省', '哪个生', '那个省', '那个生']

welcome_audio = ['/home/haoran/audio/welcome1.wav', '/home/haoran/audio/welcome2.wav']

def clear_cache():
    data = {
        "clear" : True}
    request = requests.post(clear_url, data=json.dumps(data), headers=headers)

atexit.register(clear_cache)

def callback(msg):
    play_audio("/home/haoran/audio/amazing.wav")


def main():
    
    print("init ros")
    rospy.init_node("audio_recorder", anonymous=True)
    print("init pub")
    info_pub = rospy.Publisher("/audio_info", Int32, queue_size=10)
    goal_sub = rospy.Subscriber("/reach_signal", Int32, callback=callback)
    print("all done")

    # The last time a recording was retreived from the queue.
    phrase_time = None
    # Current raw audio bytes.
    last_sample = bytes()
    # Thread safe Queue for passing data from the threaded recording callback.
    data_queue = Queue()
    # We use SpeechRecognizer to record our audio because it has a nice feauture where it can detect when speech ends.
    recorder = sr.Recognizer()
    recorder.energy_threshold = 1000
    # Definitely do this, dynamic energy compensation lowers the energy threshold dramtically to a point where the SpeechRecognizer never stops recording.
    recorder.dynamic_energy_threshold = False
    
    print("init done")
    # Important for linux users. 
    # Prevents permanent application hang and crash by using the wrong Microphone
    if 'linux' in platform:
        mic_name = 'pulse'
        if not mic_name or mic_name == 'list':
            print("Available microphone devices are: ")
            for index, name in enumerate(sr.Microphone.list_microphone_names()):
                print(f"Microphone with name \"{name}\" found")   
            return
        else:
            for index, name in enumerate(sr.Microphone.list_microphone_names()):
                if mic_name in name:
                    source = sr.Microphone(sample_rate=16000, device_index=index)
                    break
    else:
        source = sr.Microphone(sample_rate=16000)
    # source = sr.Microphone(sample_rate=16000, device_index=5)

    record_timeout = 2
    phrase_timeout = 3

    temp_file = 'temp/temp.wav'
    transcription = ['']
    translate_transcription = ['']
    
    with source:
        recorder.adjust_for_ambient_noise(source)

    def record_callback(_, audio:sr.AudioData) -> None:
        """
        Threaded callback function to recieve audio data when recordings finish.
        audio: An AudioData containing the recorded bytes.
        """
        # Grab the raw bytes and push it into the thread safe queue.
        data = audio.get_raw_data()
        data_queue.put(data)

    # Create a background thread that will pass us raw audio bytes.
    # We could do this manually but SpeechRecognizer provides a nice helper.
    recorder.listen_in_background(source, record_callback, phrase_time_limit=record_timeout)

    isRecog = False

    # Cue the user that we're ready to go.
    print("Model loaded.\n")

    while True:
        try:
            now = datetime.utcnow()
            # Pull raw recorded audio from the queue.
            if not data_queue.empty():
                phrase_complete = False
                # If enough time has passed between recordings, consider the phrase complete.
                # Clear the current working audio buffer to start over with the new data.
                if phrase_time and now - phrase_time > timedelta(seconds=phrase_timeout):
                    last_sample = bytes()
                    phrase_complete = True
                # This is the last time we received new audio data from the queue.
                phrase_time = now

                # Concatenate our current audio data with the latest audio data.
                while not data_queue.empty():
                    data = data_queue.get()
                    last_sample += data

                # Use AudioData to convert the raw data to wav data.
                audio_data = sr.AudioData(last_sample, source.SAMPLE_RATE, source.SAMPLE_WIDTH)
                wav_data = io.BytesIO(audio_data.get_wav_data())

                # Write wav data to the temporary file as bytes.
                with open(temp_file, 'w+b') as f:
                    f.write(wav_data.read())

                with open(temp_file, 'rb') as f:
                    audio_file = f.read()

                # Clear last sample to start a new recording.
                last_sample = bytes()

                # Read the transcription.
                # result = audio_model.transcribe(temp_file, fp16=torch.cuda.is_available())
                # text = result['text'].strip()

                data = {
                    "language": "English",  # 替换为你想要的语言选项
                    "model_size": "large-v2",  # 替换为你想要的模型大小选项
                    "english": False,
                    "translate": False,
                    "audio_data": base64.b64encode(audio_file).decode("utf-8"),

                }

                response = requests.post(url, data=json.dumps(data), headers=headers)

                
                if response.status_code == 200:
                    # 请求成功，处理转录结果
                    result1 = response.json()
                else:
                    # 请求失败，输出错误信息
                    print(f"Request failed with status code: {response.status_code}")
                    print(response.text)
                texts = result1['text']
                # latest_text = result1['latest_text']
                # delete_text = result1['delete_text']
                isValid = result1['valid']
                isSameSentence = result1['same_sentence']
                translate_texts = result1['translate_text']

                if isValid:
                    int_msg = Int32()
                    for key in forward_keyword:
                        if key in ' '.join(texts):
                            print('前进')
                            int_msg.data = 1
                            info_pub.publish(int_msg)
                            isRecog = False
                            clear_cache()
                            break
                    for key in backward_keyword:
                        if key in ' '.join(texts):
                            print('后退')
                            int_msg.data = 2
                            info_pub.publish(int_msg)
                            isRecog = False
                            clear_cache()
                            break
                    for key in left_keyword:
                        if key in ' '.join(texts):
                            print('左转')
                            int_msg.data = 3
                            info_pub.publish(int_msg)
                            isRecog = False
                            clear_cache()
                            break
                    for key in right_keyword:
                        if key in ' '.join(texts):
                            print('右转')
                            int_msg.data = 4
                            info_pub.publish(int_msg)
                            isRecog = False
                            clear_cache()
                            break
                    for key in stop_keyword:
                        if key in ' '.join(texts):
                            print('停止')
                            play_audio("/home/haoran/audio/mama.wav")
                            int_msg.data = 0
                            info_pub.publish(int_msg)
                            isRecog = False
                            clear_cache()
                            break
                    for key in specific_keyword:
                        if key in ' '.join(texts):
                            print('导航')
                            int_msg.data = 5
                            info_pub.publish(int_msg)
                            play_audio("/home/haoran/audio/nav.wav")
                            script_path = "/home/haoran/catkin_ws/src/RFID_Based_Robot_Navigation/scripts/run_navigation.sh"
                            try:
                                output = subprocess.check_output(["bash", script_path], text=True, stderr=subprocess.STDOUT)
                                print(output)
                            except subprocess.CalledProcessError as e:
                                print("Error:", e.output)
                            isRecog = False
                            clear_cache()
                            break
                    for key in mama_keyword:
                        if key in ' '.join(texts):
                            play_audio("/home/haoran/audio/mama.wav")
                            isRecog = False
                            clear_cache()
                            break
                    # now_time = time.time()
                    # if now_time - revive_time > 10:
                    #     isRecog = False


                if isValid:
                    for key in revive_keyword:
                        if key in ' '.join(texts):
                            # 随机播放欢迎语音
                            play_audio(welcome_audio[int(time.time()) % 2])
                            clear_cache()
                            revive_time = time.time()
                            isRecog = True

                sleep(0.25)
        except KeyboardInterrupt:
            break

    print("\n\nTranscription:")
    for line in transcription:
        print(line)


if __name__ == "__main__":
    if not os.path.exists('temp'):
        os.mkdir('temp')
    
    try:
        main()
    except rospy.ROSInterruptException:
        pass
        