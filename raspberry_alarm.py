import RPi.GPIO as GPIO
import spidev
import time
from datetime import datetime, timedelta
import board
import busio
import digitalio
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306
import subprocess
import cv2
import time 
from picamera2 import Picamera2, Preview

is_alarm_on =False
is_sleeping_picture =False

def detect_faces(image_path):
    global is_sleeping_picture
    
    # 이미지 파일 읽기
    image = cv2.imread(image_path)

    # 이미지를 그레이스케일로 변환
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # 얼굴 감지를 위한 분류기 로드
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

    # 얼굴 감지 수행
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    # 감지된 얼굴 수 출력
    print(f"Detected {len(faces)} faces in the image.")
   
    if len(faces) >= 1:
        is_sleeping_picture = True
    else:
        is_sleeping_picture = False
        
    # 각 얼굴 주변에 사각형 그리기
    for (x, y, w, h) in faces:
        cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)

    # 결과 이미지 화면에 출력
    #cv2.imshow('Face Detection', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def take_picture():
    picam = Picamera2()
    config = picam.create_preview_configuration()
    picam.start()
    time.sleep(1) #delay 
    picam.capture_file("sleep_check.jpg")
    picam.close()
    # 얼굴 감지
    image_path = 'sleep_check.jpg'
    detect_faces(image_path)




delay = 1
is_menu_displaying = True


sensor = 4 #센서 입력핀 번호 설정
GPIO.setup(sensor, GPIO.IN)

def check_motion():
    
    if GPIO.input(sensor) == 0: #센서가 Low(0)출력 , 평소 움직임이 감지되지 않을 때
        return False
            
            
    if GPIO.input(sensor) == 1: #센서가 High(1)출력 , 움직임이 감지
        return True
		

# Define the Reset Pin
oled_reset = digitalio.DigitalInOut(board.D4)
# Display Parameters
WIDTH = 128
HEIGHT = 64
BORDER = 5
# Display Refresh
LOOPTIME = 1.0
# Use for I2C.
i2c = board.I2C()
oled = adafruit_ssd1306.SSD1306_I2C(WIDTH, HEIGHT, i2c, addr=0x3C, reset=oled_reset)
# Clear display.
oled.fill(0)
oled.show()
# Create blank image for drawing.
# Make sure to create image with mode '1' for 1-bit color.
image = Image.new("1", (oled.width, oled.height))
# Get drawing object to draw on image.
draw = ImageDraw.Draw(image)

# Draw a white background
draw.rectangle((0, 0, oled.width, oled.height), outline=255, fill=255)
font = ImageFont.load_default()

GPIO.setwarnings(False)  # 불필요한 GPIO 경고 제거
GPIO.setmode(GPIO.BCM)  # GPIO 핀의 번호 모드를 BCM으로 설정

# 사용할 버튼 핀 설정
button_pins = [21,26, 19, 13, 6, 5, 0, 22, 27, 17] #버튼 0,1,2,3,4,5,6,7,8,9의 GPIO

button_0 =21
button_1 =26
button_2 =19
button_3 =13
button_4 =6
button_5 =5
button_6 =0
button_7 =22
button_8 =27
button_9 =17

# 각 버튼 핀을 입력으로 설정하고 풀다운 저항 사용
for pin in button_pins:
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def wait_for_button_press(button_name):
    pin = PIN_MAPPING.get(button_name)  # 버튼 이름에 해당하는 핀 번호 가져오기
    
    if pin is None:
        print(f"Error: Invalid button name '{button_name}'")
        return
    
    while GPIO.input(pin) == GPIO.LOW:
        pass  # 입력이 발생할 때까지 대
        
      
        
# 부저 IO
GPIO.setup(18, GPIO.OUT)
p = GPIO.PWM(18, 10)

# 알람 시간을 설정합니다 (오전 4시 19분)
alarm_hour =0
alarm_minute  =0
alarm_second  =0

def time_until_alarm():
    now = datetime.now()
    alarm_time = now.replace(hour=alarm_hour, minute=alarm_minute, second=0, microsecond=0)
    
    # 만약 현재 시간이 알람 시간보다 늦다면, 다음 날 알람 시간을 계산합니다
    if now > alarm_time:
        alarm_time += timedelta(days=1)
    
    return alarm_time - now

def play_wake_up_song():
    song_melody = [
        329, 329, 329, 261, 329, 392, 329, 392, 440, 392, 329, 261, 294, 329, 
        329, 329, 329, 261, 329, 392, 329, 392, 440, 392, 329, 261, 294, 329
    ]
    song_durations = [
        0.2, 0.2, 0.2, 0.6, 0.2, 0.6, 0.2, 0.2, 0.2, 0.6, 0.2, 0.2, 0.2, 0.6, 
        0.2, 0.2, 0.2, 0.6, 0.2, 0.6, 0.2, 0.2, 0.2, 0.6, 0.2, 0.2, 0.2, 0.6
    ]

    for note, duration in zip(song_melody, song_durations):
        p.ChangeFrequency(note)
        p.start(50)  # 50% duty cycle
        p.ChangeDutyCycle(10)  # PWM 듀티 사이클을 10%로 더 낮춰서 볼륨을 줄임
        time.sleep(duration)
    p.stop()
    
def check_light():
    ldr_value = readadc(ldr_channel)
    
    if ldr_value <=30:
        is_light_on = False
        return False
    else:
        is_light_on = True
        return True
        
def alarm_main_display():
    global is_alarm_on
    # 현재 시간을 가져옵니다
    now = datetime.now()
    current_hour = now.hour
    current_minute = now.minute
    current_second = now.second
    
    # 남은 시간을 계산합니다
    time_remaining = time_until_alarm()
    hours_remaining, remainder = divmod(time_remaining.seconds, 3600)
    minutes_remaining = remainder // 60
    
    # 현재 시간 출력
    print(f"기상까지 {hours_remaining}시간{minutes_remaining}분 남았습니다")
    
    # 현재 시간이 알람 시간과 일치하는지 확인합니다
    if current_hour == alarm_hour and current_minute == alarm_minute and current_second == alarm_second:
        print("알람시간입니다!")
        play_wake_up_song()
        is_alarm_on = True
        
    #알람이 켜져있는 경우, 알람은 1시간 뒤에 꺼짐
    if is_alarm_on and current_hour % 24 +1 == alarm_hour % 24 and current_minute == alarm_minute:
        is_alarm_on =False
                        
            
    
    # Draw a black filled box to clear the image.
    draw.rectangle((0, 0, oled.width, oled.height), outline=0, fill=0)
    # Pi Stats Display
    draw.text((-5, 0), f" TIME: {current_hour}:{current_minute}:{current_second}", font=font, fill=255)
    draw.text((-5, 16), f" Alarm: {alarm_hour}:{alarm_minute}:{alarm_second}", font=font, fill=255)
    draw.text((0, 32), "-------------------------", font=font, fill=255)
    draw.text((-5, 48), " 0.MENU       1.RESET", font=font, fill=255)
    
    
    # Display image
    repaint()
    
def alarm_menu_display():
    global alarm_hour ,alarm_minute ,alarm_second

    # Draw a black filled box to clear the image.
    draw.rectangle((0, 0, oled.width, oled.height), outline=0, fill=0)
    draw.text((-5, 0), " Set your alarm time!!", font=font, fill=255)
    draw.text((0, 16), "Hour:", font=font, fill=255)
    repaint()
    time.sleep(0.5)
    pressed_button_h_1 = None
    while pressed_button_h_1 is None:  
        pressed_button_h_1 = wait_for_any_button_press() 
    time.sleep(0.5)
    pressed_button_h_2 = None
    while pressed_button_h_2 is None:  
        pressed_button_h_2 = wait_for_any_button_press() 
    
    pressed_button_h =int(str(pressed_button_h_1)+str(pressed_button_h_2))
    draw.text((0, 16), f"Hour:{pressed_button_h}", font=font, fill=255)
    draw.text((0, 32), f"Minute:", font=font, fill=255)
    repaint()
    time.sleep(0.5)
    
    pressed_button_m_1 = None
    pressed_button_m_2 = None
    while pressed_button_m_1 is None:
        pressed_button_m_1 = wait_for_any_button_press()
    time.sleep(0.5)
    while pressed_button_m_2 is None:  
        pressed_button_m_2 = wait_for_any_button_press() 
        
    # Convert the single-digit button press into a two-digit number
    pressed_button_m = int(str(pressed_button_m_1) + str(pressed_button_m_2))
    draw.text((0, 32), f"Minute:{pressed_button_m}", font=font, fill=255)
    draw.text((0, 48), f"Second:", font=font, fill=255)
    repaint()
    time.sleep(0.5)
    
    
    pressed_button_s_1 = None
    pressed_button_s_2 = None
    while pressed_button_s_1 is None:
        pressed_button_s_1 = wait_for_any_button_press()
    time.sleep(0.5)
    while pressed_button_s_2 is None:  
        pressed_button_s_2 = wait_for_any_button_press() 
        
        # Convert the single-digit button press into a two-digit number
    pressed_button_s = int(str(pressed_button_s_1) + str(pressed_button_s_2))
    draw.text((0, 48), f"Second:{pressed_button_s}", font=font, fill=255)
    repaint()
    time.sleep(0.5)
    
    # Update the alarm time
    alarm_hour = pressed_button_h
    alarm_minute = pressed_button_m
    alarm_second = pressed_button_s        
    
def repaint():
    # Display image
    oled.image(image)
    oled.show()
    

    
def wait_for_any_button_press():
    for index, pin in enumerate(button_pins):
        if GPIO.input(pin) == GPIO.HIGH:
            print(index)
            return index

# 딜레이 시간 (센서 측정 간격)
ldr_channel = 0  # MCP3008 채널 중 센서에 연결한 채널 설정 (CH0)
spi = spidev.SpiDev()  # SPI 인스턴스 spi 생성
spi.open(0, 0)  # SPI 통신 시작하기
spi.max_speed_hz = 100000  # SPI 통신 속도 설정

def readadc(adcnum):  # 0 ~ 7까지 8개의 채널에서 SPI 데이터를 읽어서 반환
    if adcnum > 7 or adcnum < 0:
        return -1
    r = spi.xfer2([1, 8 + adcnum << 4, 0])
    data = ((r[1] & 3) << 8) + r[2]
    return data

try:
    while True:  # readadc 함수로 ldr_channel의 SPI 데이터를 읽어 저장
        
        is_light_on = check_light()
        is_motion_on = check_motion()
        print("------------------------")
        if GPIO.input(button_0) == GPIO.HIGH:  # GPIO.HIGH → 1
                alarm_menu_display()
                
        
        if is_light_on:
            print("light on")
        else:
            print("light off")
        if is_motion_on:
            print("motion on")
        else:
            print("motion off")
        if is_alarm_on:
            print("alarm on")
        else:
            print("alarm off")
        
        if is_light_on == False and is_alarm_on == True: #알람이 켜졌는데 불이 꺼져있다면 알람 반복
            play_wake_up_song()
        
        
        if is_light_on and is_motion_on and is_alarm_on: #알람 키고 일어난 것을 모션 감지 후 사진 10번
            for i in range (10):
                print("take a picture")
                take_picture()
                if is_sleeping_picture:
                    for i in range (10):
                        play_wake_up_song()
                
            
        
            
        
        alarm_main_display()
        time.sleep(delay)
        
    
except KeyboardInterrupt:  # 키보드 Ctrl+C 눌렀을때 예외발생
    pass  # 무한반복을 빠져나와 아래의 코드를 실행
finally:
    p.stop()  # PWM을 종료
    GPIO.cleanup()  # GPIO 설정을 초기화
