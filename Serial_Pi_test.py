#sudo open "/Applications/Visual Studio Code.app/Contents/MacOS/Electron"
import cv2,imutils, pygame.camera, pygame.image, sys,numpy,glob, time,serial, pygame
from MotorLibrary import *

def getCamFrame(color,camera):
    retval,frame=camera.read()
    frame = imutils.resize(frame, width=700)
    frame=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
    if not color:
        frame=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        frame=cv2.cvtColor(frame,cv2.COLOR_GRAY2RGB)
    frame=numpy.rot90(frame)
    frame=pygame.surfarray.make_surface(frame)
    return frame


ser = StartSerialPort()
ser.baudrate = 9600
ser.flush()

RightMotor = Motor(ser,"M1")
LeftMotor = Motor(ser,"M2")
time.sleep(2)


# cam = cv2.VideoCapture()


RightEncoderVal = 0
LeftEncoderVal = 0
PrevRightEncoderVal = 0
PrevLeftEncoderVal = 0


# _, frame = cam.read()
# frame = imutils.resize(frame, width=700)
# # HEIGHT, WIDTH, _ = frame.shape

HEIGHT, WIDTH, _  = (393, 700, 3)
screen = pygame.display.set_mode( ( WIDTH, HEIGHT ) )


while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

    keys_pressed = pygame.key.get_pressed()

    if keys_pressed[pygame.K_LEFT]:
        RightEncoderVal+=1
        LeftEncoderVal-=1 

    if keys_pressed[pygame.K_RIGHT]:
        RightEncoderVal-=1 
        LeftEncoderVal+=1 

    if keys_pressed[pygame.K_UP]:
        RightEncoderVal+=1 
        LeftEncoderVal+=1 

    if keys_pressed[pygame.K_DOWN]:
        RightEncoderVal-=1
        LeftEncoderVal-=1
    

    time.sleep(0.005)
    #frame = getCamFrame(True,cam)
    # screen.blit(frame, (0,0))
    pygame.display.flip()

    if abs(PrevRightEncoderVal-RightEncoderVal)>15:
        PrevRightEncoderVal = RightEncoderVal
        RightMotor.setTargetPos(RightEncoderVal)
        print(RightEncoderVal)

    if abs(PrevLeftEncoderVal-LeftEncoderVal)>15:
        PrevLeftEncoderVal = LeftEncoderVal
        LeftMotor.setTargetPos(LeftEncoderVal)


cam.release()
ser.close() 

