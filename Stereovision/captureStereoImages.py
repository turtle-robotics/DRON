from ast import Try
from PyQt5.QtWidgets import QLabel, QHBoxLayout, QVBoxLayout, QApplication, QWidget
from picamera2 import Picamera2
from PyQt5.QtGui import QImage,QPixmap
from PyQt5.QtCore import QThread
import RPi.GPIO as gp
import time
import os

width = 1920
height = 1080

adapter_info = {
    "A" : {   
        "i2c_cmd":"i2cset -y 10 0x70 0x00 0x04",
        "gpio_sta":[0,0,1],
    }, "B" : {
        "i2c_cmd":"i2cset -y 10 0x70 0x00 0x05",
        "gpio_sta":[1,0,1],
    }
}

class WorkThread(QThread):

    def __init__(self):
        super(WorkThread,self).__init__()
        gp.setwarnings(False)
        gp.setmode(gp.BOARD)
        gp.setup(7, gp.OUT)
        gp.setup(11, gp.OUT)
        gp.setup(12, gp.OUT)


    def select_channel(self,index):
        channel_info = adapter_info.get(index)
        if channel_info == None:
            print("Can't get this info")
        gpio_sta = channel_info["gpio_sta"] # gpio write
        gp.output(7, gpio_sta[0])
        gp.output(11, gpio_sta[1])
        gp.output(12, gpio_sta[2])

    def init_i2c(self,index):
        channel_info = adapter_info.get(index)
        os.system(channel_info["i2c_cmd"]) # i2c write

    def run(self):
        global picam2
        # picam2 = Picamera2()
        # picam2.configure( picam2.still_configuration(main={"size": (320, 240),"format": "BGR888"},buffer_count=1))

        flag = False

        for item in {"A","B"}:
            try:
                self.select_channel(item)
                self.init_i2c(item)
                time.sleep(0.5) 
                if flag == False:
                    flag = True
                else :
                    picam2.close()
                    # time.sleep(0.5) 
                print("init1 "+ item)
                picam2 = Picamera2()
                picam2.configure(picam2.create_still_configuration(main={"size": (320, 240),"format": "BGR888"},buffer_count=2)) 
                picam2.start()
                time.sleep(2)
                picam2.capture_array(wait=False)
                time.sleep(0.1)
            except Exception as e:
                print("except: "+str(e))

        while True:
            for item in {"A","B"}:
                self.select_channel(item)
                time.sleep(0.02)
                try:
                    buf = picam2.capture_array()
                    buf = picam2.capture_array()
                    cvimg = QImage(buf, width, height,QImage.Format_RGB888)
                    cvimg.save(f"stereo_{item}.jpg")
                    pixmap = QPixmap(cvimg)
                    if item == 'A':
                        image_label.setPixmap(pixmap)
                    elif item == 'B':
                        image_label2.setPixmap(pixmap)
                except Exception as e:
                    print("capture_buffer: "+ str(e))
            break

app = QApplication([])
window = QWidget()
layout_h = QHBoxLayout()
layout_v = QVBoxLayout()
image_label = QLabel()
image_label2 = QLabel()

# picam2 = Picamera2()

work = WorkThread()

if __name__ == "__main__":
    image_label.setFixedSize(1920, 1080)
    image_label2.setFixedSize(1920, 1080)
    window.setWindowTitle("Qt Picamera2 Arducam Multi Camera Demo")
    layout_h.addWidget(image_label)
    layout_h.addWidget(image_label2)
    layout_v.addLayout(layout_h,20)
    window.setLayout(layout_v)
    window.resize(1920*2, 1080*2)

    work.start()

    window.show()
    app.exec()
    work.quit()
    picam2.close()
