
from io import BufferedWriter
import subprocess
import os

videoFile: BufferedWriter = None

def openVideoFile():
    global videoFile
    if videoFile is None:
        subprocess.run("mount", "/dev/mmcblk0p2", "/mnt/videos")
        videoFile = open("/mnt/videos/" + str(getNextNumber) + ".h265", "wb")

def closeVideoFile():
    global videoFile
    if videoFile is not None:
        videoFile.close()
        videoFile = None
        subprocess.run("umount", "/dev/mmcblk0p2")

def saveFrameToFile(frame):
    frame.toFile(videoFile)

def getNextNumber() -> int:
    files = os.listdir("/mnt/videos")
    return int(files[-1][:-4]) + 1
