from typing import Tuple
import depthai as dai

def configurePipeline():
    pipeline = dai.Pipeline()

    camRGB = pipeline.createColorCamera()
    hdEncoder = pipeline.createVideoEncoder()
    hdOutput = pipeline.createXLinkOut()
    ldOutput = pipeline.createXLinkOut()

    hdOutput.setStreamName("hd_output")
    ldOutput.setStreamName("ld_output")

    camRGB.setBoardSocket(dai.CameraBoardSocket.RGB)
    camRGB.setResolution(dai.ColorCameraProperties.resolution.THE_4_K)

    camRGB.setPreviewSize(300, 300)
    camRGB.setInterleaved(False)
    camRGB.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

    hdEncoder.setDefaultProfilePreset(30, dai.VideoEncoderProperties.Profile.H265_MAIN)

    camRGB.video.link(hdEncoder.input)
    camRGB.preview.link(ldOutput.input)
    hdEncoder.bitstream.link(hdOutput.input)

    return pipeline

def configureQueues(device: dai.Device) -> Tuple[dai.DataOutputQueue, dai.DataOutputQueue]:
    hd = device.getOutputQueue(name="hd_output", maxSize=8, blocking=False)
    ld = device.getOutputQueue(name="ld_output", maxSize=8, blocking=False)

    return (hd, ld)

def readQueues(hd: dai.DataOutputQueue, ld: dai.DataOutputQueue) -> Tuple[any, any]:
    return (hd.get().getData(), ld.get().getCvFrame())
