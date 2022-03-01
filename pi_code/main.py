#!/usr/bin/env python3

import depthai as dai
import depthaiprocessing as daiprocessing
import networktablesprocessing as ntprocessing
import fsprocessing

if __name__ == "__main__":
    pipeline = daiprocessing.configurePipeline()
    with dai.Device(pipeline) as device:
        hd, ld = daiprocessing.configureQueues(device)
        while True:
            h256data, bgrData = daiprocessing.readQueues(hd, ld)
            ntprocessing.pushCameraServer(bgrData)
            if (ntprocessing.getRobotEnabled()):
                fsprocessing.saveFrameToFile(h256data)
            else:
                fsprocessing.closeVideoFile()
