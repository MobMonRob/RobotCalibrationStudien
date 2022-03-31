from pypylon import pylon

class Camera():

    def __init__(self):
        self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())

        self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly) 

        self.converter = pylon.ImageFormatConverter()
        self.converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

    def __del__(self):
        self.camera.StopGrabbing()

    def GetImage(self):
        grabResult = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

        while not grabResult.GrabSucceeded():
            grabResult = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
        
        image = self.converter.Convert(grabResult)
        img = image.GetArray()

        grabResult.Release() 
        return img