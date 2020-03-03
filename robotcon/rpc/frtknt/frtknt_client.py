import grpc
import yaml
import robotcon.rpc.frtknt.frtknt_pb2 as fkmsg
import robotcon.rpc.frtknt.frtknt_pb2_grpc as fkrpc

class FrtKnt(object):

    def __init__(self, host = "localhost:18300"):
        channel = grpc.insecure_channel(host)
        self.stub = fkrpc.KntStub(channel)
        self.__oldyaml = True
        if int(yaml.__version__[0]) >= 5:
            self.__oldyaml = False

    def getrgbarray(self):
        """
        get color image as an array

        :return: a colorHeight*colorWidth*4 np array, the second and third channels are repeated
        author: weiwei
        date: 20180207
        """

        if self.__oldyaml:
            return yaml.load(self.stub.getrgbarray(fkmsg.Empty()).data)
        else:
            return yaml.load(self.stub.getrgbarray(fkmsg.Empty()).data, Loader=yaml.UnsafeLoader)

    def getdeptharray(self):
        """
        get depth image as an array

        :return: a depthHeight*depthWidth*3 np array, the second and third channels are repeated
        author: weiwei
        date: 20180207
        """

        if self.__oldyaml:
            return yaml.load(self.stub.getdeptharray(fkmsg.Empty()).data)
        else:
            return yaml.load(self.stub.getdeptharray(fkmsg.Empty()).data, Loader=yaml.UnsafeLoader)

    def getdepthraw(self):
        """
        get raw ddsaasaepth image

        :return: a raw depth image
        author: weiwei
        date: 20181121
        """

        if self.__oldyaml:
            return yaml.load(self.stub.getdepthraw(fkmsg.Empty()).data)
        else:
            return yaml.load(self.stub.getdepthraw(fkmsg.Empty()).data, Loader=yaml.UnsafeLoader)

    def getpcdarray(self, mat_kw = None):
        """
        get the full poind cloud of a new frame as an array

        :param mat_kw 4x4 nparray
        :return: np.array point cloud n-by-3
        author: weiwei
        date: 20181121
        """

        matkwmsg = fkmsg.MatKW(data = yaml.dump(mat_kw))
        if self.__oldyaml:
            return yaml.load(self.stub.getpcdarray(matkwmsg).data)
        else:
            return yaml.load(self.stub.getpcdarray(matkwmsg).data, Loader=yaml.UnsafeLoader)

    def getpartialpcdarray(self, dframe, width, height, mat_kw=None):
        """
        get partial poind cloud using the given picklerawdframe, width, height in a depth img

        :param dframe return value of getdepthraw
        :param width, height
        :param picklemat_tw pickle string storing mat_tw

        author: weiwei
        date: 20181121
        """

        matkwmsg = fkmsg.MatKW(data = yaml.dump(mat_kw))
        widthpairmsg = fkmsg.Pair(data0 = width[0], data1 = width[1])
        heightpairmsg = fkmsg.Pair(data0 = height[0], data1 = height[1])
        dframemsg = fkmsg.PartialPcdPara(data = yaml.dump(dframe), width = widthpairmsg,
                                         height = heightpairmsg, matkw = matkwmsg)
        if self.__oldyaml:
            return yaml.load(self.stub.getpartialpcdarray(dframemsg).data)
        else:
            return yaml.load(self.stub.getpartialpcdarray(dframemsg).data, Loader=yaml.UnsafeLoader)

    def mapColorPointToCameraSpace(self, pt):
        """
        convert color space  , to depth space point

        :param pt: [p0, p1] or nparray([p0, p1])
        :return:
        author: weiwei
        date: 20181121
        """

        ptpairmsg = fkmsg.Pair(data0 = pt[0], data1 = pt[1])
        if self.__oldyaml:
            return yaml.load(self.stub.mapColorPointToCameraSpace(ptpairmsg).data)
        else:
            return yaml.load(self.stub.mapColorPointToCameraSpace(ptpairmsg).data, Loader=yaml.UnsafeLoader)

if __name__ == "__main__":
    frk = FrtKnt(host = "localhost:18300")
    pcd = frk.getpcdarray()