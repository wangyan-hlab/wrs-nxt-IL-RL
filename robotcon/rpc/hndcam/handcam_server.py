import grpc
import time
import yaml
import robotcon.rpc.hndcam.cameras as cameras
from concurrent import futures
import robotcon.rpc.hndcam.hndcam_pb2 as hcmsg
import robotcon.rpc.hndcam.hndcam_pb2_grpc as hcrpc

class HndCamServer(hcrpc.CamServicer):
    hc = cameras.HndCam()

    def getrc0img(self, request, context):
        """

        Inherited from the auto-generated handcam_pb2_grpc

        :param request:
        :param context:
        :return:

        author: weiwei
        date: 20190416
        """

        return hcmsg.CamImg(data=yaml.dump(self.hc.getrc0img()))

    def getrc1img(self, request, context):
        """

        Inherited from the auto-generated handcam_pb2_grpc

        :param request:
        :param context:
        :return:

        author: weiwei
        date: 20190416
        """

        return hcmsg.CamImg(data=yaml.dump(self.hc.getrc1img()))

    def getlc0img(self, request, context):
        """

        Inherited from the auto-generated handcam_pb2_grpc

        :param request:
        :param context:
        :return:

        author: weiwei
        date: 20190416
        """

        return hcmsg.CamImg(data=yaml.dump(self.hc.getlc0img()))

    def getlc1img(self, request, context):
        """

        Inherited from the auto-generated handcam_pb2_grpc

        :param request:
        :param context:
        :return:

        author: weiwei
        date: 20190416
        """

        return hcmsg.CamImg(data=yaml.dump(self.hc.getlc1img()))

def serve():
    _ONE_DAY_IN_SECONDS = 60 * 60 * 24

    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    hcrpc.add_CamServicer_to_server(HndCamServer(), server)
    server.add_insecure_port('[::]:18300')
    server.start()
    print("The HndCam server is started!")
    try:
        while True:
            time.sleep(_ONE_DAY_IN_SECONDS)
    except KeyboardInterrupt:
        server.stop(0)

if __name__ == '__main__':
    serve()