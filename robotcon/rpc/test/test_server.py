import grpc
import time
import logging
from concurrent import futures
import robotcon.rpc.test.testdef_pb2 as tgp
import robotcon.rpc.test.testdef_pb2_grpc as tgrpc

_ONE_DAY_IN_SECONDS = 60 * 60 * 24

class TServer(tgrpc.HelloServiceServicer):
    def SayHello(self, request, context):
        return tgp.HelloResponse(reply = "good" + request.greeting)

def serve():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    tgrpc.add_HelloServiceServicer_to_server(TServer(), server)
    server.add_insecure_port('[::]:10084')
    server.start()
    try:
        while True:
            time.sleep(_ONE_DAY_IN_SECONDS)
    except KeyboardInterrupt:
        server.stop(0)

if __name__ == '__main__':
    logging.basicConfig()
    serve()