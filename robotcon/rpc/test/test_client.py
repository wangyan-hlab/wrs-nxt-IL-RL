import grpc
import robotcon.rpc.test.testdef_pb2 as tgp
import robotcon.rpc.test.testdef_pb2_grpc as tgrpc

def run():
  channel = grpc.insecure_channel('localhost:10084')
  stub = tgrpc.HelloServiceStub(channel)
  response = stub.SayHello(tgp.HelloRequest(greeting='you'))
  print("Greeter client received: " + response.reply)

if __name__=="__main__":
    run()