import logging

class ProgramBuilder(object):

    def __init__(self):
        self.logger = logging.getLogger("urx")
        self.complete_program = ""
        self.logger = False

    def loadprog(self, filename):
        # clear program
        self.complete_program = ""
        self.__file = open(filename, "r")
        partscript = self.__file.read(1024)
        while partscript:
            self.complete_program += partscript
            partscript = self.__file.read(1024)

    # def receiveftdata(self):
    #     import socket
    #     s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #     # now connect to the web server on port 80 - the normal http port
    #     s.connect(("10.2.0.50", 63351))
    #     while True:
    #         print s.recv(1024)

    def ret_program_to_run(self):
        if(self.complete_program == ""):
            self.logger.debug("the auto-built program is empty")
            return ""
        return self.complete_program

if __name__=="__main__":
    import robotcon.ur3dual as ur3urx
    # ur3u = ur3urx.Ur3DualUrx()
    pb = ProgramBuilder()
    # pb.loadprog("./examples/peginholespiral.script")
    # prog = pb.ret_program_to_run()
    # # prog = "textmsg(255)"
    # # print prog
    # ur3u.rgtarm.send_program(prog)
    print(pb.receivetfdata())