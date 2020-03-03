# this file is deprecated!

from urx import Robot
import math
import math3d as m3d

class URRobot(Robot):
    """
    inheritance of urx library to add move radius

    author: weiwei
    date:
    """

    def __init__(self, host, use_rt = True):
        super(self.__class__, self).__init__(host, use_rt)

    # extension of urx.robot
    def movejointsradius(self, joints, acc=0.1, vel=0.05, radius = 0.01, wait=True, relative=False, threshold=None):
        """
        move in joint space, radius is set
        # this function is deprecated as joint-space smoothing leads to unexpected motion

        author: weiwei
        date: 20180219
        """
        if relative:
            l = self.getj()
            joints = [v + l[i] for i, v in enumerate(joints)]
        prog = self._format_move("movej", joints, acc, vel, radius = radius)
        print(prog)
        self.send_program(prog)
        if wait:
            self._wait_for_move(joints[:6], threshold=threshold, joints=True)
            return self.getj()

    def movetcpradius(self, poselist, acc=0.1, vel=0.05, radius = 0.01, wait=True, relative=False, threshold=None):
        pass

    # extension of urx.robot
    def getjoints(self):
        """
        get joints in degree

        author: weiwei
        date: 20180219
        """

        return [math.degrees(i) for i in self.getj()]

    def getactualtcppose(self, wait=False, _log=True):
        """
        get current 6d pose from base to to tcp
        """

        pose = URRobot.getl(self, wait, _log)
        trans = self.csys.inverse * m3d.Transform(pose)
        if _log:
            self.logger.debug("Returning pose to user: %s", trans.pose_vector)
        return trans.pose_vector.tolist()

if __name__ == '__main__':
    host = '10.2.0.50'
    urrob = URRobot(host)
    print(urrob.getjointangles())