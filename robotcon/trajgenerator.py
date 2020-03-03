import math
import numpy as np

def discretize(jointslist, step=1.0):
    newjointslist = []
    for i in range(0, len(jointslist)):
        if i> 70 and i < len(jointslist)-1:
            joints0 = np.array(jointslist[i])
            joints1 = np.array(jointslist[i+1])
            diff = joints1-joints0
            maxdiff = np.absolute(diff).max()
            nstep = int(math.ceil(maxdiff/step))
            if maxdiff<step:
                print("too small!")
                # assert(False)
            if maxdiff == 0:
                continue
            steplength = diff/nstep
            print("steplength")
            print(steplength)
            print("newjoints")
            for j in range(0, nstep):
                newjoints = joints0+j*steplength
                print(newjoints)
                newjointslist.append(newjoints.tolist())
            newjointslist.append(joints1.tolist())
            print(joints1.tolist())
    return newjointslist

def genmotiontrajectory(jointslist, vel=1.0, inpfunc="cubic"):
    """
    move robot continuously using interpolation

    :param jointslist:
    :param armid:
    :param vel: time to move between adjacent joints, = expandis/speed, speed = degree/second
    :param inpfunc: call cubic by default, candidate values: cubic or quintic
    :return:

    author: weiwei
    date: 20180518
    """

    def cubic(t, timestamp, q0array, v0array, q1array, v1array):
        a0 = q0array
        a1 = v0array
        a2 = (-3 * (q0array - q1array) - (2 * v0array + v1array) * timestamp) / (timestamp ** 2)
        a3 = (2 * (q0array - q1array) + (v0array + v1array) * timestamp) / (timestamp ** 3)
        qt = a0 + a1 * t + a2 * (t ** 2) + a3 * (t ** 3)
        vt = a1 + 2 * a2 * t + 3 * a3 * (t ** 2)
        return qt.tolist(), vt.tolist()

    def quintic(t, timestamp, q0array, v0array,
                q1array, v1array, a0array=None, a1array=None):
        if a0array is None:
            a0array = np.array([0.0] * q0array.shape[0])
        if a1array is None:
            a1array = np.array([0.0] * q0array.shape[0])
        a0 = q0array
        a1 = v0array
        a2 = a0array / 2.0
        a3 = (20 * (q1array - q0array) - (8 * v1array + 12 * v0array) * timestamp -
              (3 * a1array - a0array) * (timestamp ** 2)) / (2 * (timestamp ** 3))
        a4 = (30 * (q0array - q1array) + (14 * v1array + 16 * v0array) * timestamp +
              (3 * a1array - 2 * a0array) * (timestamp ** 2)) / (2 * (timestamp ** 4))
        a5 = (12 * (q1array - q0array) - 6 * (v1array + v0array) * timestamp -
              (a1array - a0array) * (timestamp ** 2)) / (2 * (timestamp ** 5))
        qt = a0 + a1 * t + a2 * (t ** 2) + a3 * (t ** 3) + a4 * (t ** 4) + a5 * (t ** 5)
        vt = a1 + 2 * a2 * t + 3 * a3 * (t ** 2) + 4 * a4 * (t ** 3) + 5 * a5 * (t ** 4)
        return qt.tolist(), vt.tolist()

    if inpfunc != "cubic" and inpfunc != "quintic":
        raise ValueError("Interpolation functions must be cubic or quintic")
    inpfunccallback = cubic
    if inpfunc == "quintic":
        inpfunccallback = quintic

    timestep = 0.002
    timepathstep = vel
    timesstamplist = []
    speedsradlist = []
    jointsradlist = []
    for id, joints in enumerate(jointslist):
        jointsrad = [math.radians(angdeg) for angdeg in joints]
        jointsradlist.append(jointsrad)
        if id == 0:
            timesstamplist.append([0.0] * len(jointsrad))
        else:
            timesstamplist.append([timepathstep] * len(jointsrad))
        if id == 0 or id == len(jointslist) - 1:
            speedsradlist.append([0.0] * len(jointsrad))
        else:
            thisjointsrad = jointsrad
            prejointsrad = [math.radians(angdeg) for angdeg in jointslist[id - 1]]
            nxtjointsrad = [math.radians(angdeg) for angdeg in jointslist[id + 1]]
            presarray = (np.array(thisjointsrad) - np.array(prejointsrad)) / timepathstep
            nxtsarray = (np.array(nxtjointsrad) - np.array(thisjointsrad)) / timepathstep
            # set to 0 if signs are different
            selectid = np.where((np.sign(presarray) + np.sign(nxtsarray)) == 0)
            sarray = (presarray + nxtsarray) / 2.0
            sarray[selectid] = 0.0
            speedsradlist.append(sarray.tolist())
    t = 0
    jointsradlist002 = []
    speedsradlist002 = []
    rawjointsradlist002 = []
    for idlist, timesstamp in enumerate(timesstamplist):
        if idlist == 0:
            continue
        timesstampnp = np.array(timesstamp)
        jointsradprenp = np.array(jointsradlist[idlist - 1])
        speedsradprenp = np.array(speedsradlist[idlist - 1])
        jointsradnp = np.array(jointsradlist[idlist])
        speedsradnp = np.array(speedsradlist[idlist])
        # reduce timestep in the last step to avoid overfitting
        if idlist == len(timesstamplist) - 1:
            while t <= timesstampnp.max():
                jsrad, vsrad = inpfunccallback(t, timesstampnp,
                                               jointsradprenp, speedsradprenp,
                                               jointsradnp, speedsradnp)
                jointsradlist002.append(jsrad)
                speedsradlist002.append(vsrad)
                rawjointsradlist002.append(jointsradprenp.tolist())
                t = t + timestep / 4
        else:
            while t <= timesstampnp.max():
                jsrad, vsrad = inpfunccallback(t, timesstampnp,
                                               jointsradprenp, speedsradprenp,
                                               jointsradnp, speedsradnp)
                jointsradlist002.append(jsrad)
                speedsradlist002.append(vsrad)
                rawjointsradlist002.append(jointsradprenp.tolist())
                t = t + timestep
            t = 0
    jointsanglist002 = []
    rawjointsanglist002 = []
    for jsrad in jointsradlist002:
        jointsanglist002.append([math.degrees(angrad) for angrad in jsrad])
    for jsrad in rawjointsradlist002:
        rawjointsanglist002.append([math.degrees(angrad) for angrad in jsrad])
    return jointsanglist002, rawjointsanglist002

if __name__=='__main__':

    import yaml
    import copy

    numikrmsmp =[]
    with open('outfilemotion_sumo.txt', 'r') as fp:
        numikrmsmp = yaml.load(fp)

    # KANEHIRO-SAMA: change the value here to further close fingers
    minusvalue = 25

    for i in range(len(numikrmsmp)):
        if numikrmsmp[i][21] < 119.0:
            numikrmsmp[i][21] = numikrmsmp[i][21]+25-minusvalue
        if numikrmsmp[i][22] < 119.0:
            numikrmsmp[i][22] = numikrmsmp[i][22]+25-minusvalue

    numikrmsmpupperbody = []
    interplationdist = .5
    for i in range(len(numikrmsmp)):
        numikrmsmpupperbody.append(numikrmsmp[i][:23])
        if i < len(numikrmsmp)-1:
            if numikrmsmp[i][21] > 119.0 and numikrmsmp[i+1][21] < 118.0:
                handopendist = numikrmsmp[i][21]-interplationdist
                while handopendist > numikrmsmp[i+1][21]:
                    tmpnumikr = copy.deepcopy(numikrmsmp[i][:23])
                    tmpnumikr[21] = handopendist
                    numikrmsmpupperbody.append(tmpnumikr)
                    handopendist = handopendist-interplationdist
            if numikrmsmp[i][21] < 118.0 and numikrmsmp[i+1][21] > 119.0:
                handopendist = numikrmsmp[i][21]+interplationdist
                while handopendist < numikrmsmp[i+1][21]:
                    tmpnumikr = copy.deepcopy(numikrmsmp[i][:23])
                    tmpnumikr[21] = handopendist
                    numikrmsmpupperbody.append(tmpnumikr)
                    handopendist = handopendist+interplationdist
            if numikrmsmp[i][22] > 119.0 and numikrmsmp[i+1][22] < 118.0:
                handopendist = numikrmsmp[i][22]-interplationdist
                while handopendist > numikrmsmp[i+1][22]:
                    tmpnumikr = copy.deepcopy(numikrmsmp[i][:23])
                    tmpnumikr[22] = handopendist
                    numikrmsmpupperbody.append(tmpnumikr)
                    handopendist = handopendist-interplationdist
            if numikrmsmp[i][22] < 118.0 and numikrmsmp[i+1][22] > 119.0:
                handopendist = numikrmsmp[i][22]+interplationdist
                while handopendist < numikrmsmp[i+1][22]:
                    tmpnumikr = copy.deepcopy(numikrmsmp[i][:23])
                    tmpnumikr[22] = handopendist
                    numikrmsmpupperbody.append(tmpnumikr)
                    handopendist = handopendist+interplationdist

    numikrmstraj, rawnumikrmstraj = genmotiontrajectory(numikrmsmpupperbody, vel = .2, inpfunc="cubic")

    fullnumikrmstraj = []
    for numikrms in numikrmstraj:
         fullnumikrms = numikrms+numikrmsmp[0][23:]
         fullnumikrmstraj.append(fullnumikrms)
    print(len(numikrmstraj))
    with open('outfiletraj_sumo.txt', 'wb') as fp:
         yaml.dump(fullnumikrmstraj, fp)