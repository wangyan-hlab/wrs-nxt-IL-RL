# utility functions that add primitives to the found path by regriptppfp

import os

def getNPath(regrip, choice):
    """
    get the number of candidate paths for the choice

    :param regrip:
    :param choice:
    :return:
    """

    directshortestpaths = []
    if choice is 'startrgtgoalrgt':
        directshortestpaths = regrip.directshortestpaths_startrgtgoalrgt
    elif choice is 'startrgtgoallft':
        directshortestpaths = regrip.directshortestpaths_startrgtgoallft
    elif choice is 'startlftgoalrgt':
        directshortestpaths = regrip.directshortestpaths_startlftgoalrgt
    elif choice is 'startlftgoallft':
        directshortestpaths = regrip.directshortestpaths_startlftgoallft

    if len(directshortestpaths) == 0:
        print("no path found")
        return 0
    else:
        return len(directshortestpaths)

def getMotionSequence(regrip, id, choice, type = "OO", previous = []):
    """
        generate motion sequence using the shortest path
        right arm
        this function is for simple pick and place with regrasp

        # 20190319 comment by weiwei
        five letters are attached to nids,
        they are "x", "w", "o", "c", "i"
        where "x" indicates handa,
        "w" indicates worlda,
        "o" and "c" are at grasping psoe, they indicate the open and close states of a hand
        "i" indicates initial pose
        these letter will be use to determine planning methods in the planner.py file
        e.g. an o->c motion will be simple finger motion, no rrt planners will be called
        a x->w will be planning with hold, x->c will be interplation, x->i will be planning without hold, etc.
        see the planning.py file for details

        #20190319 comment by weiwei
        OO means start from a hand open pose and stop at a hand open pose
        OC means start from a hand open pose and stop at a hand close pose
        CC means start from a hand close pose and stop at a hand close pose
        To generate multiple motion sequences, OC->CC->CC->...->CC->CO is the preferred type order choice


        :param: regrip an object of the regriptppfp.RegripTppfp class
        :param id: which path to plot
        :param choice: startrgtgoalrgt/startrgtgoallft/startlftgoalrgt/startlftgoallft
        :param type: one of "OO', "OC", "CC", "CO"
        :param previous: set it to [] if the motion is not a continuing one, or else, set it to [lastobjmat4, lastikr, lastjawwidth]

        :return: [[waist, lftbody, rgtbody],...]

        author: weiwei
        date: 20170302
        """

    if(choice not in ["startrgtgoalrgt", "startrgtgoallft", "startlftgoalrgt", "startlftgoallft"]):
        raise Exception("The choice parameter of getMotionSequence must be "+
                         "one of startrgtgoalrt, startrgtgoalft, startlftgoalrgt, startlftgoallft! "+
                         "Right now it is %s" % choice+".")

    if(type not in ["OO", "OC", "CC", "CO"]):
        raise Exception("The choice parameter of type must be "+
                         "one of OO, OC, CC, CO! "+ "Right now it is %s" % choice+".")

    directshortestpaths = []
    if choice is 'startrgtgoalrgt':
        directshortestpaths = regrip.directshortestpaths_startrgtgoalrgt
    elif choice is 'startrgtgoallft':
        directshortestpaths = regrip.directshortestpaths_startrgtgoallft
    elif choice is 'startlftgoalrgt':
        directshortestpaths = regrip.directshortestpaths_startlftgoalrgt
    elif choice is 'startlftgoallft':
        directshortestpaths = regrip.directshortestpaths_startlftgoallft

    if len(directshortestpaths) == 0:
        print("No path found!")
        os._exit(0)

    pathnidlist = directshortestpaths[id]
    if len(previous) != 0:
        numikrlist = [previous[1]]
        objmat4list = [previous[0]]
        jawwidth = [previous[2]]
    else:
        numikrlist = [[regrip.robot.initjnts[0], regrip.robot.initrgtjnts, regrip.robot.initlftjnts]]
        objmat4list = [regrip.startrotmat4]
        jawwidth = [[regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen]]
    extendedpathnidlist = ['begin']
    print(pathnidlist)

    for i in range(len(pathnidlist) - 1):
        if i == 0 and len(pathnidlist) == 2:
            # two node path
            # they must be both rgt or both lft
            # they cannot be handover
            ## starting node
            nid = pathnidlist[i]
            gripjawwidth = regrip.regg.nodes[nid]['jawwidth']
            armjntsgrp = regrip.regg.nodes[nid]['armjnts']
            armjntsgrphanda = regrip.regg.nodes[nid]['armjntshanda']
            armjntsgrpworlda = regrip.regg.nodes[nid]['armjntsworlda']
            # choice
            if nid.startswith('startrgt'):
                if((type is "OC") or (type is "OO")):
                    numikrlist.append([armjntsgrphanda[0], armjntsgrphanda[1], regrip.robot.initlftjnts])
                    numikrlist.append([armjntsgrp[0], armjntsgrp[1], regrip.robot.initlftjnts])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                numikrlist.append([armjntsgrp[0], armjntsgrp[1], regrip.robot.initlftjnts])
                numikrlist.append([armjntsgrpworlda[0], armjntsgrpworlda[1], regrip.robot.initlftjnts])
                jawwidth.append([gripjawwidth, regrip.robothand.jawwidthopen])
                jawwidth.append([gripjawwidth, regrip.robothand.jawwidthopen])
            if nid.startswith('startlft'):
                if((type is "OC") or (type is "OO")):
                    numikrlist.append([armjntsgrphanda[0], regrip.robot.initrgtjnts, armjntsgrphanda[1]])
                    numikrlist.append([armjntsgrp[0], regrip.robot.initrgtjnts, armjntsgrp[1]])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                numikrlist.append([armjntsgrp[0], regrip.robot.initrgtjnts, armjntsgrp[1]])
                numikrlist.append([armjntsgrpworlda[0], regrip.robot.initrgtjnts, armjntsgrpworlda[1]])
                jawwidth.append([regrip.robothand.jawwidthopen, gripjawwidth])
                jawwidth.append([regrip.robothand.jawwidthopen, gripjawwidth])
            objmat4handa = regrip.regg.nodes[nid]['tabletopplacementrotmathanda']
            objmat4 = regrip.regg.nodes[nid]['tabletopplacementrotmat']
            objmat4worlda = regrip.regg.nodes[nid]['tabletopplacementrotmatworlda']
            if ((type is "OC") or (type is "OO")):
                objmat4list.append(objmat4handa)
                objmat4list.append(objmat4)
                extendedpathnidlist.append(nid+"x")
                extendedpathnidlist.append(nid+"o")
            objmat4list.append(objmat4)
            objmat4list.append(objmat4worlda)
            extendedpathnidlist.append(nid+"c")
            extendedpathnidlist.append(nid+"w")
            ## first node
            nid = pathnidlist[i + 1]
            gripjawwidth = regrip.regg.nodes[nid]['jawwidth']
            armjntsgrp = regrip.regg.nodes[nid]['armjnts']
            armjntsgrphanda = regrip.regg.nodes[nid]['armjntshanda']
            armjntsgrpworlda = regrip.regg.nodes[nid]['armjntsworlda']
            # initialize
            # choice
            if nid.startswith('goalrgt'):
                numikrlist.append([armjntsgrpworlda[0], armjntsgrpworlda[1], regrip.robot.initlftjnts])
                numikrlist.append([armjntsgrp[0], armjntsgrp[1], regrip.robot.initlftjnts])
                jawwidth.append([gripjawwidth, regrip.robothand.jawwidthopen])
                jawwidth.append([gripjawwidth, regrip.robothand.jawwidthopen])
                if ((type is "CO") or (type is "OO")):
                    numikrlist.append([armjntsgrp[0], armjntsgrp[1], regrip.robot.initlftjnts])
                    numikrlist.append([armjntsgrphanda[0], armjntsgrphanda[1], regrip.robot.initlftjnts])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
            if nid.startswith('goallft'):
                numikrlist.append([armjntsgrpworlda[0], regrip.robot.initrgtjnts, armjntsgrpworlda[1]])
                numikrlist.append([armjntsgrp[0], regrip.robot.initrgtjnts, armjntsgrp[1]])
                jawwidth.append([regrip.robothand.jawwidthopen, gripjawwidth])
                jawwidth.append([regrip.robothand.jawwidthopen, gripjawwidth])
                if ((type is "CO") or (type is "OO")):
                    numikrlist.append([armjntsgrp[0], regrip.robot.initrgtjnts, armjntsgrp[1]])
                    numikrlist.append([armjntsgrphanda[0], regrip.robot.initrgtjnts, armjntsgrphanda[1]])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
            objmat4worlda = regrip.regg.nodes[nid]['tabletopplacementrotmatworlda']
            objmat4 = regrip.regg.nodes[nid]['tabletopplacementrotmat']
            objmat4handa = regrip.regg.nodes[nid]['tabletopplacementrotmathanda']
            objmat4list.append(objmat4worlda)
            objmat4list.append(objmat4)
            extendedpathnidlist.append(nid+"w")
            extendedpathnidlist.append(nid+"c")
            if ((type is "CO") or (type is "OO")):
                objmat4list.append(objmat4)
                objmat4list.append(objmat4handa)
                extendedpathnidlist.append(nid+"o")
                extendedpathnidlist.append(nid+"x")
        elif i == 0:
            # not two nodepath, starting node, transfer
            ## starting node
            nid = pathnidlist[i]
            gripjawwidth = regrip.regg.nodes[nid]['jawwidth']
            armjntsgrp = regrip.regg.nodes[nid]['armjnts']
            armjntsgrphanda = regrip.regg.nodes[nid]['armjntshanda']
            armjntsgrpworlda = regrip.regg.nodes[nid]['armjntsworlda']
            # choice
            if nid.startswith('startrgt'):
                if((type is "OC") or (type is "OO")):
                    numikrlist.append([armjntsgrphanda[0], armjntsgrphanda[1], regrip.robot.initlftjnts])
                    numikrlist.append([armjntsgrp[0], armjntsgrp[1], regrip.robot.initlftjnts])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                numikrlist.append([armjntsgrp[0], armjntsgrp[1], regrip.robot.initlftjnts])
                numikrlist.append([armjntsgrpworlda[0], armjntsgrpworlda[1], regrip.robot.initlftjnts])
                jawwidth.append([gripjawwidth, regrip.robothand.jawwidthopen])
                jawwidth.append([gripjawwidth, regrip.robothand.jawwidthopen])
            if nid.startswith('startlft'):
                if((type is "OC") or (type is "OO")):
                    numikrlist.append([armjntsgrphanda[0], regrip.robot.initrgtjnts, armjntsgrphanda[1]])
                    numikrlist.append([armjntsgrp[0], regrip.robot.initrgtjnts, armjntsgrp[1]])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                numikrlist.append([armjntsgrp[0], regrip.robot.initrgtjnts, armjntsgrp[1]])
                numikrlist.append([armjntsgrpworlda[0], regrip.robot.initrgtjnts, armjntsgrpworlda[1]])
                jawwidth.append([regrip.robothand.jawwidthopen, gripjawwidth])
                jawwidth.append([regrip.robothand.jawwidthopen, gripjawwidth])
            objmat4handa = regrip.regg.nodes[nid]['tabletopplacementrotmathanda']
            objmat4 = regrip.regg.nodes[nid]['tabletopplacementrotmat']
            objmat4worlda = regrip.regg.nodes[nid]['tabletopplacementrotmatworlda']
            if((type is "OC") or (type is "OO")):
                objmat4list.append(objmat4handa)
                objmat4list.append(objmat4)
                extendedpathnidlist.append(nid+"x")
                extendedpathnidlist.append(nid+"o")
            objmat4list.append(objmat4)
            objmat4list.append(objmat4worlda)
            extendedpathnidlist.append(nid+"c")
            extendedpathnidlist.append(nid+"w")
            ## first node
            nid = pathnidlist[i + 1]
            if nid.startswith('ho'):
                pass
            else:
                gripjawwidth = regrip.regg.nodes[nid]['jawwidth']
                armjntsgrp = regrip.regg.nodes[nid]['armjnts']
                armjntsgrphanda = regrip.regg.nodes[nid]['armjntshanda']
                armjntsgrpworlda = regrip.regg.nodes[nid]['armjntsworlda']
                # choice
                if nid.startswith('rgt'):
                    numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrpworlda, regrip.robot.initlftjntsr[1:]])
                    numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrp, regrip.robot.initlftjntsr[1:]])
                    numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrp, regrip.robot.initlftjntsr[1:]])
                    numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrphanda, regrip.robot.initlftjntsr[1:]])
                    jawwidth.append([gripjawwidth, regrip.robothand.jawwidthopen])
                    jawwidth.append([gripjawwidth, regrip.robothand.jawwidthopen])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                if nid.startswith('lft'):
                    numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrpworlda])
                    numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrp])
                    numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrp])
                    numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrphanda])
                    jawwidth.append([regrip.robothand.jawwidthopen, gripjawwidth])
                    jawwidth.append([regrip.robothand.jawwidthopen, gripjawwidth])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                objmat4worlda = regrip.regg.nodes[nid]['tabletopplacementrotmatworlda']
                objmat4 = regrip.regg.nodes[nid]['tabletopplacementrotmat']
                objmat4handa = regrip.regg.nodes[nid]['tabletopplacementrotmathanda']
                objmat4list.append(objmat4worlda)
                objmat4list.append(objmat4)
                objmat4list.append(objmat4)
                objmat4list.append(objmat4handa)
                extendedpathnidlist.append(nid+"w")
                extendedpathnidlist.append(nid+"c")
                extendedpathnidlist.append(nid+"o")
                extendedpathnidlist.append(nid+"x")
        elif i + 1 != len(pathnidlist) - 1:
            # not two node path, middle nodes, if transit, pass
            if regrip.regg.edges[pathnidlist[i], pathnidlist[i + 1]]['edgetype'] == "transit":
                pass
            # if handovertransit
            elif regrip.regg.edges[pathnidlist[i], pathnidlist[i + 1]]['edgetype'] == "handovertransit":
                nid0 = pathnidlist[i]
                nid1 = pathnidlist[i + 1]
                #### nid0 move to handover
                grpjawwidth0 = regrip.regg.nodes[nid0]['jawwidth']
                armjntsgrp0 = regrip.regg.nodes[nid0]['armjnts']
                # initialize
                # choice
                if nid0.startswith('horgt'):
                    numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrp0, regrip.robot.initlftjntsr[1:]])
                    jawwidth.append([grpjawwidth0, regrip.robothand.jawwidthopen])
                elif nid0.startswith('holft'):
                    numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrp0])
                    jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth0])
                objmat40 = regrip.regg.nodes[nid0]['floatingposerotmat4']
                objmat4list.append(objmat40)
                extendedpathnidlist.append(nid0+"c")
                #### nid1 move to handover
                grpjawwidth1 = regrip.regg.nodes[nid1]['jawwidth']
                armjntsgrphanda1 = regrip.regg.nodes[nid1]['armjntshanda']
                # initialize
                armjntsgrp1 = regrip.regg.nodes[nid1]['armjnts']
                # choice
                if nid1.startswith('horgt'):
                    numikrlist.append([regrip.robot.initrgtjntsr[0], armjntsgrphanda1, armjntsgrp0])
                    numikrlist.append([regrip.robot.initrgtjntsr[0], armjntsgrp1, armjntsgrp0])
                    numikrlist.append([regrip.robot.initrgtjntsr[0], armjntsgrp1, armjntsgrp0])
                    jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth0])
                    jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth0])
                    jawwidth.append([grpjawwidth1, grpjawwidth0])
                elif nid1.startswith('holft'):
                    numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrp0, armjntsgrphanda1])
                    numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrp0, armjntsgrp1])
                    numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrp0, armjntsgrp1])
                    jawwidth.append([grpjawwidth0, regrip.robothand.jawwidthopen])
                    jawwidth.append([grpjawwidth0, regrip.robothand.jawwidthopen])
                    jawwidth.append([grpjawwidth0, grpjawwidth1])
                objmat41 = regrip.regg.nodes[nid0]['floatingposerotmat4']
                objmat4list.append(objmat41)
                objmat4list.append(objmat41)
                objmat4list.append(objmat41)
                extendedpathnidlist.append(nid1+"x")
                extendedpathnidlist.append(nid1+"o")
                extendedpathnidlist.append(nid1+"c")
                #### nid0 move back
                armjntsgrpb = regrip.regg.nodes[nid0]['armjntshanda']
                # choice
                if nid0.startswith('horgt'):
                    numikrlist.append([regrip.robot.initrgtjntsr[0], armjntsgrp0, armjntsgrp1])
                    numikrlist.append([regrip.robot.initrgtjntsr[0], armjntsgrpb, armjntsgrp1])
                    jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth1])
                    jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth1])
                elif nid0.startswith('holft'):
                    numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrp1, armjntsgrp0])
                    numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrp1, armjntsgrpb])
                    jawwidth.append([grpjawwidth1, regrip.robothand.jawwidthopen])
                    jawwidth.append([grpjawwidth1, regrip.robothand.jawwidthopen])
                objmat4b = regrip.regg.nodes[nid0]['floatingposerotmat4']
                objmat4list.append(objmat4b)
                objmat4list.append(objmat4b)
                extendedpathnidlist.append(nid0+"o")
                extendedpathnidlist.append(nid0+"x")
                # move back to init pose
                if nid0.startswith('horgt'):
                    numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrp1])
                    jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth1])
                elif nid0.startswith('holft'):
                    numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrp1, regrip.robot.initlftjntsr[1:]])
                    jawwidth.append([grpjawwidth1, regrip.robothand.jawwidthopen])
                objmat4list.append(objmat4b)
                extendedpathnidlist.append(nid0+"i")
            else:
                print(regrip.regg.edges[pathnidlist[i], pathnidlist[i + 1]]['edgetype'])
                # not two node path, middle nodes, if transfer
                ## middle first
                nid = pathnidlist[i]
                if nid.startswith('ho'):
                    pass
                else:
                    gripjawwidth = regrip.regg.nodes[nid]['jawwidth']
                    armjntsgrp = regrip.regg.nodes[nid]['armjnts']
                    armjntsgrphanda = regrip.regg.nodes[nid]['armjntshanda']
                    armjntsgrpworlda = regrip.regg.nodes[nid]['armjntsworlda']
                    # initialize
                    # choice
                    if nid.startswith('rgt'):
                        numikrlist.append(
                            [regrip.robot.initlftjntsr[0], armjntsgrphanda, regrip.robot.initlftjntsr[1:]])
                        numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrp, regrip.robot.initlftjntsr[1:]])
                        numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrp, regrip.robot.initlftjntsr[1:]])
                        numikrlist.append(
                            [regrip.robot.initlftjntsr[0], armjntsgrpworlda, regrip.robot.initlftjntsr[1:]])
                        jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                        jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                        jawwidth.append([gripjawwidth, regrip.robothand.jawwidthopen])
                        jawwidth.append([gripjawwidth, regrip.robothand.jawwidthopen])
                    if nid.startswith('lft'):
                        numikrlist.append(
                            [regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrphanda])
                        numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrp])
                        numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrp])
                        numikrlist.append(
                            [regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrpworlda])
                        jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                        jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                        jawwidth.append([regrip.robothand.jawwidthopen, gripjawwidth])
                        jawwidth.append([regrip.robothand.jawwidthopen, gripjawwidth])
                    objmat4handa = regrip.regg.nodes[nid]['tabletopplacementrotmathanda']
                    objmat4 = regrip.regg.nodes[nid]['tabletopplacementrotmat']
                    objmat4worlda = regrip.regg.nodes[nid]['tabletopplacementrotmatworlda']
                    objmat4list.append(objmat4handa)
                    objmat4list.append(objmat4)
                    objmat4list.append(objmat4)
                    objmat4list.append(objmat4worlda)
                    extendedpathnidlist.append(nid+"x")
                    extendedpathnidlist.append(nid+"o")
                    extendedpathnidlist.append(nid+"c")
                    extendedpathnidlist.append(nid+"w")
                ## middle second
                nid = pathnidlist[i + 1]
                # could be ho
                if nid.startswith('ho'):
                    pass
                else:
                    gripjawwidth = regrip.regg.nodes[nid]['jawwidth']
                    armjntsgrp = regrip.regg.nodes[nid]['armjnts']
                    armjntsgrphanda = regrip.regg.nodes[nid]['armjntshanda']
                    armjntsgrpworlda = regrip.regg.nodes[nid]['armjntsworlda']
                    # initialize
                    # choice
                    if nid.startswith('rgt'):
                        numikrlist.append(
                            [regrip.robot.initlftjntsr[0], armjntsgrpworlda, regrip.robot.initlftjntsr[1:]])
                        numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrp, regrip.robot.initlftjntsr[1:]])
                        numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrp, regrip.robot.initlftjntsr[1:]])
                        numikrlist.append(
                            [regrip.robot.initlftjntsr[0], armjntsgrphanda, regrip.robot.initlftjntsr[1:]])
                        jawwidth.append([gripjawwidth, regrip.robothand.jawwidthopen])
                        jawwidth.append([gripjawwidth, regrip.robothand.jawwidthopen])
                        jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                        jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                    if nid.startswith('lft'):
                        numikrlist.append(
                            [regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrpworlda])
                        numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrp])
                        numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrp])
                        numikrlist.append(
                            [regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrphanda])
                        jawwidth.append([regrip.robothand.jawwidthopen, gripjawwidth])
                        jawwidth.append([regrip.robothand.jawwidthopen, gripjawwidth])
                        jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                        jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                    objmat4worlda = regrip.regg.nodes[nid]['tabletopplacementrotmatworlda']
                    objmat4 = regrip.regg.nodes[nid]['tabletopplacementrotmat']
                    objmat4handa = regrip.regg.nodes[nid]['tabletopplacementrotmathanda']
                    objmat4list.append(objmat4worlda)
                    objmat4list.append(objmat4)
                    objmat4list.append(objmat4)
                    objmat4list.append(objmat4handa)
                    extendedpathnidlist.append(nid+"w")
                    extendedpathnidlist.append(nid+"c")
                    extendedpathnidlist.append(nid+"o")
                    extendedpathnidlist.append(nid+"x")
        else:
            # not two node path, end nodes, transfer
            ## second to last node
            nid = pathnidlist[i]
            # print nid
            if nid.startswith('ho'):
                pass
            else:
                gripjawwidth = regrip.regg.nodes[nid]['jawwidth']
                armjntsgrp = regrip.regg.nodes[nid]['armjnts']
                armjntsgrphanda = regrip.regg.nodes[nid]['armjntshanda']
                armjntsgrpworlda = regrip.regg.nodes[nid]['armjntsworlda']
                # initialize
                # choice
                if nid.startswith('rgt'):
                    numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrphanda, regrip.robot.initlftjntsr[1:]])
                    numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrp, regrip.robot.initlftjntsr[1:]])
                    numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrp, regrip.robot.initlftjntsr[1:]])
                    numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrpworlda, regrip.robot.initlftjntsr[1:]])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                    jawwidth.append([gripjawwidth, regrip.robothand.jawwidthopen])
                    jawwidth.append([gripjawwidth, regrip.robothand.jawwidthopen])
                if nid.startswith('lft'):
                    numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrphanda])
                    numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrp])
                    numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrp])
                    numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrpworlda])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                    jawwidth.append([regrip.robothand.jawwidthopen, gripjawwidth])
                    jawwidth.append([regrip.robothand.jawwidthopen, gripjawwidth])
                objmat4handa = regrip.regg.nodes[nid]['tabletopplacementrotmathanda']
                objmat4 = regrip.regg.nodes[nid]['tabletopplacementrotmat']
                objmat4worlda = regrip.regg.nodes[nid]['tabletopplacementrotmatworlda']
                objmat4list.append(objmat4handa)
                objmat4list.append(objmat4)
                objmat4list.append(objmat4)
                objmat4list.append(objmat4worlda)
                extendedpathnidlist.append(nid+"x")
                extendedpathnidlist.append(nid+"o")
                extendedpathnidlist.append(nid+"c")
                extendedpathnidlist.append(nid+"w")
            ## last node
            nid = pathnidlist[i + 1]
            gripjawwidth = regrip.regg.nodes[nid]['jawwidth']
            armjntsgrp = regrip.regg.nodes[nid]['armjnts']
            armjntsgrphanda = regrip.regg.nodes[nid]['armjntshanda']
            armjntsgrpworlda = regrip.regg.nodes[nid]['armjntsworlda']
            # initialize
            # choice
            if nid.startswith('goalrgt'):
                numikrlist.append([armjntsgrpworlda[0], armjntsgrpworlda[1], regrip.robot.initlftjnts])
                numikrlist.append([armjntsgrp[0], armjntsgrp[1], regrip.robot.initlftjnts])
                jawwidth.append([gripjawwidth, regrip.robothand.jawwidthopen])
                jawwidth.append([gripjawwidth, regrip.robothand.jawwidthopen])
                if ((type is "CO") or (type is "OO")):
                    numikrlist.append([armjntsgrp[0], armjntsgrp[1], regrip.robot.initlftjnts])
                    numikrlist.append([armjntsgrphanda[0], armjntsgrphanda[1], regrip.robot.initlftjnts])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
            if nid.startswith('goallft'):
                numikrlist.append([armjntsgrpworlda[0], regrip.robot.initrgtjnts, armjntsgrpworlda[1]])
                numikrlist.append([armjntsgrp[0], regrip.robot.initrgtjnts, armjntsgrp[1]])
                jawwidth.append([regrip.robothand.jawwidthopen, gripjawwidth])
                jawwidth.append([regrip.robothand.jawwidthopen, gripjawwidth])
                if ((type is "CO") or (type is "OO")):
                    numikrlist.append([armjntsgrp[0], regrip.robot.initrgtjnts, armjntsgrp[1]])
                    numikrlist.append([armjntsgrphanda[0], regrip.robot.initrgtjnts, armjntsgrphanda[1]])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
            objmat4worlda = regrip.regg.nodes[nid]['tabletopplacementrotmatworlda']
            objmat4 = regrip.regg.nodes[nid]['tabletopplacementrotmat']
            objmat4handa = regrip.regg.nodes[nid]['tabletopplacementrotmathanda']
            objmat4list.append(objmat4worlda)
            objmat4list.append(objmat4)
            extendedpathnidlist.append(nid+"w")
            extendedpathnidlist.append(nid+"c")
            if ((type is "CO") or (type is "OO")):
                objmat4list.append(objmat4)
                objmat4list.append(objmat4handa)
                extendedpathnidlist.append(nid+"o")
                extendedpathnidlist.append(nid+"x")

    numikrlist.append([regrip.robot.initjnts[0], regrip.robot.initrgtjnts, regrip.robot.initlftjnts])
    objmat4list.append(regrip.goalrotmat4)
    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
    extendedpathnidlist.append('end')
    return [objmat4list, numikrlist, jawwidth, extendedpathnidlist, pathnidlist]