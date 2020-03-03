import numpy as np
import shapely
# from robotsim.hrp5p import hrp5p
from scipy.spatial import ConvexHull
from shapely.geometry import MultiPoint
from shapely.geometry import Polygon
from shapely.geometry import Point


#A class that handles the robot dynamics, COM amd support polygon


class Hrp5PDynamics(object):
    def __init__(self):
        self.__name = 'hrp5pDynamics'
        self.mindistance = 10000.0
        self.comlist = []
        self.comlistnolegs = []

    def getobjectcom(self,objectmass = 0.0,objectrotmat=None, objectlocalcntrofmasspos = None, objectglobalpos = None ):

        #Returns the COM of the object times its mass

        # if objectglobalcntrofmasspos is not None:
        # objectcom = objectglobalcntrofmasspos
        # objectcom = np.array([objectcom[0] * objectmass, objectcom[1] * objectmass, objectcom[2] * objectmass, objectmass])
        # return objectcom

        if objectrotmat is not None and objectglobalpos is not None:
            objectcom = np.dot(objectrotmat, objectlocalcntrofmasspos) + objectglobalpos
            objectcom = np.array(
            [objectcom[0] * objectmass, objectcom[1] * objectmass, objectcom[2] * objectmass, objectmass])
            return objectcom

        else:
            return np.array([0,0,0,0])
        #
        # if holdinghand != "rgt" and holdinghand != "lft":
        #     return np.array([0,0,0,0])
        #
        # if holdinghand == "rgt":
        #     localaxispos = robot.rgtarm[9]['linkend']
        #     objectrotmat = robot.rgtarm[9]['rotmat']
        #     objectcom = np.dot(objectrotmat, objectlocalcntrofmasspos) + localaxispos
        #     objectcom = np.array([objectcom[0] * objectmass,objectcom[1] * objectmass,objectcom[2] * objectmass,objectmass])
        #     return objectcom
        #
        # if holdinghand == "lft":
        #     localaxispos = robot.lftarm[9]['linkend']
        #     objectrotmat = robot.lftarm[9]['rotmat']
        #     objectcom = np.asarray(np.dot(objectrotmat, objectlocalcntrofmasspos) + localaxispos)
        #     objectcom = np.array([objectcom[0] * objectmass,objectcom[1] * objectmass,objectcom[2] * objectmass,objectmass])
        #     return objectcom



    def getchestcenterofmass(self, robot):

        """
        This function returns a vector with the center of mass TIMES its mass.
        to get the center of mass just divide the first 3 terms of the array by the 4th term (mass)

        return:
        author: Daniel
        date: 20180522
        """

        com = np.array([0, 0, 0])
        mass = 0
        for i in robot.chest:

            com = com + i["mass"] * i["gcntrofmass"]
            mass = mass + i["mass"]
        com = np.array([com[0], com[1], com[2], mass])

        return com

    def getleftarmcenterofmass(self, robot):
        """
        This function returns a vector with the center of mass TIMES its mass.
        to get the center of mass just divide the first 3 terms of the array by the 4th term (mass)

        return:
        author: Daniel
        date: 20180522
        """

        com = np.array([0,0,0])
        mass = 0
        for i in robot.lftarm:
            com = com + i["mass"]*i["gcntrofmass"]
            mass = mass + i["mass"]
       # com = np.array([com[0]/mass,com[1]/mass,com[2]/mass,mass])
        com = np.array([com[0], com[1], com[2], mass])

        return com

    def getrightarmcenterofmass(self, robot):

        """
                This function returns a vector with the center of mass TIMES its mass.
                to get the center of mass just divide the first 3 terms of the array by the 4th term (mass)

                return:
                author: Daniel
                date: 20180522
                """

        com = np.array([0,0,0])
        mass = 0
        for i in robot.rgtarm:
            com = com + i["mass"]*i["gcntrofmass"]
            mass = mass + i["mass"]
        #com = np.array([com[0]/mass,com[1]/mass,com[2]/mass,mass])
        com = np.array([com[0], com[1], com[2], mass])

        return com

    def getleftlegcenterofmass(self, robot):

        """
                This function returns a vector with the center of mass TIMES its mass.
                to get the center of mass just divide the first 3 terms of the array by the 4th term (mass)

                return:
                author: Daniel
                date: 20180522
                """

        com = np.array([0,0,0])
        mass = 0
        for i in robot.lftleg:
            com = com + i["mass"]*i["gcntrofmass"]
            mass = mass + i["mass"]
        #com = np.array([com[0]/mass,com[1]/mass,com[2]/mass,mass])
        com = np.array([com[0], com[1], com[2], mass])

        return com

    def getrightlegcenterofmass(self, robot):

        """
                This function returns a vector with the center of mass TIMES its mass.
                to get the center of mass just divide the first 3 terms of the array by the 4th term (mass)

                return:
                author: Daniel
                date: 20180522
                """

        com = np.array([0,0,0])
        mass = 0
        for i in robot.rgtleg:
            com = com + i["mass"]*i["gcntrofmass"]
            mass = mass + i["mass"]
        #com = np.array([com[0]/mass,com[1]/mass,com[2]/mass,mass])
        com = np.array([com[0], com[1], com[2], mass])

        return com

    def getfeetposition(self, robot):

        leftfootvertices = robot.lftleg[6]['gsolevertices']
        rightfootvertices = robot.rgtleg[6]['gsolevertices']

        return leftfootvertices, rightfootvertices

    def getcenterofmass(self,robot, isholdingobject = False,objectmass = 0.0,objectrotmat=None, objectglobalpos = None, objectlocalcntrofmasspos = None , record = False):

        chestcom = self.getchestcenterofmass(robot)
        leftarmcom = self.getleftarmcenterofmass(robot)
        leftlegcom = self.getleftlegcenterofmass(robot)
        rightarmcom = self.getrightarmcenterofmass(robot)
        rightlegcom = self.getrightlegcenterofmass(robot)
        objectcom = np.array([0,0,0,0])

        if isholdingobject == True:
            objectcom = self.getobjectcom(objectmass=objectmass,objectrotmat=objectrotmat, objectglobalpos = objectglobalpos , objectlocalcntrofmasspos = objectlocalcntrofmasspos)

        com = chestcom + leftarmcom + leftlegcom + rightarmcom + rightlegcom + objectcom
        com = np.array([com[0]/com[3],com[1]/com[3],com[2]/com[3],com[3]])

        if record:
            self.comlist.append(np.array([com[0],com[1],com[2]]))

        com = Point(np.array([com[0],com[1],com[2]]))
        return com

    def getcenterofmass_nolegs(self,robot, isholdingobject = False,objectmass = 0.0,objectrotmat=None, objectglobalpos = None, objectlocalcntrofmasspos = None , record = False):

        chestcom = self.getchestcenterofmass(robot)
        leftarmcom = self.getleftarmcenterofmass(robot)
        leftlegcom = self.getleftlegcenterofmass(robot)
        rightarmcom = self.getrightarmcenterofmass(robot)
        rightlegcom = self.getrightlegcenterofmass(robot)
        objectcom = np.array([0,0,0,0])

        if isholdingobject == True:
            objectcom = self.getobjectcom(objectmass=objectmass,objectrotmat=objectrotmat, objectglobalpos = objectglobalpos , objectlocalcntrofmasspos = objectlocalcntrofmasspos)

        com = chestcom + rightarmcom + rightlegcom + objectcom
        com = np.array([com[0]/com[3],com[1]/com[3],com[2]/com[3],com[3]])

        if record:
            self.comlistnolegs.append(np.array([com[0],com[1],com[2]]))

        com = Point(np.array([com[0],com[1],com[2]]))
        return com

    def getfeetconvexhull(self, robot):

        [leftfootvertices,rightfootvertices] = self.getfeetposition(robot)
        feetvertices = np.concatenate((leftfootvertices,rightfootvertices))
        # feetvertices = rightfootvertices #For one leg balance (right leg)

        hull = Polygon(shapely.geometry.MultiPoint(feetvertices).convex_hull)

        return hull

    def getfeetconvexhullcentroid(self,robot):

        hull = self.getfeetconvexhull(robot)
        centroid = Point(hull.centroid)

        return centroid

    def getfeetconvexhullvertices(self,robot):

        hull = self.getfeetconvexhull(robot)
        hullvertices = MultiPoint(hull.exterior.coords)

        return hullvertices

    def getcomdistance(self,robot, isholdingobject = False,objectmass = 0.0,objectrotmat=None, objectglobalpos = None, objectlocalcntrofmasspos = None ):

        hull = self.getfeetconvexhull(robot)
        centerofmass = self.getcenterofmass(robot, isholdingobject = isholdingobject,objectmass = objectmass,
                                            objectrotmat=objectrotmat, objectglobalpos=objectglobalpos,
                                            objectlocalcntrofmasspos=objectlocalcntrofmasspos)

        distance = centerofmass.distance(hull.exterior)

        return distance

    def getcomdistance_nolegs(self,robot, isholdingobject = False,objectmass = 0.0,objectrotmat=None, objectglobalpos = None, objectlocalcntrofmasspos = None ):

        hull = self.getfeetconvexhull(robot)
        centerofmass = self.getcenterofmass_nolegs(robot, isholdingobject = isholdingobject,objectmass = objectmass,
                                            objectrotmat=objectrotmat, objectglobalpos=objectglobalpos,
                                            objectlocalcntrofmasspos=objectlocalcntrofmasspos)

        distance = centerofmass.distance(hull.exterior)

        return distance

    # def getcomdistance2(self, hull, isholdingobject=False, objectmass=0.0, objectrotmat=None, objectglobalpos=None,
    #                    objectlocalcntrofmasspos=None):
    #
    #     # hull = self.getfeetconvexhull(robot)
    #     # centerofmass = self.getcenterofmass(robot, isholdingobject=isholdingobject, objectmass=objectmass,
    #     #                                     objectrotmat=objectrotmat, objectglobalpos=objectglobalpos,
    #     #                                     objectlocalcntrofmasspos=objectlocalcntrofmasspos)
    #
    #     com = Point(np.array([70.0, 10.0, 0.0]))
    #
    #     distance = com.distance(hull.exterior)
    #
    #     return distance

    def getcomstabilitymeasure(self,robot=None,threshold = None, isholdingobject = False,objectmass = 0.0,objectrotmat=None, objectglobalpos = None, objectlocalcntrofmasspos = None ):
        if threshold == None:
            threshold = 50
        distance = self.getcomdistance(robot, isholdingobject = isholdingobject,objectmass = objectmass,objectrotmat=objectrotmat, objectglobalpos = objectglobalpos, objectlocalcntrofmasspos = objectlocalcntrofmasspos)
        hull = self.getfeetconvexhull(robot)
        centerofmass = self.getcenterofmass(robot=robot,isholdingobject=isholdingobject,objectmass=objectmass,
                                            objectrotmat = objectrotmat, objectglobalpos = objectglobalpos,
                                            objectlocalcntrofmasspos=objectlocalcntrofmasspos)
        # print("COM distance", distance)
        if distance < self.mindistance:
            self.mindistance = distance
            print( "Min COM distance", distance)


        return (distance <= threshold) or (not centerofmass.within(hull)), distance
        # return False, distance

    def relocate_centroid(self, comlist=[], robot=None, centroidpos=None, relocate = False):

        if centroidpos is None:

            comx = []
            comy = []

            for com in comlist:
                comx.append(com[0])
                comy.append(com[1])

            comx = np.asarray(comx)
            comy = np.asarray(comy)

            mean_com_x = comx.mean()
            mean_com_y = comy.mean()

            centroid = self.getfeetconvexhullcentroid(robot)

            centroid = np.array([centroid.x,centroid.y,0])

            if relocate:
                legid = "rgt"
                relocate_vector = np.array([mean_com_x,mean_com_y,0]) - centroid
                if mean_com_y >= 0:
                    relocate_vector = np.array([mean_com_x, mean_com_y * -1.0, 0.0]) - centroid

                robotfeetpos = robot.rgtleg[-1]['linkend']
                tgtpos = robotfeetpos + relocate_vector
                tgtrot = robot.rgtleg[-1]['rotmat']

                rgtlegjntsgoal = robot.numlegik(tgtpos, tgtrot, legid)
                legid = "lft"
                relocate_vector = np.array([mean_com_x, mean_com_y, 0]) - centroid
                if mean_com_y < 0:
                    relocate_vector = np.array([mean_com_x, mean_com_y * -1.0, 0.0]) - centroid
                robotfeetpos = robot.lftleg[-1]['linkend']
                tgtpos = robotfeetpos + relocate_vector

                tgtrot = robot.lftleg[-1]['rotmat']
                lftlegjntsgoal = robot.numlegik(tgtpos, tgtrot, legid)
                # #
                if rgtlegjntsgoal is not None and lftlegjntsgoal is not None:
                    robot.movelegfk(rgtlegjntsgoal, legid="rgt")
                    robot.movelegfk(lftlegjntsgoal, legid="lft")
                    print( "good")
                    relocate_vector = np.array([mean_com_x, mean_com_y, 0]) - centroid
                    print( relocate_vector, "relocation vector!")
            else:
                relocate_vector = np.array([mean_com_x, mean_com_y, 0]) - centroid
                print( repr(relocate_vector), "relocation vector!")
                if mean_com_y >= 0:
                    relocate_vector = np.array([mean_com_x, -1 * mean_com_y , 0.0]) - centroid
                print( repr(relocate_vector), "relocation vector, rgt leg!")

                relocate_vector = np.array([mean_com_x, mean_com_y, 0]) - centroid
                if mean_com_y < 0:
                    relocate_vector = np.array([mean_com_x, -1 * mean_com_y , 0.0]) - centroid
                print( repr(relocate_vector), "relocation vector, lft leg!")


        else:

            comx = []
            comy = []

            for com in comlist:
                comx.append(com[0])
                comy.append(com[1])

            comx = np.asarray(comx)
            comy = np.asarray(comy)

            mean_com_x = comx.mean()
            mean_com_y = comy.mean()

            centroid = centroidpos

            relocate_vector = np.array([mean_com_x, mean_com_y, 0]) - centroid

            print( relocate_vector, "relocation vector!")



