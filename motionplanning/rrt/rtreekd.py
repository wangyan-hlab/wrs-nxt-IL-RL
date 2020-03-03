from rtree import index

class RtreeKD():
    """
    12D rtree
    """

    def __init__(self, dimension):
        p = index.Property()
        p.dimension = dimension
        p.storage= index.RT_Memory
        self.__idxkd = index.Index(str(p.dimension)+"d_index", properties = p)
        self.__npoints = 0
        self.__dimension = dimension

    def insert(self, point):
        """
        The dimension of a point must be equal to dimension of the tree

        :param point: a kd list
        :return:

        author: weiwei
        date: 20180520
        """

        # print(self.__npoints)
        # print(self.__dimension)
        # print(point+point)
        self.__idxkd.insert(self.__npoints, point+point, point)
        self.__npoints += 1

    def nearest(self, point):
        """
        The dimension of a point must be equal to dimension of the tree

        :param point: a kd list
        :return:

        author: weiwei
        date: 20180520
        """

        return list(self.__idxkd.nearest(point+point, 1, objects='raw'))[0]

if __name__ == '__main__':
    rt = RtreeKD(6)
    points0 = [0,0,0,0,0,0]
    points1 = [1,1,0,0,0,0]
    points2 = [1,1,0,0,0,0]
    points3 = [1,0.1,0,0,0,0]
    # rt = RtreeKD(3)
    # points0 = [0,0,0]
    # points1 = [1,1,1]
    # points2 = [0,1,0]
    rt.insert(points0)
    rt.insert(points1)
    print(rt.nearest(points3))