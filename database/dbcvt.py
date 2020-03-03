# for saving data into mysql
from panda3d.core import *


def mat4ToStr(pdmat4):
    """
    convert a mat4 matrix to a string like e00, e01, e02, ...

    :param pdmat4:
    :return: a string

    author: weiwei
    date: 20161212, tsukuba
    """

    row0 = pdmat4.getRow(0)
    row1 = pdmat4.getRow(1)
    row2 = pdmat4.getRow(2)
    row3 = pdmat4.getRow(3)

    # print([e for e in row0], [e for e in row1], [e for e in row2], [e for e in row3])
    # print(','.join(str(e) for e in row0)+','+','.join(str(e) for e in row1)+\
    #        ','+','.join(str(e) for e in row2)+','+','.join(str(e) for e in row3))

    # the float(format(e, '.12g')) means to format the significant digits of a number to 12
    # mean while, it keeps a number to be "float" when there is only one digit
    return ','.join(('%.6f' % e) for e in row0)+','+','.join(('%.6f' % e) for e in row1)+\
           ','+','.join(('%.6f' % e) for e in row2)+','+','.join(('%.6f' % e) for e in row3)

def strToMat4(dbstr):
    """
    convert a string like e00, e01, e02, ... into Mat4

    :param str:
    :return: panda Mat4

    author: weiwei
    date: 20161212, tsukuba
    """

    exx = dbstr.split(',')
    exxdecimal = list(map(float, exx))
    if(len(exxdecimal) is 16):
        return Mat4(exxdecimal[0], exxdecimal[1], exxdecimal[2], exxdecimal[3],
                    exxdecimal[4], exxdecimal[5], exxdecimal[6], exxdecimal[7],
                    exxdecimal[8], exxdecimal[9], exxdecimal[10], exxdecimal[11],
                    exxdecimal[12], exxdecimal[13], exxdecimal[14], exxdecimal[15])
    else:
        raise Exception("Wrong number of string element in strToMat4! Expected 16, Received %d" % len(exxdecimal))

def strToMat3(dbstr):
    """
    convert a string (16 e) like e00, e01, e02, ... into Mat3

    :param str:
    :return: panda Mat4

    author: weiwei
    date: 20161213, tsukuba
    """

    exx = dbstr.split(',')
    exxdecimal = list(map(float, exx))
    if(len(exxdecimal) is 16):
        return Mat3(exxdecimal[0], exxdecimal[1], exxdecimal[2],
                    exxdecimal[4], exxdecimal[5], exxdecimal[6],
                    exxdecimal[8], exxdecimal[9], exxdecimal[10])
    else:
        raise Exception("Wrong number of string element in strToMat3! Expected 16, Received %d" % len(exxdecimal))

def v3ToStr(v3):
    """
    convert a vbase3 vector to a string like v0,v1,v2

    :param v3:
    :return:
    """

    return ','.join(('%.6f' % e) for e in v3)

def strToV3(dbstr):
    """
    convert a string like v0,v1,v2 to a v3

    :param v3:
    :return:
    """

    exx = dbstr.split(',')
    exxdecimal = list(map(float, exx))
    if(len(exxdecimal) is 3):
        return VBase3(exxdecimal[0], exxdecimal[1], exxdecimal[2])
    else:
        raise Exception("Wrong number of string element in strToV3! Expected 3, Received %d" % len(exxdecimal))

def listToStr(list):
    """
    convert a list to a string like e0, e1, e2, ...

    :param list:
    :return:

    author: weiwei
    date: 20190315, toyonaka
    """

    if(len(list)==0):
        return ''
    row = [('%.6f' % e) for e in list]
    return ','.join(row)


def strToList(dbstr):
    """
    convert a string to a float list [e0, e1, e2] ...

    :param list:
    :return:

    author: weiwei
    date: 20190315, toyonaka
    """

    if(len(dbstr)==0):
        return []
    ex = dbstr.split(',')
    return list(map(float, ex))
