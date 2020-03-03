import os
import numpy as np
import utiltools.robotmath as rm
import pandaplotutils.pandageom as pg
import pandaplotutils.pandactrl as pc
import matplotlib.pyplot as plt
from sklearn.metrics import r2_score
from scipy.interpolate import UnivariateSpline

# def readfile():
#     this_dir, this_filename = os.path.split(__file__)
#     f = open(os.path.join(this_dir, "document", "Lpath.txt"), "r")
#     # if f.mode == "r":
#     #     contents = f.read()
#     #     print(contents)
#
#     f1 = f.readlines()
#     for content in f1:
#         print(content)

def writefile(list, filename):
    this_dir, this_filename = os.path.split(__file__)
    f = open(os.path.join(this_dir, "document", filename), "w")
    for poses in list:
        # print(poses)
        f.write(str(poses)+'\n')
    f.close()

def setposrpy(Mat4):
    RPY = rm.euler_from_matrix(pg.npToMat3(np.array([[Mat4[0][0], Mat4[0][1], Mat4[0][2]],
                                                     [Mat4[1][0], Mat4[1][1], Mat4[1][2]],
                                                     [Mat4[2][0], Mat4[2][1], Mat4[2][2]]])))
    Pos = np.array([Mat4[3][0], Mat4[3][1], Mat4[3][2]])

    return Pos, RPY


if __name__=="__main__":
    # main()
    base = pc.World(camp=[0, 800, 0], lookatp=[0, 0, 0], up=[0, 0, 1], fov=40, w=1920, h=1080)
    demopathlist = [
       #  ['tenon', np.array([  -4.04583369, -104.16197312,   23.1199353 ]), np.array([[ 0.99974087, -0.0016003 , -0.02270569],
       # [ 0.00710675,  0.96960421,  0.24457558],
       # [ 0.02162404, -0.24467356,  0.96936435]])], ['tenon', np.array([  -4.15740948, -101.98736154,   22.06333484]), np.array([[ 0.99970242,  0.00414848, -0.0240376 ],
       # [ 0.00230123,  0.96499787,  0.26224792],
       # [ 0.02428414, -0.26222515,  0.96470105]])], ['tenon', np.array([ -4.22084443, -98.79402275,  21.03861148]), np.array([[ 0.99947416,  0.01546587, -0.02850096],
       # [-0.00684746,  0.95977442,  0.28068877],
       # [ 0.03169568, -0.28034594,  0.95937553]])], ['tenon', np.array([ -4.10887824, -98.5493264 ,  20.94525193]), np.array([[ 0.99937079,  0.01578947, -0.03176019],
       # [-0.00600283,  0.95782397,  0.28729285],
       # [ 0.03495685, -0.28692137,  0.95731609]])], ['tenon', np.array([ -4.4812868 , -97.10304938,  20.02358111]), np.array([[ 0.99929034,  0.01948534, -0.03223418],
       # [-0.00874052,  0.95239232,  0.30475015],
       # [ 0.03663778, -0.30425217,  0.95188667]])], ['tenon', np.array([ -4.42376789, -95.95803926,  19.12217008]), np.array([[ 0.99908265,  0.0223119 , -0.03655168],
       # [-0.01001473,  0.95160996,  0.30714528],
       # [ 0.04163601, -0.30649741,  0.95096035]])], ['tenon', np.array([ -4.74087077, -94.91769908,  19.09567653]), np.array([[ 0.99892551,  0.02284427, -0.04032115],
       # [-0.00875253,  0.94740367,  0.31992162],
       # [ 0.04550883, -0.31922495,  0.94658562]])], ['tenon', np.array([ -4.63113477, -92.11649895,  18.45226743]), np.array([[ 0.99898085,  0.02490629, -0.03764123],
       # [-0.01112878,  0.94413861,  0.32936063],
       # [ 0.0437417 , -0.32860608,  0.94345354]])], ['tenon', np.array([ -5.13149497, -91.55184237,  18.64164204]), np.array([[ 0.99888174,  0.02783592, -0.03821505],
       # [-0.01341096,  0.94192435,  0.33555732],
       # [ 0.0453363 , -0.33466955,  0.94124433]])], ['tenon', np.array([ -4.97370855, -89.14466934,  17.27138291]), np.array([[ 0.9986462 ,  0.03144915, -0.04143179],
       # [-0.01584136,  0.94256927,  0.33363485],
       # [ 0.04954493, -0.33252684,  0.94179141]])], ['tenon', np.array([ -5.07478475, -87.92992803,  16.99237531]), np.array([[ 0.99862935,  0.0333897 , -0.04030673],
       # [-0.01805877,  0.94260408,  0.33342376],
       # [ 0.04912624, -0.33223882,  0.94191506]])], ['tenon', np.array([ -4.95151865, -87.20843557,  17.04380296]), np.array([[ 0.99862506,  0.0372213 , -0.03691429],
       # [-0.0228037 ,  0.94249623,  0.33343791],
       # [ 0.04720268, -0.33213763,  0.94204904]])], ['tenon', np.array([ -5.02373374, -85.02163048,  16.00118179]), np.array([[ 0.99844892,  0.03973972, -0.03899403],
       # [-0.02387037,  0.93828921,  0.34502704],
       # [ 0.05029897, -0.34356099,  0.93778233]])], ['tenon', np.array([ -5.47042256, -83.76613696,  15.53264763]), np.array([[ 0.99812603,  0.04779281, -0.03821492],
       # [-0.03155229,  0.93704214,  0.34778797],
       # [ 0.05243065, -0.34593046,  0.93679404]])], ['tenon', np.array([ -5.80312813, -83.49540639,  15.15155687]), np.array([[ 0.99802102,  0.04902771, -0.03937654],
       # [-0.03240608,  0.93763734,  0.34610124],
       # [ 0.05388945, -0.34414025,  0.9373705 ]])], ['tenon', np.array([ -5.92780922, -82.82085876,  14.2675713 ]), np.array([[ 0.99787971,  0.04858814, -0.04330334],
       # [-0.03088563,  0.93916988,  0.34206161],
       # [ 0.05728942, -0.33999881,  0.93867919]])], ['tenon', np.array([ -5.74029883, -82.10890346,  14.6121434 ]), np.array([[ 0.99817467,  0.04618868, -0.03890878],
       # [-0.03032195,  0.94046351,  0.33853948],
       # [ 0.05222912, -0.33674172,  0.94014734]])], ['tenon', np.array([ -5.29725738, -81.75297135,  14.8513588 ]), np.array([[ 0.9985074 ,  0.04417709, -0.03211494],
       # [-0.03096698,  0.94228286,  0.33338292],
       # [ 0.0449893 , -0.33189077,  0.94224441]])], ['tenon', np.array([ -5.55421816, -80.6005074 ,  15.17077438]), np.array([[ 0.99821871,  0.05004041, -0.0324889 ],
       # [-0.03681135,  0.94511233,  0.32466534],
       # [ 0.04695209, -0.32289102,  0.94527077]])], ['tenon', np.array([ -5.27128789, -80.07956882,  14.54111621]), np.array([[ 0.99830522,  0.04805565, -0.03282388],
       # [-0.0349376 ,  0.9459736 ,  0.32235603],
       # [ 0.04654158, -0.32066284,  0.94604919]])],
       #  ['tenon', np.array([ -5.43976223, -79.93434675,  15.00888035]), np.array([[ 0.99812447,  0.05266657, -0.03120661],
       # [-0.03973406,  0.94514486,  0.32422602],
       # [ 0.04657064, -0.32237793,  0.94546479]])],
        ['tenon', np.array([ -5.73826664, -79.25563482,  14.92496054]), np.array([[ 0.9981996 ,  0.0533105 , -0.0274875 ],
       [-0.0416214 ,  0.94563808,  0.32254643],
       [ 0.04318831, -0.3208217 ,  0.94615453]])], ['tenon', np.array([ -5.80018528, -78.35461208,  14.89169411]), np.array([[ 0.99820279,  0.05226622, -0.02931364],
       [-0.04039872,  0.9482276 ,  0.31501193],
       [ 0.04426054, -0.31326155,  0.94863494]])], ['tenon', np.array([ -5.53747054, -77.27624206,  13.92599495]), np.array([[ 0.99876397,  0.03807851, -0.03194821],
       [-0.02626875,  0.95001581,  0.31109483],
       [ 0.04219744, -0.30987102,  0.94984174]])],

       #  ['tenon', np.array([ -4.73403702, -76.43807961,  12.80756261]), np.array([[ 0.99896045,  0.03176033, -0.03270039],
       # [-0.02082626,  0.95607753,  0.2923734 ],
       # [ 0.04055007, -0.29138845,  0.95574494]])],
       #  ['tenon', np.array([ -4.67127682, -77.81740801,  13.07569503]), np.array([[ 0.99916309,  0.0279956 , -0.02982507],
       # [-0.01879563,  0.96179374,  0.27312915],
       # [ 0.03633194, -0.27233994,  0.96151486]])], ['tenon', np.array([ -4.85683349, -77.98563823,  12.42787242]), np.array([[ 0.99924921,  0.02413556, -0.03030462],
       # [-0.01506621,  0.96274891,  0.26997707],
       # [ 0.03569183, -0.26931775,  0.96238971]])], ['tenon', np.array([ -4.80531454, -77.27207526,  11.77009908]), np.array([[ 0.99928915,  0.02224825, -0.03043389],
       # [-0.01326907,  0.96320264,  0.26844863],
       # [ 0.03528646, -0.26785399,  0.96281309]])],
       #
       #  ['tenon', np.array([ -4.06274509, -77.52445986,  10.55478317]), np.array([[ 0.99932878,  0.01294553, -0.03426675],
       # [-0.00381878,  0.96719059,  0.25402332],
       # [ 0.0364309 , -0.25372193,  0.96659085]])],
       #  ['tenon', np.array([ -3.35131707, -77.48500977,   8.78772353]), np.array([[ 0.99952213,  0.00461333, -0.03056582],
       # [ 0.00208025,  0.97652102,  0.21541226],
       # [ 0.03084202, -0.21537291,  0.97604473]])],

        ['tenon', np.array([ -3.11935112, -76.35935833,   8.58060048]), np.array([[ 9.99597583e-01,  6.17025122e-03, -2.76873378e-02],
       [-9.30837334e-05,  9.76764322e-01,  2.14316150e-01],
       [ 2.83664259e-02, -2.14227336e-01,  9.76371877e-01]])],
                                                   ['tenon', np.array([ -3.0759386 , -75.45105842,   9.44781914]), np.array([[ 0.99965226,  0.00321951, -0.0261727 ],
       [ 0.00232887,  0.97786218,  0.20923676],
       [ 0.02626687, -0.20922492,  0.97751475]])],
        ['tenon', np.array([ -2.9077887 , -74.48581032,   9.41741975]), np.array([[ 9.99676333e-01,  6.18829202e-03, -2.46724752e-02],
       [-7.97622781e-04,  9.77105055e-01,  2.12756374e-01],
       [ 2.54242102e-02, -2.12667790e-01,  9.76793673e-01]])], ['tenon', np.array([ -2.76688882, -73.44726693,   9.22560804]), np.array([[ 9.99741124e-01,  5.40731047e-03, -2.20995138e-02],
       [-5.41198733e-04,  9.76723537e-01,  2.14501519e-01],
       [ 2.27450382e-02, -2.14434014e-01,  9.76473620e-01]])], ['tenon', np.array([ -2.60037906, -72.31603003,   9.45824538]), np.array([[ 0.99978227,  0.00907408, -0.01878822],
       [-0.00470301,  0.97531228,  0.22078011],
       [ 0.02032787, -0.22064367,  0.97514265]])], ['tenon', np.array([ -1.54096425, -70.13011602,   8.54937833]), np.array([[ 0.99987456,  0.00680028, -0.01430236],
       [-0.00355047,  0.97638197,  0.21602244],
       [ 0.01543363, -0.21594453,  0.97628363]])], ['tenon', np.array([ -0.93197257, -69.78465376,   7.95621096]), np.array([[ 9.99964061e-01,  1.85683689e-04, -8.47094424e-03],
       [ 1.49969138e-03,  9.80096973e-01,  1.98513907e-01],
       [ 8.33918090e-03, -1.98519494e-01,  9.80061425e-01]])], ['tenon', np.array([  0.74487922, -69.72855114,   5.85952825]), np.array([[ 0.99995613,  0.00305078, -0.00886608],
       [-0.00156433,  0.98661524,  0.16305846],
       [ 0.00924492, -0.16303738,  0.98657658]])], ['tenon', np.array([  1.01042125, -69.37454747,   4.84276747]), np.array([[ 0.99998092,  0.00385806, -0.0048102 ],
       [-0.00313187,  0.98975198,  0.14276298],
       [ 0.00531165, -0.14274518,  0.98974521]])], ['tenon', np.array([  0.86478442, -69.72282409,   3.14961593]), np.array([[ 0.99995396,  0.00826717, -0.00486443],
       [-0.00768089,  0.99388018,  0.11019706],
       [ 0.00574571, -0.11015458,  0.99389778]])], ['tenon', np.array([  0.63555423, -71.21651216,   1.5015602 ]), np.array([[ 0.99991912,  0.00898789, -0.00900115],
       [-0.00828383,  0.99711775,  0.07541556],
       [ 0.00965298, -0.07533489,  0.99711156]])], ['tenon', np.array([  0.64673777, -70.90885034,   0.44502828]), np.array([[ 0.99988347,  0.01164553, -0.00986759],
       [-0.01097423,  0.99778893,  0.0655498 ],
       [ 0.0106091 , -0.06543388,  0.99780049]])],
       #  ['tenon', np.array([  1.01594054, -70.95339994,  -1.0385497 ]), np.array([[ 0.99986738,  0.00998399, -0.01286533],
       # [-0.00938   ,  0.99888907,  0.04618202],
       # [ 0.01331225, -0.04605519,  0.9988501 ]])], ['tenon', np.array([  0.89600598, -72.09707394,  -0.99919087]), np.array([[ 0.99982449,  0.00508553, -0.01803207],
       # [-0.00445615,  0.9993853 ,  0.03477323],
       # [ 0.0181979 , -0.03468676,  0.99923252]])], ['tenon', np.array([  1.02430134, -72.52198167,  -1.19352395]), np.array([[ 9.99915742e-01,  4.83647037e-04, -1.29719410e-02],
       # [-2.42405604e-05,  9.99373487e-01,  3.53920852e-02],
       # [ 1.29809575e-02, -3.53887958e-02,  9.99289282e-01]])],
       #  ['tenon', np.array([  1.02309862, -73.85656618,  -1.96204799]), np.array([[ 9.99833091e-01,  1.07159543e-03, -1.82398505e-02],
       # [-6.58602403e-04,  9.99743666e-01,  2.26331180e-02],
       # [ 1.82594877e-02, -2.26173402e-02,  9.99577460e-01]])],
       #  ['tenon', np.array([  0.99388829, -74.77597305,  -1.7230376 ]), np.array([[ 0.9999211 , -0.00101142, -0.01252576],
       # [ 0.00132954,  0.99967608,  0.02541616],
       # [ 0.01249595, -0.02543082,  0.99959852]])], ['tenon', np.array([  1.27334324, -75.02823971,  -2.4161915 ]), np.array([[ 0.99982538, -0.00176405, -0.01860558],
       # [ 0.00213628,  0.9997976 ,  0.02000499],
       # [ 0.01856648, -0.02004123,  0.99962674]])],
       #  ['tenon', np.array([  1.14836994, -76.44438895,  -2.2606039 ]), np.array([[ 0.99967041, -0.00393817, -0.02536546],
       # [ 0.00438896,  0.99983302,  0.01773988],
       # [ 0.02529143, -0.01784537,  0.99952085]])], ['tenon', np.array([  0.18876637, -76.84599542,  -1.97079964]), np.array([[ 0.99940112, -0.0048191 , -0.03426752],
       # [ 0.00531785,  0.99988112,  0.01447772],
       # [ 0.03419375, -0.0146513 ,  0.99930779]])], ['tenon', np.array([ 1.81102699e-02, -7.76417530e+01, -2.77148790e+00]), np.array([[ 0.99898196, -0.00230206, -0.04505292],
       # [ 0.00240096,  0.99999486,  0.00214156],
       # [ 0.04504771, -0.00224756,  0.99898224]])], ['tenon', np.array([  0.43094636, -81.14301362,  -3.55673126]), np.array([[ 0.99998867, -0.00117267, -0.00461796],
       # [ 0.00113116,  0.99995901, -0.00898678],
       # [ 0.00462826,  0.00898148,  0.99994896]])]
    ]



    # demopathlist = []
    # for num in range(len(demolist) - 1):
    #     pose = ['L', np.array([(demolist[num][1][0] + demolist[num + 1][1][0]) / 2,
    #                       (demolist[num][1][1] + demolist[num + 1][1][1]) / 2,
    #                       (demolist[num][1][2] + demolist[num + 1][1][2]) / 2]),
    #             np.array([[(demolist[num][2][0][0] + demolist[num + 1][2][0][0]) / 2,
    #                        (demolist[num][2][0][1] + demolist[num + 1][2][0][1]) / 2,
    #                        (demolist[num][2][0][2] + demolist[num + 1][2][0][2]) / 2],
    #                       [(demolist[num][2][1][0] + demolist[num + 1][2][1][0]) / 2,
    #                        (demolist[num][2][1][1] + demolist[num + 1][2][1][1]) / 2,
    #                        (demolist[num][2][1][2] + demolist[num + 1][2][1][2]) / 2],
    #                       [(demolist[num][2][2][0] + demolist[num + 1][2][2][0]) / 2,
    #                        (demolist[num][2][2][1] + demolist[num + 1][2][2][1]) / 2,
    #                        (demolist[num][2][2][2] + demolist[num + 1][2][2][2]) / 2]
    #                       ])]
    #
    #     demopathlist.append(pose)

    pitchangle = []
    for pose in demopathlist:
        relpos1 = pose[1]
        relrot1 = pose[2]
        relmat1 = base.pg.npToMat4(relrot1, relpos1)
        virtualgoalpos1, virtualgoalrot1 = setposrpy(relmat1)
        pitchangle.append(virtualgoalrot1[0])
        # print("pitch angle =", virtualgoalrot1[1])

    # print(len(pitchangle))
    # print(pitchangle)

    x = np.arange(0.0, len(pitchangle)*0.1, 0.1)
    print(len(x))
    # x = np.delete(x, [len(x)])
    plt.figure(1)
    plt.subplot(131)
    # origin = plt.scatter(x[3:-3], pitchangle[3:-3], c="red", label="origin data")
    origin = plt.scatter(x, pitchangle, c="red", label="origin data")
    ## use curve fitting
    # z = np.polyfit(x, pitchangle, 15)
    # p = np.poly1d(z)
    # print(p)
    # fitting = plt.plot(x[23:-3], p(x[23:-3]), "-", label="fitting curve")
    # q = p.deriv()
    # qq = q.deriv()
    # # print(q)
    # plt.plot(x[23:-3], q(x[23:-3]), label="1st deriv")
    # plt.plot(x[23:-3], qq(x[23:-3]), label="2nd deriv")

    ## use interpolation
    y_spl = UnivariateSpline(x, pitchangle, s=10, k=5)
    # plt.plot(x[3:-3], y_spl(x[3:-3]))
    plt.plot(x, y_spl(x), label="Interpolation")
    plt.legend()
    plt.grid()
    plt.title("Pitch angles of key poses")
    plt.xlabel('time(s)')
    plt.ylabel('pitch angle(°)')

    plt.subplot(132)
    y_spl_1d = y_spl.derivative(n=1)
    # plt.plot(x[3:-3], y_spl_1d(x[3:-3]), label="1st derivative")
    plt.plot(x, y_spl_1d(x), "g", label="1st derivative")
    plt.grid()
    plt.title("1st derivative of pitch angle of key poses")
    plt.xlabel('time(s)')
    plt.ylabel('1st derivative of pitch angles(°/s)')

    plt.subplot(133)
    y_spl_2d = y_spl.derivative(n=2)
    # plt.plot(x[3:-3], y_spl_2d(x[3:-3]), "--", label="2nd derivative")
    plt.plot(x, y_spl_2d(x), "y", label="2nd derivative")
    plt.grid()
    plt.title("2nd derivative of pitch angle of key poses")
    plt.xlabel('time(s)')
    plt.ylabel('2nd derivative of pitch angles(°/s^2)')
    # my_x_ticks = np.arange(0, 1.5, 0.5)
    # my_y_ticks = np.arange(-1300, 900, 100)
    # plt.xticks(my_x_ticks)
    # plt.yticks(my_y_ticks)

    deriv = []
    secderiv = []
    # for time in range(0, len(x)-2):
    for time in range(len(x)):
        deriv.append(abs(y_spl_1d(x[time])))
        secderiv.append(abs(y_spl_2d(x[time])))
        # deriv.append(y_spl_1d(x[time]))
        # secderiv.append(y_spl_2d(x[time]))
    rearrfd = sorted(deriv, reverse=True)
    # rearrfd = rearrfd[3:-3]
    rearrsd = sorted(secderiv, reverse=True)
    # rearrsd = rearrsd[3:-3]
    print(rearrfd)
    print(rearrsd)
    poseidlist1 = []
    poseidlist2 = []
    for fd in rearrfd:
        poseid1 = np.where(deriv == fd)[0][0]
        poseidlist1.append(poseid1)
    for sd in rearrsd:
        poseid2 = np.where(secderiv == sd)[0][0]
        poseidlist2.append(poseid2)
    # print(poseidlist)
    rearrposeidlist1 = []
    rearrposeidlist2 = []
    for num in poseidlist1:
        k = 0
        # k = 3
        rearrposeidlist1.append(demopathlist[num+k][1])
        rearrposeidlist1.append(demopathlist[num+k][2][0])
        rearrposeidlist1.append(demopathlist[num+k][2][1])
        rearrposeidlist1.append(demopathlist[num+k][2][2])
    print(rearrposeidlist1)  # Rearranged demo poses according to derivative
    for num in poseidlist2:
        k = 0
        # k = 3
        rearrposeidlist2.append(demopathlist[num+k][1])
        rearrposeidlist2.append(demopathlist[num+k][2][0])
        rearrposeidlist2.append(demopathlist[num+k][2][1])
        rearrposeidlist2.append(demopathlist[num+k][2][2])
    print(rearrposeidlist2)  # Rearranged demo poses according to 2nd derivative
    writefile(rearrposeidlist1, "Rearrangedtenonpath(1st deriv).txt")
    writefile(rearrposeidlist2, "Rearrangedtenonpath(2nd deriv).txt")

    # errorlist = []
    # coeflist = []
    # for m in range(19):
    #     z = np.polyfit(x, pitchangle, m)
    #     p = np.poly1d(z)
    #     print(p)
    #     coefficient_of_dermination = r2_score(pitchangle, p(x))
    #     coeflist.append(coefficient_of_dermination)
    #     print("coefficient_of_dermination =", coefficient_of_dermination)
    #     error = 0
    #     for time in x:
    #         # print(time)
    #         time_id = np.where(x == time)[0][0]
    #         error += pow(pitchangle[time_id] - p(time), 2)
    #     errorlist.append(error)
    #
    # print(errorlist)
    # print(coeflist)
    # m_chosen = errorlist.index(min(errorlist))
    # m2_chosen = coeflist.index(max(coeflist))
    # print("id of minimum error", m_chosen)
    # print("id of maximum coefficient", m2_chosen)
    # print("minimum error =", min(errorlist))
    # print("maximum coefficient", max(coeflist))
    # z = np.polyfit(x, pitchangle, m_chosen)
    # p = np.poly1d(z)
    # fitting = plt.plot(x, p(x), "-", label="fitting curve")

    #
    # q = p.deriv()
    # print(q)
    # for xx in x:
    #     print(q(xx))
    #
    # for xx in x:
    #     eps = np.sqrt(np.finfo(float).eps) * (1.0 + xx)
    #     print((p(xx + eps) - p(xx - eps)) / (2.0 * eps * xx))

    plt.show()
