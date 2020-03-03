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
    demolist = [
    # ['L', np.array([-96.31549327,  -3.74002179,  48.66593604]), np.array([[ 0.98074607, -0.0085411 ,  0.19510014],
    #    [ 0.01786082,  0.99877903, -0.0460596 ],
    #    [-0.19446853,  0.04865739,  0.97970115]])],
    #    ['L', np.array([-88.39470049,  -3.20773658,  47.09146426]), np.array([[ 0.97880289, -0.02758914,  0.20293762],
    #    [ 0.03325061,  0.99914562, -0.02454062],
    #    [-0.20208719,  0.03076829,  0.97888413]])],
    #    ['L', np.array([-82.17088415,  -2.0557995 ,  45.94957298]), np.array([[ 0.97379254, -0.03120214,  0.22528735],
    #    [ 0.035296  ,  0.99927643, -0.01416598],
    #    [-0.22468237,  0.02174653,  0.97418941]])],
    #    ['L', np.array([-75.79421264,  -1.09726655,  45.24008915]), np.array([[ 0.96829678, -0.03045245,  0.24793949],
    #    [ 0.03220708,  0.99947655, -0.00302296],
    #    [-0.24771766,  0.01091251,  0.96877081]])],
    #    ['L', np.array([-72.50034361,  -0.8599554 ,  44.34646064]), np.array([[ 0.96595054, -0.02783988,  0.25722437],
    #    [ 0.02851708,  0.99959269,  0.00109799],
    #    [-0.25715019,  0.00627465,  0.96635107]])],
    #    ['L', np.array([-70.55691311,  -0.48355484,  43.68155559]), np.array([[ 0.95854108, -0.02765491,  0.28360914],
    #    [ 0.02845532,  0.99959424,  0.0012979 ],
    #    [-0.28352996,  0.00682605,  0.95893915]])],
    #    ['L', np.array([-68.01551447,  -0.84062415,  43.69497351]), np.array([[ 0.96260809, -0.02172508,  0.27002513],
    #    [ 0.02387735,  0.99970391, -0.00468802],
    #    [-0.26984334,  0.0109602 ,  0.96284192]])],
    #    ['L', np.array([-65.45624316,  -0.70387272,  43.50257523]), np.array([[ 0.95764086, -0.02748047,  0.28665052],
    #    [ 0.02680133,  0.99962099,  0.00629336],
    #    [-0.28671484,  0.00165585,  0.95801453]])],
    #    ['L', np.array([-63.07975482,  -0.39237625,  42.35229017]), np.array([[ 9.54526826e-01, -2.13081335e-02,  2.97362495e-01],
    #    [ 2.24670542e-02,  9.99747463e-01, -4.79776854e-04],
    #    [-2.97277181e-01,  7.13876386e-03,  9.54764544e-01]])],
    #    ['L', np.array([-60.08789539,  -1.60535932,  42.88056682]), np.array([[ 0.95562898, -0.02211225,  0.29374182],
    #    [ 0.02647868,  0.99959004, -0.01089603],
    #    [-0.29338049,  0.01819043,  0.95582264]])],
    #    ['L', np.array([-58.11388425,  -1.62800892,  41.4996245 ]), np.array([[ 0.95349749, -0.02034466,  0.30071356],
    #    [ 0.0279704 ,  0.99938659, -0.02107497],
    #    [-0.30010032,  0.02850595,  0.95348165]])],
    #    ['L', np.array([-57.15134288,  -2.16297186,  41.71151131]), np.array([[ 0.95370709, -0.02029347,  0.3000511 ],
    #    [ 0.02588986,  0.99955691, -0.01468711],
    #    [-0.29962016,  0.02177537,  0.9538101 ]])],
       ['L', np.array([-56.34711289,  -2.3277788 ,  41.486317  ]), np.array([[ 0.957233  , -0.02124666,  0.28853689],
       [ 0.0281112 ,  0.99941128, -0.01966753],
       [-0.28794916,  0.02693747,  0.95726672]])],
       ['L', np.array([-55.29537773,  -2.51049493,  40.57262921]), np.array([[ 0.95681391, -0.01526481,  0.2902998 ],
       [ 0.02533061,  0.99920006, -0.03094758],
       [-0.2895952 ,  0.03696457,  0.95643522]])],
       ['L', np.array([-56.28811304,  -3.55672667,  40.3790682 ]), np.array([[ 0.96447225, -0.01227223,  0.2638989 ],
       [ 0.02384718,  0.99888664, -0.04070254],
       [-0.26310561,  0.04554969,  0.96369113]])],
       ['L', np.array([-55.37825773,  -1.68474561,  39.82546385]), np.array([[ 0.96550984, -0.01636351,  0.25985156],
       [ 0.02114006,  0.99965474, -0.01559763],
       [-0.25950664,  0.02055303,  0.96552264]])],
       ['L', np.array([-54.50492747,  -1.62321191,  39.95768752]), np.array([[ 0.96517502, -0.02170076,  0.26070336],
       [ 0.02545853,  0.99961491, -0.0110453 ],
       [-0.26036325,  0.01729778,  0.96535576]])],
       ['L', np.array([-54.99177087,  -1.22233754,  39.41459372]), np.array([[ 0.96905854, -0.01542571,  0.24634838],
       [ 0.01894627,  0.99974934, -0.01192694],
       [-0.24610265,  0.01622536,  0.96910796]])],
       ['L', np.array([-54.73268736,  -0.90911092,  39.26989627]), np.array([[ 0.96947976, -0.01360526,  0.24479381],
       [ 0.01481142,  0.99988564, -0.0030869 ],
       [-0.24472382,  0.00661853,  0.96957025]])],
       ['L', np.array([-56.56095666,  -0.96448707,  38.51783869]), np.array([[ 0.97539864, -0.01540023,  0.21990957],
       [ 0.0175    ,  0.99981796, -0.00760326],
       [-0.21975248,  0.01126463,  0.97549068]])],
       ['L', np.array([-57.41808535,  -1.07939   ,  38.53897999]), np.array([[ 0.97963831, -0.00684274,  0.20065332],
       [ 0.00829576,  0.99994509, -0.00640148],
       [-0.20059851,  0.00793565,  0.97964136]])],
       ['L', np.array([-59.44218587,  -1.35169849,  37.79899559]), np.array([[ 0.98397869, -0.00628695,  0.17817539],
       [ 0.00843703,  0.99990049, -0.01131218],
       [-0.17808655,  0.01263419,  0.98393369]])],
       ['L', np.array([-60.95621057,  -0.5953807 ,  37.39275103]), np.array([[ 0.98607025,  0.0026192 ,  0.16630839],
       [-0.00112822,  0.99995842, -0.00905904],
       [-0.1663252 ,  0.00874515,  0.98603221]])],
       ['L', np.array([-62.79357403,  -1.05823876,  36.48028262]), np.array([[ 0.99027563,  0.00118221,  0.13911419],
       [ 0.00111986,  0.99986365, -0.01646864],
       [-0.13911473,  0.01646428,  0.99013946]])],
       ['L', np.array([-64.80450837,  -0.23494515,  34.82992182]), np.array([[ 0.99401502, -0.00158832,  0.10923094],
       [ 0.00259749,  0.99995524, -0.00909716],
       [-0.10921163,  0.00932654,  0.9939748 ]])],
       ['L', np.array([-66.22917913,   0.1118955 ,  34.28614898]), np.array([[ 0.99455811, -0.0040383 ,  0.1041043 ],
       [ 0.00425792,  0.99998906, -0.00188753],
       [-0.10409555,  0.0023206 ,  0.9945646 ]])],
       ['L', np.array([-67.10650641,  -0.1193292 ,  33.93423328]), np.array([[ 0.99609031, -0.01031331,  0.08773627],
       [ 0.01003265,  0.99994311,  0.00363922],
       [-0.08776881, -0.00274479,  0.99613706]])],
       ['L', np.array([-68.61183264,  -0.21232315,  32.84707963]), np.array([[ 0.99864274, -0.00538155,  0.05180529],
       [ 0.00527699,  0.99998384,  0.00215483],
       [-0.05181606, -0.00187851,  0.9986549 ]])],
       ['L', np.array([-69.54832495,  -0.32143422,  31.95026492]), np.array([[ 0.9991783 , -0.0107607 ,  0.03907496],
       [ 0.01088172,  0.99993666, -0.0028859 ],
       [-0.03904143,  0.00330862,  0.99923216]])],
       ['L', np.array([-70.96072921,  -0.62346984,  30.96529228]), np.array([[ 0.99990782, -0.01045392,  0.00865956],
       [ 0.01051936,  0.99991613, -0.0075462 ],
       [-0.00857995,  0.00763665,  0.999934  ]])],
       ['L', np.array([-72.35619886,  -0.56641044,  30.18126581]), np.array([[ 0.99985765, -0.01595814, -0.00546159],
       [ 0.01591153,  0.99983752, -0.00847297],
       [ 0.00559591,  0.00838475,  0.99994921]])]]

    demopathlist = []
    for num in range(len(demolist) - 1):
        pose = ['L', np.array([(demolist[num][1][0] + demolist[num + 1][1][0]) / 2,
                          (demolist[num][1][1] + demolist[num + 1][1][1]) / 2,
                          (demolist[num][1][2] + demolist[num + 1][1][2]) / 2]),
                np.array([[(demolist[num][2][0][0] + demolist[num + 1][2][0][0]) / 2,
                           (demolist[num][2][0][1] + demolist[num + 1][2][0][1]) / 2,
                           (demolist[num][2][0][2] + demolist[num + 1][2][0][2]) / 2],
                          [(demolist[num][2][1][0] + demolist[num + 1][2][1][0]) / 2,
                           (demolist[num][2][1][1] + demolist[num + 1][2][1][1]) / 2,
                           (demolist[num][2][1][2] + demolist[num + 1][2][1][2]) / 2],
                          [(demolist[num][2][2][0] + demolist[num + 1][2][2][0]) / 2,
                           (demolist[num][2][2][1] + demolist[num + 1][2][2][1]) / 2,
                           (demolist[num][2][2][2] + demolist[num + 1][2][2][2]) / 2]
                          ])]

        demopathlist.append(pose)

    pitchangle = []
    for pose in demopathlist:
        relpos1 = pose[1]
        relrot1 = pose[2]
        relmat1 = base.pg.npToMat4(relrot1, relpos1)
        virtualgoalpos1, virtualgoalrot1 = setposrpy(relmat1)
        pitchangle.append(virtualgoalrot1[1])
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
    # my_x_ticks = np.arange(0, 2.0, 0.5)
    # my_y_ticks = np.arange(-70, 70, 5)
    # plt.xticks(my_x_ticks)
    # plt.yticks(my_y_ticks)

    deriv = []
    secderiv = []
    deriv1 = []
    secderiv1 = []
    # for time in range(3, len(x)-3):
    for time in range(len(x)):
        # print("2nd derivative is", y_spl_2d(time))
        deriv.append(abs(y_spl_1d(x[time])))
        secderiv.append(abs(y_spl_2d(x[time])))

        a = y_spl_1d(x[time])
        b = y_spl_2d(x[time])
        deriv1.append(a)
        secderiv1.append(b)

    print("1st derivs are", deriv1)
    print("2nd derivs are", secderiv1)

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
    writefile(rearrposeidlist1, "RearrangedLpath(1st deriv).txt")
    writefile(rearrposeidlist2, "RearrangedLpath(2nd deriv).txt")

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
