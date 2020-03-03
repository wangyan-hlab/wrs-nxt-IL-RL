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
        ['twostairpeg', np.array([-68.16825573, -3.07134597, 68.39087797]), np.array([[0.85085546, -0.52437833, 0.03274851],
                                                                                [0.52494641, 0.85105708, -0.0115296],
                                                                                [-0.02182493, 0.0270013, 0.999397]])],
        ['twostairpeg', np.array([-67.41246327, -2.04825544, 65.92249945]), np.array([[0.85080635, -0.52440196, 0.03363214],
                                                                                [0.52503067, 0.85098121, -0.01317888],
                                                                                [-0.02170929, 0.02887056,
                                                                                 0.99934735]])],
        ['twostairpeg', np.array([-66.82948049, -1.6261596, 63.45758577]), np.array([[0.84944539, -0.52646185, 0.03578455],
                                                                               [0.52720983, 0.84959196, -0.01559745],
                                                                               [-0.02219079, 0.03211517, 0.99923789]])],
        ['twostairpeg', np.array([-66.67455365, -5.18839509, 64.98130156]), np.array([[0.85569613, -0.51683654, 0.02576911],
                                                                                [0.51727699, 0.85569128, -0.01472427],
                                                                                [-0.01444036, 0.02592932, 0.9995594]])],
        ['twostairpeg', np.array([-66.11533036, -3.77260937, 63.31175485]), np.array([[0.86104679, -0.50815526, 0.01940602],
                                                                                [0.5084426, 0.86096871, -0.01479374],
                                                                                [-0.00919046, 0.02260502,
                                                                                 0.99970214]])],
        ['twostairpeg', np.array([-66.17023649, -2.41809204, 61.85567189]), np.array([[0.85681277, -0.51476307, 0.029848],
                                                                                [0.51530262, 0.85689187, -0.01412486],
                                                                                [-0.01830553, 0.02748315,
                                                                                 0.99945464]])],
        ['twostairpeg', np.array([-65.71311877, -2.11160596, 60.44442524]), np.array([[0.85869089, -0.51196057, 0.02337279],
                                                                                [0.51231258, 0.85870683, -0.01258467],
                                                                                [-0.01362754, 0.02278048,
                                                                                 0.99964763]])],
        ['twostairpeg', np.array([-65.83378281, -2.85199983, 59.59547463]), np.array([[0.86236242, -0.50612572, 0.01295679],
                                                                                [0.50623557, 0.8623651, -0.00720224],
                                                                                [-0.00752825, 0.01277012,
                                                                                 0.99989006]])],
        ['twostairpeg', np.array([-65.39377242, -3.27001026, 57.37386793]), np.array([[0.85978966, -0.51052201, 0.01136229],
                                                                                [0.51057193, 0.8598331, -0.00183194],
                                                                                [-0.00883437, 0.00737644,
                                                                                 0.99993372]])],
        ['twostairpeg', np.array([-65.89944375, -2.65051079, 55.47221608]), np.array([[0.85792282, -0.51363373, 0.01220392],
                                                                                [0.51364451, 0.85799956, 0.00247427],
                                                                                [-0.01174182, 0.00414575,
                                                                                 0.99992246]])],
        ['twostairpeg', np.array([-66.10795433, -3.29982024, 52.73843384]),
         np.array([[8.55397009e-01, -5.17853558e-01, 1.11308664e-02],
                [5.17891892e-01, 8.55445772e-01, -6.72578199e-04],
                [-9.17351132e-03, 6.33995021e-03, 9.99937812e-01]])],
        ['twostairpeg', np.array([-65.42397626, -2.66280436, 48.97823825]),
         np.array([[8.56382614e-01, -5.16253392e-01, 9.55247200e-03],
                [5.16282690e-01, 8.56417861e-01, -7.12714418e-04],
                [-7.81293079e-03, 5.54215284e-03, 9.99954150e-01]])],
        ['twostairpeg', np.array([-65.3949972, -1.9719965, 43.11363394]),
         np.array([[8.52161723e-01, -5.23243346e-01, 6.06611417e-03],
                [5.23227767e-01, 8.52183273e-01, 4.04461570e-03],
                [-7.28580985e-03, -2.72695216e-04, 9.99973421e-01]])],
        ['twostairpeg', np.array([-65.25483813, -2.55604968, 38.6930268]), np.array([[0.85322314, -0.52152784, 0.00434025],
                                                                               [0.52154348, 0.85316408, -0.01017418],
                                                                               [0.00160315, 0.01094442, 0.99993884]])],
        ['twostairpeg', np.array([-64.48635906, -1.97167888, 34.34894129]),
         np.array([[8.46747606e-01, -5.31994761e-01, 2.33049240e-04],
                [5.31987172e-01, 8.46732954e-01, -5.72775739e-03],
                [2.84986200e-03, 4.97397079e-03, 9.99983585e-01]])],
        ['twostairpeg', np.array([-64.60163481, -2.23327432, 30.80605679]), np.array([[0.84528563, -0.53426443, -0.00732125],
                                                                                [0.53426523, 0.84531448, -0.00200984],
                                                                                [0.00726261, -0.00221255,
                                                                                 0.99997118]])],
        ['twostairpeg', np.array([-64.34008629, -1.50252529, 27.17762189]), np.array([[0.84250367, -0.53867638, -0.00391538],
                                                                                [0.53864753, 0.84250594, -0.00652307],
                                                                                [0.00681249, 0.00338666, 0.99997108]])],
        ['twostairpeg', np.array([-64.45686942, -1.25056565, 24.5417452]), np.array([[0.83453583, -0.55093368, 0.00469691],
                                                                               [0.5509457, 0.83444787, -0.01246776],
                                                                               [0.00294962, 0.01299255, 0.99991117]])],
        ['twostairpeg', np.array([-64.33036467, -1.41276727, 23.14804821]), np.array([[0.83306407, -0.55310446, 0.00892421],
                                                                                [0.55316327, 0.83304688, -0.00656054],
                                                                                [-0.00380564, 0.01040192,
                                                                                 0.99993869]])],
        ['twostairpeg', np.array([-64.7755037, -1.74119809, 20.7214468]), np.array([[0.83378879, -0.55182273, 0.01697171],
                                                                              [0.55203883, 0.83371858, -0.01290111],
                                                                              [-0.00703052, 0.02012579, 0.99977271]])],
        ['twostairpeg', np.array([-64.8950338, -2.18261605, 18.62756754]), np.array([[0.83941053, -0.54313596, 0.01983789],
                                                                               [0.54338871, 0.83941453, -0.01058307],
                                                                               [-0.01090417, 0.01966327, 0.99974722]])],
        ['twostairpeg', np.array([-64.45110847, -2.15165518, 16.42787823]), np.array([[0.83836743, -0.54425558, 0.03043074],
                                                                                [0.54461129, 0.83867784, -0.00424894],
                                                                                [-0.02320909, 0.02013507, 0.9995279]])],
        ['twostairpeg', np.array([-64.92345883, -1.4504101, 15.19893324]),
         np.array([[8.33358759e-01, -5.51630692e-01, 3.48861451e-02],
                [5.51940676e-01, 8.33882874e-01, 8.81841113e-04],
                [-2.95774261e-02, 1.85201821e-02, 9.99390939e-01]])],
        ['twostairpeg', np.array([-6.50254465e+01, 9.06037395e-03, 1.34434859e+01]),
         np.array([[0.8230719, -0.56620226, 0.04435984],
                [0.56672186, 0.82390851, 0.00103645],
                [-0.0371353, 0.02428667, 0.99901503]])],
        ['twostairpeg', np.array([-64.73706522, 0.13017901, 12.97164992]), np.array([[0.82406623, -0.56476962, 0.04416018],
                                                                               [0.56436597, 0.82522277, 0.02232285],
                                                                               [-0.04904919, 0.00652708, 0.99877501]])],
        ['twostairpeg', np.array([-64.93962831, 1.04268524, 11.10688264]), np.array([[0.81563383, -0.57607886, 0.05361485],
                                                                               [0.57518219, 0.8173823, 0.03242894],
                                                                               [-0.06250547, 0.00438818, 0.99803492]])],
        ['twostairpeg', np.array([-64.93144078, 0.39778945, 11.40186743]), np.array([[0.81625413, -0.57676569, 0.03272101],
                                                                               [0.57553863, 0.81679172, 0.04008408],
                                                                               [-0.04984535, -0.01388655, 0.9986604]])],
        ['twostairpeg', np.array([-65.15301265, 1.37963641, 9.77591474]), np.array([[0.7985183, -0.60118503, 0.03074534],
                                                                              [0.60056761, 0.79910241, 0.02745598],
                                                                              [-0.04107472, -0.00345939, 0.9991501]])],
        ['twostairpeg', np.array([-65.00726582, 1.98013074, 9.67320275]), np.array([[0.79838339, -0.60206015, 0.01036888],
                                                                              [0.60140611, 0.79813501, 0.0359356],
                                                                              [-0.02991115, -0.02245445, 0.99930026]])],
        ['twostairpeg', np.array([-65.96913047, 0.98967153, 9.37291531]), np.array([[0.80590601, -0.59195208, -0.01041524],
                                                                              [0.59196486, 0.80538711, 0.03047976],
                                                                              [-0.0096542, -0.03072921, 0.99948105]])],
        ['twostairpeg', np.array([-65.55095797, 1.14921603, 9.37868225]), np.array([[0.80294605, -0.59599053, -0.00854547],
                                                                              [0.59588841, 0.80230781, 0.03491515],
                                                                              [-0.01395299, -0.03312713, 0.99935373]])],
        ['twostairpeg', np.array([-65.13238472, 2.20441221, 9.53754527]), np.array([[0.79695662, -0.60399759, -0.00685714],
                                                                              [0.60380831, 0.79629221, 0.03652734],
                                                                              [-0.01660216, -0.03325106, 0.99930908]])],
        ['twostairpeg', np.array([-65.7272957, 2.0385146, 9.16188069]), np.array([[0.79679814, -0.60294253, -0.03966321],
                                                                            [0.60424305, 0.79527081, 0.04934165],
                                                                            [0.0017928, -0.06328151, 0.99799416]])],
        ['twostairpeg', np.array([-65.17786105, 1.77876588, 9.84877717]), np.array([[0.7939977, -0.60716148, -0.03037631],
                                                                              [0.60791842, 0.79286386, 0.04244887],
                                                                              [-0.00168897, -0.05217059, 0.99863674]])],
        ['twostairpeg', np.array([-65.06923528, 1.5353837, 10.34634346]), np.array([[0.79633377, -0.6047815, -0.00959321],
                                                                              [0.60468254, 0.7956217, 0.03667653],
                                                                              [-0.01454872, -0.03500761, 0.99928112]])],
        ['twostairpeg', np.array([-65.15766978, 1.20397861, 9.79124589]), np.array([[0.83485656, -0.55046393, 0.0020201],
                                                                              [0.55035265, 0.83475152, 0.01737524],
                                                                              [-0.01125065, -0.01339398, 0.99984692]])],
        ['twostairpeg', np.array([-64.86678788, 0.26218932, 9.76702229]),
         np.array([[8.90250830e-01, -4.55452813e-01, 4.05512607e-03],
                [4.55444099e-01, 8.90259600e-01, 2.90684674e-03],
                [-4.93405337e-03, -7.40933524e-04, 9.99987501e-01]])],
        ['twostairpeg', np.array([-64.78565357, -1.29180829, 10.18531014]), np.array([[0.92935851, -0.36896119, 0.01266899],
                                                                                [0.36912499, 0.92926101, -0.01485615],
                                                                                [-0.00629143, 0.0184831, 0.99980937]])],
        ['twostairpeg', np.array([-65.27536488, -2.49941707, 10.2783622]), np.array([[0.96322685, -0.26822971, 0.01571702],
                                                                               [0.26834155, 0.96330833, -0.00546379],
                                                                               [-0.01367481, 0.00948036, 0.9998615]])],
        ['twostairpeg', np.array([-66.33052777, -2.91133317, 9.83998774]), np.array([[0.98097749, -0.19403328, 0.00585487],
                                                                               [0.19408439, 0.98093367, -0.0100231],
                                                                               [-0.0037984, 0.0109688, 0.99993255]])],
        ['twostairpeg', np.array([-65.86662655, -3.48502495, 10.44975835]), np.array([[0.98869576, -0.14922727, 0.01455921],
                                                                                [0.14910791, 0.98878017, 0.00897051],
                                                                                [-0.01573448, -0.00669816,
                                                                                 0.99985382]])],
        ['twostairpeg', np.array([-66.00341317, -2.70007784, 10.4493319]), np.array([[0.99732992, -0.07169608, 0.01389146],
                                                                               [0.07178591, 0.9974016, -0.00607971],
                                                                               [-0.01341948, 0.0070607, 0.99988503]])],
        ['twostairpeg', np.array([-66.08362223, -2.36916293, 10.60755778]),
         np.array([[9.99741531e-01, 1.43747326e-02, 1.76117924e-02],
                [-1.43636687e-02, 9.99896533e-01, -7.54611268e-04],
                [-1.76207929e-02, 5.01471398e-04, 9.99844619e-01]])],
        ['twostairpeg', np.array([-66.51597572, -3.06240849, 10.20771623]), np.array([[0.99958024, 0.02877581, 0.00339309],
                                                                                [-0.02879167, 0.99957429, 0.00471828],
                                                                                [-0.00325587, -0.00481396,
                                                                                 0.99998312]])],
        ['twostairpeg', np.array([-66.23090473, -1.78946312, 10.17691788]), np.array([[0.99957327, 0.02098394, 0.02032126],
                                                                                [-0.02095131, 0.99977882, -0.00181748],
                                                                                [-0.02035492, 0.00139104,
                                                                                 0.99979185]])],
        ['twostairpeg', np.array([-66.34047943, -1.98123495, 10.2467643]),
         np.array([[9.99131001e-01, 3.93723658e-02, 1.36851170e-02],
                [-3.93558751e-02, 9.99224126e-01, -1.47500623e-03],
                [-1.37325969e-02, 9.35273798e-04, 9.99905196e-01]])],
        ['twostairpeg', np.array([-65.44180849, -1.14410812, 11.30166172]), np.array([[0.99837763, 0.05357023, 0.01930079],
                                                                                [-0.05360175, 0.99856175, 0.00111872],
                                                                                [-0.01921309, -0.00215146,
                                                                                 0.99981305]])],
        ['twostairpeg', np.array([-65.79791863, -0.81741708, 10.61929245]), np.array([[0.99775818, 0.05955872, 0.03052032],
                                                                                [-0.05963538, 0.99821897, 0.00160673],
                                                                                [-0.03037028, -0.00342313,
                                                                                 0.99953279]])],
        ['twostairpeg', np.array([-65.85453115, -0.73739203, 10.75701632]), np.array([[0.99953001, 0.0274941, 0.01356012],
                                                                                [-0.02741853, 0.99960751, -0.00572805],
                                                                                [-0.01371231, 0.0053536, 0.99989158]])],
        ['twostairpeg', np.array([-66.03870912, -0.23251076, 10.74679743]), np.array([[0.99978802, -0.00658181, 0.01950825],
                                                                                [0.00661116, 0.99997705, -0.0014395],
                                                                                [-0.01949831, 0.00156817,
                                                                                 0.99980871]])],
        ['twostairpeg', np.array([-65.99961869, -0.12461466, 10.71346768]), np.array([[0.99992156, -0.00624859, 0.01085497],
                                                                                [0.00626183, 0.99997966, -0.00118494],
                                                                                [-0.01084733, 0.00125297, 0.99994028]])]

    ]

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

    rollangle = []
    for pose in demopathlist:
        relpos1 = pose[1]
        relrot1 = pose[2]
        relmat1 = base.pg.npToMat4(relrot1, relpos1)
        virtualgoalpos1, virtualgoalrot1 = setposrpy(relmat1)
        rollangle.append(virtualgoalrot1[2])
        # print("roll angle =", virtualgoalrot1[1])

    # print(len(rollangle))
    # print(rollangle)

    x = np.arange(0.0, len(rollangle)*0.1, 0.1)
    print(len(x))
    # x = np.delete(x, [len(x)])
    plt.figure(1)
    plt.subplot(131)
    # origin = plt.scatter(x[3:-3], rollangle[3:-3], c="red", label="origin data")
    origin = plt.scatter(x[0:-2], rollangle[0:-2], c="red", label="origin data")
    ## use curve fitting
    # z = np.polyfit(x, rollangle, 15)
    # p = np.poly1d(z)
    # print(p)
    # fitting = plt.plot(x[23:-3], p(x[23:-3]), "-", label="fitting curve")
    # q = p.deriv()
    # qq = q.deriv()
    # # print(q)
    # plt.plot(x[23:-3], q(x[23:-3]), label="1st deriv")
    # plt.plot(x[23:-3], qq(x[23:-3]), label="2nd deriv")

    ## use interpolation
    y_spl = UnivariateSpline(x, rollangle, s=10, k=5)
    # plt.plot(x[3:-3], y_spl(x[3:-3]))
    plt.plot(x[0:-2], y_spl(x[0:-2]), label="Interpolation")
    plt.legend()
    plt.grid()
    plt.title("roll angles of key poses")
    plt.xlabel('time(s)')
    plt.ylabel('roll angle(°)')

    plt.subplot(132)
    y_spl_1d = y_spl.derivative(n=1)
    # plt.plot(x[3:-3], y_spl_1d(x[3:-3]), label="1st derivative")
    plt.plot(x[0:-2], y_spl_1d(x[0:-2]), "g", label="1st derivative")
    plt.grid()
    plt.title("1st derivative of roll angle of key poses")
    plt.xlabel('time(s)')
    plt.ylabel('1st derivative of roll angles(°/s)')

    plt.subplot(133)
    y_spl_2d = y_spl.derivative(n=2)
    # plt.plot(x[3:-3], y_spl_2d(x[3:-3]), "--", label="2nd derivative")
    plt.plot(x[0:-2], y_spl_2d(x[0:-2]), "y", label="2nd derivative")
    plt.grid()
    plt.title("2nd derivative of roll angle of key poses")
    plt.xlabel('time(s)')
    plt.ylabel('2nd derivative of roll angles(°/s^2)')
    # my_x_ticks = np.arange(0, 2.0, 0.5)
    # my_y_ticks = np.arange(-70, 70, 5)
    # plt.xticks(my_x_ticks)
    # plt.yticks(my_y_ticks)

    deriv = []
    secderiv = []
    deriv1 = []
    secderiv1 = []
    for time in range(0, len(x)-2):
    # for time in range(len(x)):
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
        k = 2
        # k = 3
        rearrposeidlist1.append(demopathlist[num+k][1])
        rearrposeidlist1.append(demopathlist[num+k][2][0])
        rearrposeidlist1.append(demopathlist[num+k][2][1])
        rearrposeidlist1.append(demopathlist[num+k][2][2])
    print(rearrposeidlist1)  # Rearranged demo poses according to derivative
    for num in poseidlist2:
        k = 2
        # k = 3
        rearrposeidlist2.append(demopathlist[num+k][1])
        rearrposeidlist2.append(demopathlist[num+k][2][0])
        rearrposeidlist2.append(demopathlist[num+k][2][1])
        rearrposeidlist2.append(demopathlist[num+k][2][2])
    print(rearrposeidlist2)  # Rearranged demo poses according to 2nd derivative
    writefile(rearrposeidlist1, "Rearrangedpegpath(1st deriv).txt")
    writefile(rearrposeidlist2, "Rearrangedpegpath(2nd deriv).txt")

    # errorlist = []
    # coeflist = []
    # for m in range(19):
    #     z = np.polyfit(x, rollangle, m)
    #     p = np.poly1d(z)
    #     print(p)
    #     coefficient_of_dermination = r2_score(rollangle, p(x))
    #     coeflist.append(coefficient_of_dermination)
    #     print("coefficient_of_dermination =", coefficient_of_dermination)
    #     error = 0
    #     for time in x:
    #         # print(time)
    #         time_id = np.where(x == time)[0][0]
    #         error += pow(rollangle[time_id] - p(time), 2)
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
    # z = np.polyfit(x, rollangle, m_chosen)
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
