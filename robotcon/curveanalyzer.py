import matplotlib.pyplot as plt
import numpy as np
import yaml

with open('outfiletraj.txt', 'r') as fp:
    numikrmsmp = yaml.load(fp)

# polynomial
numikrmsmpupperbody = []
for numikr in numikrmsmp:
    print(numikr[:23])
    numikrmsmpupperbody.append(numikr[:23])
numikrmsmpupperbody = np.array(numikrmsmpupperbody)
plt.plot(numikrmsmpupperbody[:,21])
plt.plot(numikrmsmpupperbody[:,22])
# plt.plot(range(0, len(numikrmsmpupperbody), 10), numikrmsmpupperbody[::10,3], '-o')
# plt.plot(range(0, len(numikrmsmpupperbody), 10), numikrmsmpupperbody[::10,4], '-o')
# plt.plot(range(0, len(numikrmsmpupperbody), 10), numikrmsmpupperbody[::10,5], '-o')
# plt.plot(range(0, len(numikrmsmpupperbody), 10), numikrmsmpupperbody[::10,6], '-o')
# plt.plot(range(0, len(numikrmsmpupperbody), 10), numikrmsmpupperbody[::10,7], '-o')
# plt.plot(range(0, len(numikrmsmpupperbody), 10), numikrmsmpupperbody[::10,8], '-o')
# plt.plot(range(0, len(numikrmsmpupperbody), 10), numikrmsmpupperbody[::10,9], '-o')
# plt.plot(range(0, len(numikrmsmpupperbody), 10), numikrmsmpupperbody[::10,10], '-o')
# plt.plot(range(0, len(numikrmsmpupperbody), 10), numikrmsmpupperbody[::10,11], '-o')
# plt.plot(range(0, len(numikrmsmpupperbody), 10), numikrmsmpupperbody[::10,12], '-o')
# plt.plot(range(0, len(numikrmsmpupperbody), 10), numikrmsmpupperbody[::10,21], '-o')
# plt.plot(range(0, len(numikrmsmpupperbody), 10), numikrmsmpupperbody[::10,22], '-o')
#
# motion = np.array(json.load(open('outfiletrajq.txt','r')))
# print(np.array(motion).shape)
#
# # plt.plot(motion[:,3])
# # plt.plot(motion[:,4])
# # plt.plot(motion[:,5])
# # plt.plot(motion[:,6])
# # plt.plot(motion[:,7])
# # plt.plot(motion[:,8])
# # plt.plot(motion[:,9])
# # plt.plot(motion[:,10])
# # plt.plot(motion[:,11])
# # plt.plot(motion[:,12])
# plt.plot(motion[:,21])
# plt.plot(motion[:,22])
# plt.show()


# robotmotion = []
# for line in open('robotmotion.pos', 'r'):
#     itemList = line.split(' ')[1:-1]
#     itemList = map(float, itemList)
#     robotmotion.append(itemList)
# robotmotion = np.array(robotmotion)
# plt.plot(robotmotion[:,26])
# plt.plot(robotmotion[:,29])
# plt.plot(robotmotion[:,44]*100)
# plt.plot(robotmotion[:,47]*100)
plt.show()