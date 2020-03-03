import rpyc
import pickle
import time

# nextage-left
conn = rpyc.connect("10.0.1.102", port=15010)
# # nextage-right
# conn = rpyc.connect("10.0.1.74", port=15010)

# conn.root.attachHandtoollft()
# conn.root.attachHandtoolrgt()

# conn.root.closeGripperlft()
# conn.root.closeGripperrgt()

# conn.root.openGripperlft()
# conn.root.openGripperrgt()

# conn.root.ejectHandtoollft()
# conn.root.ejectHandtoolrgt()

# conn.root.drawinAirhandlft()
# conn.root.drawinAirhandrgt()

# conn.root.keepAirhandlft()
# conn.root.keepAirhandrgt()

# conn.root.releaseAirhandlft()
# conn.root.releaseAirhandrgt()

# # port 24 is lft, and port 19 is rgt
# # when the hand is attached, set port19/24 to 1 before detaching it
# # but if you set the value to 1 without the hand attached, the air will blow out
indices = pickle.dumps([18])     #set the value to 1
assignments = pickle.dumps([18, 23])
conn.root.dioWriter(indices, assignments)

