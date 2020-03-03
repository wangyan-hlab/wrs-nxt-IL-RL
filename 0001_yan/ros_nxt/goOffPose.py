import rpyc
import pickle
import time

# nextage-left
conn = rpyc.connect("10.0.1.102", port=15010)
# nextage-right
# conn = rpyc.connect("10.0.1.74", port=15010)

conn.root.goOffPose()