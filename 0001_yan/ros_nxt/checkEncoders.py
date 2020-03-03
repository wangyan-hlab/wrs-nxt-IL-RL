import rpyc
import pickle

# # nextage-left
conn = rpyc.connect("10.0.1.102", port=15010)
# # nextage-right
# conn = rpyc.connect("10.0.1.74", port=15010)
conn.root.checkEncoders()
conn.root.goInitial()
