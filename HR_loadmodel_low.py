from __future__ import absolute_import, division, print_function
import pandas as pd
import seaborn as sns
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
import os
from tensorflow.keras.models import load_model
import pickle

this_dir, this_filename = os.path.split(__file__)
column_names_low = ['sc_px','sc_py','sc_pz','sc_prx','sc_pry','sc_prz',
                    'a_px','a_py','a_pz','a_prx','a_pry','a_prz','a_fx','a_fy','a_fz','a_frx','a_fry','a_frz',
                    'sl_px','sl_py','sl_pz','sl_prx','sl_pry','sl_prz']
raw_dataset_low = pd.read_csv(os.path.join(this_dir, "document", "Data_low.csv"), names=column_names_low, na_values="?", comment='\t', sep=",", skipinitialspace=True)
dataset_low = raw_dataset_low.copy()

dataset_low.isna().sum()
dataset_low = dataset_low.dropna()

train_dataset_low = dataset_low.sample(frac=0.8, random_state=0)
test_dataset_low = dataset_low.drop(train_dataset_low.index)
#
# sns.pairplot(train_dataset_low[['sc_px','sc_py','sc_pz']], diag_kind="kde")
#
train_stats_low = train_dataset_low.describe()
for string in ['a_px','a_py','a_pz','a_prx','a_pry','a_prz','a_fx','a_fy','a_fz','a_frx','a_fry','a_frz']:
    train_stats_low.pop(string)
train_stats_low = train_stats_low.transpose()

train_labels_low = train_dataset_low.copy()
test_labels_low = test_dataset_low.copy()
for string in ['sc_px','sc_py','sc_pz','sc_prx','sc_pry','sc_prz','sl_px','sl_py','sl_pz','sl_prx','sl_pry','sl_prz']:
    train_labels_low.pop(string)
    test_labels_low.pop(string)
for string in ['a_px', 'a_py', 'a_pz', 'a_prx', 'a_pry', 'a_prz', 'a_fx', 'a_fy', 'a_fz', 'a_frx', 'a_fry', 'a_frz']:
    train_dataset_low.pop(string)
    test_dataset_low.pop(string)

# with open(os.path.join(this_dir, "nnmodel","low_train_data_mean_std.pickle"), "wb") as f:
#     pickle.dump([train_stats_low['mean'], train_stats_low['std']], f)

with open(os.path.join(this_dir, "nnmodel","low_train_data_mean_std.pickle"), "rb") as f:
    train_mean, train_std = pickle.load(f)

def norm(x):
  return (x - train_mean)/train_std

normed_train_data_low = norm(train_dataset_low)
normed_test_data_low = norm(test_dataset_low)
print(normed_test_data_low.loc[[4]].to_string())

# load model
model = load_model(os.path.join(this_dir, "nnmodel", "low_model.h5"))
model.summary()

loss, mae, mse = model.evaluate(normed_test_data_low, test_labels_low, verbose=1)
print("loss = {}, mae = {}, mse = {}".format(loss, mae, mse))

a = model.predict([[-2.150169, 1.391643, 2.464242, -0.679618, 0.699722, 0.179538,
                    -2.301596, 1.377005, 2.713695, -0.895146, 0.774518, 0.163099]])
print(a)