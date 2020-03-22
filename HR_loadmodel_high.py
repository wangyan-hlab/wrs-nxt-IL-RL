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
column_names_high = ['sc_px','sc_py','sc_pz','sc_prx','sc_pry','sc_prz',
                    'sl_px','sl_py','sl_pz','sl_prx','sl_pry','sl_prz',
                    'sh_px','sh_py','sh_pz','sh_prx','sh_pry','sh_prz']
raw_dataset_high = pd.read_csv(os.path.join(this_dir, "document", "Data_high.csv"), names=column_names_high, na_values="?", comment='\t', sep=",", skipinitialspace=True)
dataset_high = raw_dataset_high.copy()

dataset_high.isna().sum()
dataset_high = dataset_high.dropna()

train_dataset_high = dataset_high.sample(frac=0.8, random_state=0)
test_dataset_high = dataset_high.drop(train_dataset_high.index)
#
# sns.pairplot(train_dataset_low[['sc_px','sc_py','sc_pz']], diag_kind="kde")
#
train_stats_high= train_dataset_high.describe()
for string in ['sl_px','sl_py','sl_pz','sl_prx','sl_pry','sl_prz']:
    train_stats_high.pop(string)
train_stats_high = train_stats_high.transpose()

train_labels_high = train_dataset_high.copy()
test_labels_high = test_dataset_high.copy()
for string in ['sc_px','sc_py','sc_pz','sc_prx','sc_pry','sc_prz','sh_px','sh_py','sh_pz','sh_prx','sh_pry','sh_prz']:
    train_labels_high.pop(string)
    test_labels_high.pop(string)
for string in ['sl_px','sl_py','sl_pz','sl_prx','sl_pry','sl_prz']:
    train_dataset_high.pop(string)
    test_dataset_high.pop(string)

# with open(os.path.join(this_dir, "nnmodel","high_train_data_mean_std.pickle"), "wb") as f:
#     pickle.dump([train_stats_high['mean'], train_stats_high['std']], f)

with open(os.path.join(this_dir, "nnmodel","high_train_data_mean_std.pickle"), "rb") as f:
    train_mean, train_std = pickle.load(f)

def norm(x):
  return (x - train_mean)/train_std

normed_train_data_high = norm(train_dataset_high)
normed_test_data_high = norm(test_dataset_high)
print(normed_test_data_high.loc[[1]].to_string())

# load model
model = load_model(os.path.join(this_dir, "nnmodel", "high_model.h5"))
model.summary()

loss, mae, mse = model.evaluate(normed_test_data_high, test_labels_high, verbose=1)
print("loss = {}, mae = {}, mse = {}".format(loss, mae, mse))

sl = model.predict([[-2.094224, 1.355269, 2.417076, -0.633914, 0.648896, 0.158601,
                     -2.687091, 1.462195, 3.409983, -0.766682, 0.768667, 0.179917]])
print(sl)