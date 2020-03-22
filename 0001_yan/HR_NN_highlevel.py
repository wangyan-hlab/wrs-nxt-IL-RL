from __future__ import absolute_import, division, print_function
import pandas as pd
import seaborn as sns
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
import os
import matplotlib.pyplot as plt

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

def norm(x):
  return (x - train_stats_high['mean'])/train_stats_high['std']

normed_train_data_high = norm(train_dataset_high)
normed_test_data_high = norm(test_dataset_high)

def plot_history(history):
  hist = pd.DataFrame(history.history)
  hist['epoch'] = history.epoch
  plt.figure()
  plt.xlabel('Epoch')
  plt.ylabel('Mean Abs Error [MPG]')
  plt.plot(hist['epoch'],hist['mae'],label='Train Error')
  plt.plot(hist['epoch'],hist['val_mae'],label='Val Error')
  plt.legend()
  plt.ylim([0,5])

  plt.figure()
  plt.xlabel('Epoch')
  plt.ylabel('Mean Square Error [$MPG^2$]')
  plt.plot(hist['epoch'],hist['mse'],label='Train Error')
  plt.plot(hist['epoch'],hist['val_mse'],label='Val Error')
  plt.legend()
  plt.ylim([0,20])

model = keras.Sequential([
            layers.Dense(256,activation=tf.nn.relu,input_shape=[len(train_dataset_high.keys())]),
            layers.Dense(256,activation=tf.nn.relu),
            layers.Dense(6)
])
# optimizer = tf.keras.optimizers.RMSprop(0.005)
optimizer = tf.keras.optimizers.Adam(0.005)
model.compile(loss='mse',
            optimizer=optimizer,
            metrics=['mae','mse'])

model.summary()

early_stop = keras.callbacks.EarlyStopping(monitor='val_loss', patience=10)
import time
start_time = time.time()
history = model.fit(normed_train_data_high, train_labels_high, batch_size=128, epochs=100,
                    validation_split=0.2, verbose=1, callbacks=[early_stop])
print("Training took {} seconds".format(time.time()-start_time))
hist = pd.DataFrame(history.history)
hist['epoch'] = history.epoch
print(hist)
plot_history(history)
#
# loss, mae, mse = model.evaluate(normed_test_data, test_labels, verbose=0)
# print('Testing set Mean Abs Error: {:5.2f} MPG'.format(mae))
#
# test_predictions = model.predict(normed_test_data).flatten()
# plt.figure()
# plt.scatter(test_labels, test_predictions)
# plt.xlabel('True values [MPG]')
# plt.ylabel('Predictions [MPG]')
# plt.axis('equal')
# plt.axis('square')
# plt.xlim([0,plt.xlim()[1]])
# plt.ylim([0,plt.ylim()[1]])
# _ = plt.plot([-100, 100], [-100, 100])

# error = test_predictions - test_labels
# plt.figure()
# plt.hist(error, bins=25)
# plt.xlabel('Predicton Error [MPG]')
# _ = plt.ylabel('Count')
loss, mae, mse = model.evaluate(normed_test_data_high, test_labels_high, verbose=1)
print("loss = {}, mae = {}, mse = {}".format(loss, mae, mse))
# plt.show()

# save the model
model.save(os.path.join(this_dir, "nnmodel", "high_model.h5"))
print("Saved model to disk")