import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("result.csv", header=None)
#%%
x = df.loc[19::-1, 11].to_numpy()
y = df.loc[19::-1, 7].to_numpy()
plt.figure()
plt.plot(y, label="length")

#%%
x = df.loc[39:20:-1, 12].to_numpy()
y = df.loc[39:20:-1, 7].to_numpy()

plt.plot(y, label="width")

#%%
x = df.loc[59:40:-1, 13].to_numpy()
y = df.loc[59:40:-1, 7].to_numpy()

plt.plot(y, label="height")

#%%
x = df.loc[79:60:-1, 14].to_numpy()
y = df.loc[79:60:-1, 7].to_numpy()

plt.plot(y, label="mesh resolution")

plt.plot([9, 9], [0, 5], "--")

plt.ylim(0, 0.5)
plt.xlim(0, 19)

plt.legend(loc='upper right')

num_sample = len(y)
plt.xticks([0, 4.75, 4.75*2, 4.75*3, 4.75*4],
           ["0% \n min", "25%", "50%", "75%", "100% \n max"])
plt.show()

