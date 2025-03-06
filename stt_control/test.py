import matplotlib.pyplot as plt

fig, axs = plt.subplots(2, 2, figsize=(12, 10))

axs[0, 0].plot([1,2,4,5,6],[1,2,3,5,6])
axs[0, 1].plot([1,2,4,5,6],[1,2,3,5,6])
plt.show()