import matplotlib.pyplot as plt

x_times = []
x_values = []
u_times = []
u_values = []

# parse and load values from text files
opt_u_file = open("opt_u.txt")
opt_x_file = open("opt_x.txt")
x_samples = opt_x_file.readlines()
u_samples = opt_u_file.readlines()

for sample in x_samples:
    x_times.append(float(sample.split(" ")[0]))
    n = len(sample.split(" ")) - 2
    x_value = []
    for i in range(n):
        x_value.append(float(sample.split(" ")[1 + i]))
    x_values.append(x_value)

for sample in u_samples:
    u_times.append(float(sample.split(" ")[0]))
    m = len(sample.split(" ")) - 2
    u_value = []
    for i in range(m):
        u_value.append(float(sample.split(" ")[1 + i]))
    u_values.append(u_value)
    
# plot 
plt.subplot(1,2,1)
plt.plot(x_times, x_values, marker='.')
plt.title("optimal state trajectory")
plt.xlabel("time (s)")
plt.grid(True, color='grey', linewidth=.5, linestyle=':')
xlegends = []
for i in range(n):
    xlegends.append('x_' + str(i))
plt.legend(xlegends)

plt.subplot(1,2,2)
plt.plot(u_times, u_values, marker='.')
plt.title("optimal control policy")
plt.xlabel("time (s)")
plt.grid(True, color='grey', linewidth=.5, linestyle=':')
ulegends = []
for i in range(m):
    ulegends.append('u_' + str(i))
plt.legend(ulegends)

plt.show()