import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Ler os pontos do arquivo CSV
x = []
y = []
z = []

with open('trajetoria_circulo.csv', 'r') as csvfile:
    reader = csv.DictReader(csvfile)
    for row in reader:
        x.append(float(row['x']))
        y.append(float(row['y']))
        z.append(float(row['z']))

# Plotar a trajetória
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x, y, z, marker='o')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Trajetória Circular do UR3')
plt.savefig('trajetoria_circulo.png')
plt.show() 